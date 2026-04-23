// ================================================================
//  MAP DISPLAY v2.0  —  ESP32-S3 + ST7789 + SD + BLE GPS
//
//  Core 0 : BLE state machine (scan → connect → receive notify)
//  Core 1 : Tile rendering + SD loading + button handling (loop)
//
//  BLE data format from transmitter: "LAT,LON,HDG"
//  Example: "19.050123,72.901234,270.50"
//
//  Button map:
//    BTN1 (N) / BTN2 (S) / BTN3 (W) / BTN4 (E)  — manual pan
//    BTN1 + BTN2 held 1.5 s                       — zoom in
//    BTN3 + BTN4 held 1.5 s                       — zoom out
//    BTN1+BTN2+BTN3+BTN4 simultaneously            — re-lock GPS
// ================================================================

#include <lgfx_config.hpp>
#include <SdFat.h>
#include <SPI.h>
#include "esp_heap_caps.h"
#include <BLEDevice.h>
#include <BLEClient.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <math.h>

// ──────────────────────────────────────────────────────────────
//  BLE CONFIG  — must match your GPS transmitter firmware
// ──────────────────────────────────────────────────────────────
#define GPS_SERVICE_UUID   "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define GPS_CHAR_UUID      "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define TARGET_BLE_NAME    "GPS_TRACKER"   // change to your device name

// ──────────────────────────────────────────────────────────────
//  DISPLAY (configured via lgfx_config.hpp)
// ──────────────────────────────────────────────────────────────
LGFX         lcd;
LGFX_Sprite  sprite(&lcd);

// ──────────────────────────────────────────────────────────────
//  SD CARD  (HSPI bus — isolated from display SPI)
// ──────────────────────────────────────────────────────────────
#define SD_SCK   13
#define SD_MOSI  5
#define SD_MISO  14
#define SD_CS    4

SPIClass spiSD(HSPI);
SdFat    sd;

// ──────────────────────────────────────────────────────────────
//  BUTTONS
// ──────────────────────────────────────────────────────────────
#define BTN1  3   // North
#define BTN2  2   // South
#define BTN3  8   // West
#define BTN4  7   // East

// ──────────────────────────────────────────────────────────────
//  MAP PARAMETERS
// ──────────────────────────────────────────────────────────────
#define TILE_SIZE   256
#define SCREEN_W    240
#define SCREEN_H    320
#define TILE_COLS   2     // 2 tiles wide  (240 < 2×256 ✓)
#define TILE_ROWS   3     // 3 tiles tall  (320 > 2×256 − worst case offset)
#define ZOOM_MIN    12
#define ZOOM_MAX    18

// ──────────────────────────────────────────────────────────────
//  PSRAM TILE CACHE  2×3 = 6 tiles × 128 KB = 768 KB
// ──────────────────────────────────────────────────────────────
uint16_t* tileCache[TILE_COLS][TILE_ROWS];

// DMA-safe line buffer for SD reads
uint16_t* lineBuffer;

// ──────────────────────────────────────────────────────────────
//  MAP STATE  (owned by Core 1 / loop — no mutex needed)
// ──────────────────────────────────────────────────────────────
double mapLat     = 19.05;   // manual-pan center
double mapLon     = 72.90;
int    zoom       = 16;
int    cacheTileX = -999;    // cached tile origin
int    cacheTileY = -999;
bool   followGPS  = true;    // auto-center on BLE GPS position

// ──────────────────────────────────────────────────────────────
//  BUTTON COMBO STATE
// ──────────────────────────────────────────────────────────────
struct Combo { bool active = false; uint32_t t0 = 0; bool fired = false; };
Combo cZoomIn;    // BTN1+BTN2
Combo cZoomOut;   // BTN3+BTN4

// ──────────────────────────────────────────────────────────────
//  SHARED GPS DATA  (written Core 0, read Core 1)
//  Access via gpsMutex
// ──────────────────────────────────────────────────────────────
struct GPSData {
  double   lat     = 19.05;
  double   lon     = 72.90;
  float    heading = 0.0f;
  bool     valid   = false;
  uint32_t updated = 0;
};

GPSData           gpsShared;
SemaphoreHandle_t gpsMutex;

// ──────────────────────────────────────────────────────────────
//  BLE STATE MACHINE  (Core 0)
// ──────────────────────────────────────────────────────────────
enum BLEState { BLE_SCANNING, BLE_FOUND, BLE_CONNECTING,
                BLE_CONNECTED, BLE_WAIT_RETRY };
volatile BLEState bleState     = BLE_SCANNING;
volatile bool     bleConnected = false;

BLEClient*               bleClient  = nullptr;
BLERemoteCharacteristic* gpsChar    = nullptr;
BLEAddress*              serverAddr = nullptr;

// ──────────────────────────────────────────────────────────────
//  BLE NOTIFICATION CALLBACK  (runs inside BLE stack, Core 0)
//  Parses "LAT,LON,HDG" and stores under mutex
// ──────────────────────────────────────────────────────────────
static void onGPSNotify(
    BLERemoteCharacteristic*, uint8_t* data, size_t len, bool)
{
  if (len == 0 || len > 63) return;

  char buf[64];
  memcpy(buf, data, len);
  buf[len] = '\0';

  double newLat, newLon;
  float  newHdg = 0.0f;

  // Accepts both "lat,lon" and "lat,lon,heading"
  if (sscanf(buf, "%lf,%lf,%f", &newLat, &newLon, &newHdg) >= 2) {
    if (xSemaphoreTake(gpsMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      gpsShared.lat     = newLat;
      gpsShared.lon     = newLon;
      gpsShared.heading = newHdg;
      gpsShared.valid   = true;
      gpsShared.updated = millis();
      xSemaphoreGive(gpsMutex);
    }
  }
}

// ──────────────────────────────────────────────────────────────
//  BLE: SCAN CALLBACK
// ──────────────────────────────────────────────────────────────
class ScanCB : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice dev) override {
    bool byName    = (dev.getName() == TARGET_BLE_NAME);
    bool bySvc     = dev.haveServiceUUID() &&
                     dev.isAdvertisingService(BLEUUID(GPS_SERVICE_UUID));
    if (byName || bySvc) {
      BLEDevice::getScan()->stop();
      delete serverAddr;
      serverAddr = new BLEAddress(dev.getAddress());
      bleState   = BLE_FOUND;
      Serial.printf("✅ Found GPS Tracker: %s\n",
                    dev.getAddress().toString().c_str());
    }
  }
};

// ──────────────────────────────────────────────────────────────
//  BLE: CLIENT CALLBACKS
// ──────────────────────────────────────────────────────────────
class ClientCB : public BLEClientCallbacks {
  void onConnect(BLEClient*) override {
    bleConnected = true;
    bleState     = BLE_CONNECTED;
    Serial.println("🔗 BLE Connected");
  }
  void onDisconnect(BLEClient*) override {
    bleConnected = false;
    bleState     = BLE_WAIT_RETRY;
    Serial.println("🔌 BLE Disconnected — will retry");
    // Mark position stale on disconnect
    if (xSemaphoreTake(gpsMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
      gpsShared.valid = false;
      xSemaphoreGive(gpsMutex);
    }
  }
};

// ──────────────────────────────────────────────────────────────
//  BLE: CONNECT TO SERVER + SUBSCRIBE NOTIFICATIONS
// ──────────────────────────────────────────────────────────────
bool connectToBLE() {
  if (!bleClient) {
    bleClient = BLEDevice::createClient();
    bleClient->setClientCallbacks(new ClientCB());
  }

  // Clean up any lingering connection
  if (bleClient->isConnected()) {
    bleClient->disconnect();
    vTaskDelay(pdMS_TO_TICKS(400));
  }

  Serial.printf("  → Connecting to %s\n",
                serverAddr->toString().c_str());

  if (!bleClient->connect(*serverAddr)) {
    Serial.println("  ❌ connect() failed");
    return false;
  }

  BLERemoteService* svc = bleClient->getService(GPS_SERVICE_UUID);
  if (!svc) {
    Serial.println("  ❌ GPS service not found");
    bleClient->disconnect();
    return false;
  }

  gpsChar = svc->getCharacteristic(GPS_CHAR_UUID);
  if (!gpsChar) {
    Serial.println("  ❌ GPS characteristic not found");
    bleClient->disconnect();
    return false;
  }

  if (!gpsChar->canNotify()) {
    Serial.println("  ❌ Notifications not supported by server");
    bleClient->disconnect();
    return false;
  }

  gpsChar->registerForNotify(onGPSNotify);
  Serial.println("  ✅ Notifications subscribed — GPS live");
  return true;
}

// ──────────────────────────────────────────────────────────────
//  BLE TASK  (Core 0)
//  Non-blocking state machine — no busy loops, watchdog-safe
// ──────────────────────────────────────────────────────────────
void bleTask(void* param) {
  BLEScan* scan = BLEDevice::getScan();
  scan->setAdvertisedDeviceCallbacks(new ScanCB(), true);
  scan->setActiveScan(true);
  scan->setInterval(150);
  scan->setWindow(140);

  uint32_t retryAt   = 0;
  uint32_t retryWait = 5000;   // exponential backoff
  uint32_t nextScan  = 0;

  Serial.println("🔵 BLE task running on Core 0");

  for (;;) {
    switch (bleState) {

      // ── Periodic scan ─────────────────────────────────────
      case BLE_SCANNING:
        if (millis() > nextScan) {
          Serial.println("🔍 Scanning for GPS Tracker...");
          scan->clearResults();
          scan->start(4, false);   // 4 s async scan
          nextScan = millis() + 7000;  // re-attempt every 7 s
        }
        break;

      // ── Device found — move to connect ────────────────────
      case BLE_FOUND:
        bleState = BLE_CONNECTING;
        break;

      // ── Attempt connection ─────────────────────────────────
      case BLE_CONNECTING:
        if (connectToBLE()) {
          retryWait = 5000;   // reset backoff on success
        } else {
          bleState  = BLE_WAIT_RETRY;
          retryAt   = millis() + retryWait;
          retryWait = min(retryWait * 2, (uint32_t)30000); // max 30 s
        }
        break;

      // ── Connected: notifications handle everything ─────────
      case BLE_CONNECTED:
        if (!bleConnected) bleState = BLE_WAIT_RETRY;
        break;

      // ── Wait before retry ──────────────────────────────────
      case BLE_WAIT_RETRY:
        if (millis() > retryAt) {
          bleState  = BLE_SCANNING;
          nextScan  = 0;   // scan immediately
        }
        break;
    }

    vTaskDelay(pdMS_TO_TICKS(200));  // MUST yield — prevents watchdog reset
  }
}

// ──────────────────────────────────────────────────────────────
//  TILE MATH
// ──────────────────────────────────────────────────────────────
double lon2worldX(double lon, int z) {
  return (lon + 180.0) / 360.0 * (double)(1 << z) * TILE_SIZE;
}

double lat2worldY(double lat, int z) {
  double r = lat * (PI / 180.0);
  return (1.0 - log(tan(r) + 1.0 / cos(r)) / PI) / 2.0
         * (double)(1 << z) * TILE_SIZE;
}

// ──────────────────────────────────────────────────────────────
//  HARDWARE INIT
// ──────────────────────────────────────────────────────────────
void initPSRAM() {
  for (int x = 0; x < TILE_COLS; x++) {
    for (int y = 0; y < TILE_ROWS; y++) {
      tileCache[x][y] = (uint16_t*)ps_malloc(TILE_SIZE * TILE_SIZE * 2);
      if (!tileCache[x][y]) {
        Serial.printf("❌ PSRAM alloc fail [%d][%d]\n", x, y);
        esp_restart();
      }
    }
  }
  Serial.printf("✅ PSRAM cache: %d×%d tiles = %d KB\n",
                TILE_COLS, TILE_ROWS, TILE_COLS * TILE_ROWS * 128);
}

void initLineBuffer() {
  lineBuffer = (uint16_t*)heap_caps_malloc(TILE_SIZE * 2, MALLOC_CAP_DMA);
  if (!lineBuffer) {
    Serial.println("❌ DMA line buffer alloc failed");
    esp_restart();
  }
  Serial.println("✅ DMA line buffer ready");
}

void initSD() {
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);
  spiSD.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  SdSpiConfig cfg(SD_CS, DEDICATED_SPI, SD_SCK_MHZ(20), &spiSD);
  if (!sd.begin(cfg)) {
    Serial.println("❌ SD init failed");
    sd.printSdError(&Serial);
    esp_restart();
  }
  Serial.println("✅ SD card ready");
}

// ──────────────────────────────────────────────────────────────
//  SD TILE LOADER
// ──────────────────────────────────────────────────────────────
bool loadTile(int z, int tx, int ty, uint16_t* buf) {
  char path[64];
  snprintf(path, sizeof(path), "/maptiles/%d/%d/%d.bin", z, tx, ty);

  File f = sd.open(path);
  if (!f) return false;

  for (int row = 0; row < TILE_SIZE; row++) {
    if (f.read((uint8_t*)lineBuffer, TILE_SIZE * 2) != TILE_SIZE * 2) {
      f.close();
      return false;
    }
    memcpy(buf + row * TILE_SIZE, lineBuffer, TILE_SIZE * 2);
  }
  f.close();
  return true;
}

// Fill missing tile with dark checkerboard (visually distinct from black)
void fillMissingTile(uint16_t* buf) {
  for (int i = 0; i < TILE_SIZE * TILE_SIZE; i++) {
    int row = i / TILE_SIZE, col = i % TILE_SIZE;
    buf[i] = ((row / 16 + col / 16) & 1) ? (uint16_t)0x2945 : (uint16_t)0x18C3;
  }
}

// ──────────────────────────────────────────────────────────────
//  HEADING ARROW  (filled triangle pointing in heading direction)
// ──────────────────────────────────────────────────────────────
void drawArrow(LGFX_Sprite& spr, int cx, int cy, float hdg) {
  float rad = hdg * (PI / 180.0f);

  // Tip: 20 px forward
  int tx = cx + (int)(20.0f * sinf(rad));
  int ty = cy - (int)(20.0f * cosf(rad));

  // Left wing: +145°
  float lr = rad + 2.530f;
  int lx = cx + (int)(11.0f * sinf(lr));
  int ly = cy - (int)(11.0f * cosf(lr));

  // Right wing: −145°
  float rr = rad - 2.530f;
  int rx = cx + (int)(11.0f * sinf(rr));
  int ry = cy - (int)(11.0f * cosf(rr));

  spr.fillTriangle(tx, ty, lx, ly, rx, ry, TFT_RED);
  spr.drawTriangle(tx, ty, lx, ly, rx, ry, TFT_WHITE);
  spr.fillCircle(cx, cy, 3, TFT_WHITE);  // center dot
}

// ──────────────────────────────────────────────────────────────
//  STATUS BAR  (top 22 px, drawn over the map)
// ──────────────────────────────────────────────────────────────
void drawStatusBar(const GPSData& gps) {
  sprite.fillRect(0, 0, SCREEN_W, 22, (uint16_t)0x0000);
  sprite.setTextSize(1);

  // BLE dot: green = connected, red = searching
  sprite.fillCircle(7, 11, 4,
    bleConnected ? (uint16_t)TFT_GREEN : (uint16_t)TFT_RED);

  bool stale = gps.valid && (millis() - gps.updated > 5000);
  char buf[36];

  if (gps.valid && !stale) {
    sprite.setTextColor(TFT_WHITE);
    snprintf(buf, sizeof(buf), "%.5f %.5f", gps.lat, gps.lon);
    sprite.drawString(buf, 15, 2);
    snprintf(buf, sizeof(buf), "HDG %.0f  Z%d  %s",
             gps.heading, zoom, followGPS ? "[GPS]" : "[MAN]");
    sprite.drawString(buf, 15, 12);

  } else if (stale) {
    sprite.setTextColor(TFT_YELLOW);
    snprintf(buf, sizeof(buf), "GPS lost  Z%d  %s",
             zoom, followGPS ? "[GPS]" : "[MAN]");
    sprite.drawString(buf, 15, 6);

  } else {
    sprite.setTextColor(TFT_YELLOW);
    sprite.drawString(bleConnected ? "BLE: no fix" : "BLE: scanning",
                      15, 6);
  }
}

// ──────────────────────────────────────────────────────────────
//  BUTTON HANDLER  (Core 1)
// ──────────────────────────────────────────────────────────────
void handleButtons() {
  bool b1 = !digitalRead(BTN1);  // North
  bool b2 = !digitalRead(BTN2);  // South
  bool b3 = !digitalRead(BTN3);  // West
  bool b4 = !digitalRead(BTN4);  // East

  // ── All 4: snap back to GPS follow ───────────────────────────
  if (b1 && b2 && b3 && b4) {
    if (!followGPS) {
      followGPS = true;
      Serial.println("📍 GPS follow re-enabled");
    }
    return;
  }

  // ── BTN1+BTN2 held → zoom in ──────────────────────────────────
  if (b1 && b2 && !b3 && !b4) {
    if (!cZoomIn.active) cZoomIn = {true, millis(), false};
    if (!cZoomIn.fired && millis() - cZoomIn.t0 >= 1500) {
      if (zoom < ZOOM_MAX) {
        zoom++;
        cacheTileX = cacheTileY = -999;
        Serial.printf("🔍 Zoom %d\n", zoom);
      }
      cZoomIn.fired = true;
    }
    return;
  }
  cZoomIn.active = false;

  // ── BTN3+BTN4 held → zoom out ─────────────────────────────────
  if (b3 && b4 && !b1 && !b2) {
    if (!cZoomOut.active) cZoomOut = {true, millis(), false};
    if (!cZoomOut.fired && millis() - cZoomOut.t0 >= 1500) {
      if (zoom > ZOOM_MIN) {
        zoom--;
        cacheTileX = cacheTileY = -999;
        Serial.printf("🔭 Zoom %d\n", zoom);
      }
      cZoomOut.fired = true;
    }
    return;
  }
  cZoomOut.active = false;

  // ── Directional pan: disables GPS follow ──────────────────────
  if (b1 || b2 || b3 || b4) {
    followGPS = false;
    // Step scales with zoom so panning feels consistent
    double step = 0.00015 * pow(2.0, (double)(16 - zoom));
    if (b1) mapLat += step;
    if (b2) mapLat -= step;
    if (b3) mapLon -= step;
    if (b4) mapLon += step;
  }
}

// ──────────────────────────────────────────────────────────────
//  RENDER MAP  (Core 1)
// ──────────────────────────────────────────────────────────────
void renderMap() {
  // Snapshot GPS data with minimal lock time
  GPSData gps;
  if (xSemaphoreTake(gpsMutex, pdMS_TO_TICKS(3)) == pdTRUE) {
    gps = gpsShared;
    xSemaphoreGive(gpsMutex);
  }

  // Choose view center
  double centerLat = (followGPS && gps.valid) ? gps.lat : mapLat;
  double centerLon = (followGPS && gps.valid) ? gps.lon : mapLon;

  // World pixel of screen center
  double wX = lon2worldX(centerLon, zoom);
  double wY = lat2worldY(centerLat, zoom);

  // World pixel of top-left corner of screen
  double tlX = wX - SCREEN_W / 2.0;
  double tlY = wY - SCREEN_H / 2.0;

  // Anchor tile (top-left tile of the 2×3 grid)
  int tileX = (int)floor(tlX / TILE_SIZE);
  int tileY = (int)floor(tlY / TILE_SIZE);

  // Pixel offset of that tile's top-left within viewport
  int offX = (int)(tlX - (double)tileX * TILE_SIZE);
  int offY = (int)(tlY - (double)tileY * TILE_SIZE);

  // ── Load new tiles when viewport crosses tile boundary ────────
  if (tileX != cacheTileX || tileY != cacheTileY) {
    Serial.printf("🗂️  Load 2×3 @ (%d,%d) z%d\n", tileX, tileY, zoom);
    for (int cx = 0; cx < TILE_COLS; cx++) {
      for (int cy = 0; cy < TILE_ROWS; cy++) {
        if (!loadTile(zoom, tileX + cx, tileY + cy, tileCache[cx][cy])) {
          fillMissingTile(tileCache[cx][cy]);
        }
      }
    }
    cacheTileX = tileX;
    cacheTileY = tileY;
  }

  // ── Blit tiles to sprite ──────────────────────────────────────
  sprite.fillSprite(TFT_BLACK);
  for (int cx = 0; cx < TILE_COLS; cx++) {
    for (int cy = 0; cy < TILE_ROWS; cy++) {
      sprite.pushImage(
        cx * TILE_SIZE - offX,
        cy * TILE_SIZE - offY,
        TILE_SIZE, TILE_SIZE,
        tileCache[cx][cy]);
    }
  }

  // ── GPS / heading indicator ───────────────────────────────────
  bool stale = gps.valid && (millis() - gps.updated > 5000);

  if (followGPS && gps.valid && !stale) {
    // Arrow at screen center, pointing in heading direction
    drawArrow(sprite, SCREEN_W / 2, SCREEN_H / 2, gps.heading);

  } else if (followGPS && gps.valid && stale) {
    // Lost fix: grey circle at center
    sprite.fillCircle(SCREEN_W / 2, SCREEN_H / 2, 5, TFT_DARKGREY);
    sprite.drawCircle(SCREEN_W / 2, SCREEN_H / 2, 7, TFT_WHITE);

  } else if (!followGPS && gps.valid) {
    // Manual pan mode: project GPS position onto current viewport
    int sx = (int)(lon2worldX(gps.lon, zoom) - tlX);
    int sy = (int)(lat2worldY(gps.lat, zoom) - tlY);
    if (sx >= 8 && sx < SCREEN_W - 8 && sy >= 24 && sy < SCREEN_H - 8) {
      drawArrow(sprite, sx, sy, gps.heading);
    }
  }

  // ── Status bar on top of map ──────────────────────────────────
  drawStatusBar(gps);

  sprite.pushSprite(0, 0);
}

// ──────────────────────────────────────────────────────────────
//  SETUP
// ──────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== MAP DISPLAY v2.0 BOOT ===");

  // Mutex MUST exist before any task can take it
  gpsMutex = xSemaphoreCreateMutex();
  if (!gpsMutex) {
    Serial.println("❌ Mutex creation failed");
    esp_restart();
  }

  // Display
  lcd.init();
  lcd.setRotation(2);
  sprite.createSprite(SCREEN_W, SCREEN_H);

  // Memory + Storage
  initPSRAM();
  initLineBuffer();
  initSD();

  // Buttons
  pinMode(BTN1, INPUT_PULLUP);
  pinMode(BTN2, INPUT_PULLUP);
  pinMode(BTN3, INPUT_PULLUP);
  pinMode(BTN4, INPUT_PULLUP);

  // BLE init MUST happen in setup(), before spawning BLE task
  BLEDevice::init("MapDisplay");
  Serial.println("✅ BLE device initialized");

  // Spawn BLE task on Core 0  (stack 8 KB, priority 2)
  xTaskCreatePinnedToCore(
    bleTask,      // function
    "BLE_TASK",   // name
    8192,         // stack bytes — BLE needs ≥6 KB
    nullptr,      // param
    2,            // priority
    nullptr,      // handle (not needed)
    0             // Core 0
  );

  Serial.println("🚀 System ready");
  Serial.println("   Core 0 → BLE  |  Core 1 → Render");
}

// ──────────────────────────────────────────────────────────────
//  LOOP  (runs on Core 1 — Arduino default)
// ──────────────────────────────────────────────────────────────
void loop() {
  handleButtons();
  renderMap();
  delay(16);   // ~60 fps ceiling
}
