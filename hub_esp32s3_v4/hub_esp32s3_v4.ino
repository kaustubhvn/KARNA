// =============================================================================
//  hub_esp32s3.ino   —   KARNA Hub Node  v4
//  Hardware : ESP32-S3 SuperMini
//
//  Roles:
//    1. UART RX  ← Teensy 4.1 (GS CSV lines at 115200 baud, GPIO 44)
//                  Format: "GS,<mic>,<angle>,<direction>\n"
//    2. BLE Central ← DR/PDR Unit  → reads EKF lat/lon/heading
//                     Device name : "GPS_TRACKER"          ← matches doc 1
//                     Service UUID: 4fafc201-...           ← matches doc 1
//                     Char UUID   : beb5483e-...           ← matches doc 1
//                     Payload     : "rawLat,rawLon,heading_deg"
//    3. BLE Peripheral A → Display unit  (same GPS_TRACKER service/char UUIDs)
//                          Payload: "fusedLat,fusedLon,gsAngle"
//    4. BLE Peripheral B → Python laptop (KARNA_HUB service)
//                          GS char: "GS,<mic>,<angle>,<direction>"
//                          DR char: "DR,<full DR string>"
//    5. LoRa TX → 3× receiver nodes
//                 "KARNA,0,<seq>,GS,<mic>,<angle>,<direction>"
//                 "KARNA,0,<seq>,DR,<DR string>"
//
//  Libraries:  (same stack as PDR unit — doc 1)
//    - BLEDevice / BLEServer / BLEClient / BLEUtils / BLE2902
//    - LoRa by sandeepmistry
//
//  Wiring:
//    Teensy TX  → ESP32-S3 GPIO 44  (shared GND)
//    SX1278  SCK  → GPIO 11,  MISO → GPIO 10
//            MOSI → GPIO  9,  NSS  → GPIO  8
//            RST  → GPIO  7,  DIO0 → GPIO  6
//
//  Key change vs v3:
//    • ArduinoBLE replaced with BLEDevice/BLEServer/BLEClient
//      (identical stack to PDR unit, so UUIDs and pairing work out of the box)
//    • BLEDevice stack supports concurrent Central + Peripheral on ESP32,
//      so the PHASE_PERIPHERAL / PHASE_CENTRAL alternation is REMOVED.
//      Hub stays advertised to display + Python at all times while scanning
//      for the DR unit in the background via a periodic client poll.
//    • UUID / device-name mismatch fixed:
//        v3 scanned for "PDR_ESP32" with placeholder UUIDs → never connected
//        v4 scans for "GPS_TRACKER" with the exact UUIDs from doc 1
// =============================================================================

#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <HardwareSerial.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <BLEClient.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

// =============================================================================
// ── PIN CONFIG ────────────────────────────────────────────────────────────────
// =============================================================================

#define TEENSY_RX_PIN   44
#define TEENSY_BAUD     115200

#define LORA_SCK    11
#define LORA_MISO   10
#define LORA_MOSI    9
#define LORA_SS      8
#define LORA_RST     7
#define LORA_DIO0    6
#define LORA_FREQ   433E6
#define LORA_TX_PWR  17

// =============================================================================
// ── BLE UUIDs ─────────────────────────────────────────────────────────────────
// =============================================================================

// ── DR/PDR unit (doc 1) — hub connects to this as BLE Central ────────────────
// PDR unit advertises as "GPS_TRACKER" with these exact UUIDs.
// Hub scans for this name and subscribes to notifications.
#define DR_DEVICE_NAME   "GPS_TRACKER"
#define DR_SERVICE_UUID  "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define DR_CHAR_UUID     "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// ── Display unit — hub advertises this so display auto-connects ───────────────
// Display scans for "GPS_TRACKER" / same UUIDs → hub looks like the DR unit.
// Payload: "fusedLat,fusedLon,gsAngle"
#define DISPLAY_DEVICE_NAME  "GPS_TRACKER"
#define DISPLAY_SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define DISPLAY_CHAR_UUID    "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// ── Python laptop — hub advertises a second service ───────────────────────────
#define HUB_SERVICE_UUID "AAAA0001-0000-1000-8000-00805F9B34FB"
#define HUB_GS_CHAR_UUID "AAAA0002-0000-1000-8000-00805F9B34FB"
#define HUB_DR_CHAR_UUID "AAAA0003-0000-1000-8000-00805F9B34FB"

// =============================================================================
// ── TIMING ────────────────────────────────────────────────────────────────────
// =============================================================================

#define CENTRAL_POLL_MS   4000   // how often to attempt DR unit read
#define CENTRAL_TIMEOUT   3000   // scan + connect timeout
#define CENTRAL_READ_MS   2000   // how long to wait for a notify after connecting
#define DR_LORA_MIN_MS     500   // minimum gap between consecutive DR LoRa TX

// =============================================================================
// ── BLE PERIPHERAL OBJECTS ───────────────────────────────────────────────────
// =============================================================================

static BLEServer*         bleServer   = nullptr;

// Service 1: GPS_TRACKER → display unit
static BLEService*        displaySvc  = nullptr;
static BLECharacteristic* displayChar = nullptr;

// Service 2: KARNA_HUB → Python laptop
static BLEService*        hubSvc      = nullptr;
static BLECharacteristic* hubGsChar   = nullptr;
static BLECharacteristic* hubDrChar   = nullptr;

static bool displayConnected = false;
static bool hubConnected     = false;

// Peripheral server callbacks
class HubServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer*) override {
        Serial.println("[BLE-P] Client connected");
        // Restart advertising so more clients can connect simultaneously
        BLEDevice::startAdvertising();
    }
    void onDisconnect(BLEServer*) override {
        Serial.println("[BLE-P] Client disconnected — restarting advertising");
        BLEDevice::startAdvertising();
    }
};

// =============================================================================
// ── BLE CENTRAL OBJECTS ───────────────────────────────────────────────────────
// =============================================================================

static BLEClient*  drClient    = nullptr;
static bool        drConnected = false;
static bool        drFoundFlag = false;

static BLEAddress  drAddress("00:00:00:00:00:00");

// Scan callback — fires when the scan finds an advertised device
class DrScanCallback : public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice dev) override {
        if (dev.getName() == DR_DEVICE_NAME &&
            dev.haveServiceUUID() &&
            dev.isAdvertisingService(BLEUUID(DR_SERVICE_UUID))) {
            drAddress  = dev.getAddress();
            drFoundFlag = true;
            dev.getScan()->stop();
            Serial.printf("[BLE-C] DR unit found: %s\n",
                          drAddress.toString().c_str());
        }
    }
};
static DrScanCallback drScanCB;

// Client callbacks — detect disconnection from DR unit
class DrClientCallbacks : public BLEClientCallbacks {
    void onConnect(BLEClient*)    override { drConnected = true;  }
    void onDisconnect(BLEClient*) override {
        drConnected = false;
        Serial.println("[BLE-C] DR unit disconnected");
    }
};

// =============================================================================
// ── STATE ─────────────────────────────────────────────────────────────────────
// =============================================================================

#define LINE_BUF 128

// UART line buffer (Teensy input)
static char uart_buf[LINE_BUF];
static int  uart_pos = 0;

// GS event queue — same as v3
#define GS_QUEUE_SIZE 4
static char gs_queue[GS_QUEUE_SIZE][LINE_BUF];
static int  gs_queue_head = 0;
static int  gs_queue_tail = 0;

// Latest DR data — parsed from PDR unit payload "rawLat,rawLon,heading_deg"
static char   latestDrStr[256]  = {0};   // raw string as received
static double latestFusedLat    = 0.0;
static double latestFusedLon    = 0.0;
static float  latestDrHeading   = 0.0f;
static bool   newDrAvailable    = false;
static bool   drHasGPS          = false;

// Latest GS angle for forwarding to display as heading
static float  latestGsAngle   = 0.0f;
static bool   newGsForDisplay = false;

// LoRa sequence counter and DR rate-limiting
static uint32_t loraSeq      = 0;
static uint32_t lastDrLoraTx = 0;

// Central poll timing
static uint32_t lastCentralSwitch = 0;

// =============================================================================
// ── GS QUEUE — identical to v3 ───────────────────────────────────────────────
// =============================================================================

static void gsEnqueue(const char* s) {
    int next = (gs_queue_tail + 1) % GS_QUEUE_SIZE;
    if (next == gs_queue_head)
        gs_queue_head = (gs_queue_head + 1) % GS_QUEUE_SIZE; // drop oldest
    strncpy(gs_queue[gs_queue_tail], s, LINE_BUF - 1);
    gs_queue[gs_queue_tail][LINE_BUF - 1] = '\0';
    gs_queue_tail = next;
}

static bool gsDequeue(char* out) {
    if (gs_queue_head == gs_queue_tail) return false;
    strncpy(out, gs_queue[gs_queue_head], LINE_BUF - 1);
    out[LINE_BUF - 1] = '\0';
    gs_queue_head = (gs_queue_head + 1) % GS_QUEUE_SIZE;
    return true;
}

// =============================================================================
// ── LoRa — identical to v3 ────────────────────────────────────────────────────
// =============================================================================

static void loraSendGS(const char* gsLine) {
    char msg[200];
    snprintf(msg, sizeof(msg), "KARNA,0,%lu,GS,%s", ++loraSeq, gsLine);
    msg[199] = '\0';
    LoRa.beginPacket();
    LoRa.print(msg);
    LoRa.endPacket();
    Serial.print("[LORA] GS TX: "); Serial.println(msg);
}

static void loraSendDR(const char* drStr) {
    uint32_t now = millis();
    if (now - lastDrLoraTx < (uint32_t)DR_LORA_MIN_MS) return;
    lastDrLoraTx = now;
    char msg[280];
    snprintf(msg, sizeof(msg), "KARNA,0,%lu,DR,%s", ++loraSeq, drStr);
    msg[249] = '\0';
    LoRa.beginPacket();
    LoRa.print(msg);
    LoRa.endPacket();
    Serial.print("[LORA] DR TX: "); Serial.println(msg);
}

// =============================================================================
// ── PARSE DR PAYLOAD from PDR unit ───────────────────────────────────────────
// PDR unit (doc 1) transmitBLE() sends: "%.6f,%.6f,%.1f"
//   → rawLat, rawLon, heading_deg
// Store into latestDrStr, latestFusedLat/Lon, latestDrHeading, drHasGPS.
// Also forward raw string to LoRa so receiver node parseDR() can use it.
// =============================================================================

static void parseDrPayload(const std::string& val) {
    if (val.empty() || val.size() >= sizeof(latestDrStr)) return;

    // Copy raw string for LoRa forwarding
    strncpy(latestDrStr, val.c_str(), sizeof(latestDrStr) - 1);
    latestDrStr[sizeof(latestDrStr) - 1] = '\0';

    // Parse "lat,lon,hdg"
    float lat = 0.0f, lon = 0.0f, hdg = 0.0f;
    int n = sscanf(latestDrStr, "%f,%f,%f", &lat, &lon, &hdg);
    if (n == 3) {
        latestFusedLat  = (double)lat;
        latestFusedLon  = (double)lon;
        latestDrHeading = hdg;
        drHasGPS        = true;
        newDrAvailable  = true;
        Serial.printf("[DR] lat=%.6f lon=%.6f hdg=%.1f\n",
                      latestFusedLat, latestFusedLon, latestDrHeading);
    }
}

// =============================================================================
// ── TEENSY UART — identical logic to v3 ──────────────────────────────────────
// =============================================================================

static void processTeensyLine(const char* line) {
    if (strncmp(line, "GS,", 3) != 0) return;

    Serial.print("[UART] "); Serial.println(line);

    // Extract angle (field index 2)
    char tmp[LINE_BUF];
    strncpy(tmp, line, sizeof(tmp) - 1);
    tmp[sizeof(tmp) - 1] = '\0';
    char* tok = strtok(tmp, ",");   // "GS"
    tok = strtok(nullptr, ",");     // mic
    tok = strtok(nullptr, ",");     // angle
    if (tok) {
        latestGsAngle  = atof(tok);
        newGsForDisplay = true;
    }

    loraSendGS(line);
    gsEnqueue(line);
}

static void drainTeensyUart() {
    while (Serial1.available()) {
        char c = (char)Serial1.read();
        if (c == '\n' || c == '\r') {
            if (uart_pos > 0) {
                uart_buf[uart_pos] = '\0';
                processTeensyLine(uart_buf);
                uart_pos = 0;
            }
        } else if (uart_pos < LINE_BUF - 1) {
            uart_buf[uart_pos++] = c;
        }
    }
}

// =============================================================================
// ── BUILD DISPLAY PAYLOAD — identical to v3 ──────────────────────────────────
// Display unit parses: sscanf(buf, "%lf,%lf,%f", &lat, &lon, &hdg)
// We send GS azimuth as heading so arrow shows gunshot direction.
// =============================================================================

static void buildDisplayPayload(char* out, int maxlen) {
    snprintf(out, maxlen, "%.7f,%.7f,%.1f",
             latestFusedLat, latestFusedLon, latestGsAngle);
}

// =============================================================================
// ── PERIPHERAL NOTIFY — replaces runPeripheral() from v3 ─────────────────────
// Called every loop iteration. No phase gating needed — BLEDevice stack
// handles peripheral + central concurrently.
// =============================================================================

static void runPeripheral() {
    // ── Display unit ──────────────────────────────────────────────────────────
    if (newGsForDisplay) {
        newGsForDisplay = false;
        char payload[64];
        buildDisplayPayload(payload, sizeof(payload));
        displayChar->setValue((uint8_t*)payload, strlen(payload));
        displayChar->notify();
        Serial.print("[BLE-P] Display notify: "); Serial.println(payload);
    }

    // ── Python laptop: flush queued GS events ────────────────────────────────
    char gsStr[LINE_BUF];
    while (gsDequeue(gsStr)) {
        hubGsChar->setValue((uint8_t*)gsStr, strlen(gsStr));
        hubGsChar->notify();
        Serial.print("[BLE-P] GS→Python: "); Serial.println(gsStr);
    }

    // ── Python laptop + display: push latest DR ───────────────────────────────
    if (newDrAvailable && latestDrStr[0] != '\0') {
        newDrAvailable = false;

        // Forward to Python with "DR," prefix (same as v3)
        char drFwd[260];
        snprintf(drFwd, sizeof(drFwd), "DR,%s", latestDrStr);
        hubDrChar->setValue((uint8_t*)drFwd, strlen(drFwd));
        hubDrChar->notify();
        Serial.print("[BLE-P] DR→Python: "); Serial.println(drFwd);

        // Also refresh display with updated GPS position (same as v3)
        char payload[64];
        buildDisplayPayload(payload, sizeof(payload));
        displayChar->setValue((uint8_t*)payload, strlen(payload));
        displayChar->notify();
    }
}

// =============================================================================
// ── CENTRAL POLL — replaces runCentral() / phase-switch from v3 ──────────────
// Called every CENTRAL_POLL_MS. Scans → connects → waits for notify →
// disconnects. UART is drained throughout so no GS events are lost.
// =============================================================================

static void pollDrUnit() {
    Serial.println("[BLE-C] Scanning for DR unit...");

    drFoundFlag = false;
    BLEScan* scan = BLEDevice::getScan();
    scan->setAdvertisedDeviceCallbacks(&drScanCB, false);
    scan->setActiveScan(true);
    scan->start(CENTRAL_TIMEOUT / 1000, false);   // blocking scan (seconds)

    drainTeensyUart();  // drain UART during scan

    if (!drFoundFlag) {
        Serial.println("[BLE-C] DR unit not found");
        return;
    }

    // ── Connect ───────────────────────────────────────────────────────────────
    if (!drClient) {
        drClient = BLEDevice::createClient();
        drClient->setClientCallbacks(new DrClientCallbacks());
    }

    Serial.printf("[BLE-C] Connecting to %s\n",
                  drAddress.toString().c_str());

    if (!drClient->connect(drAddress)) {
        Serial.println("[BLE-C] Connect failed");
        return;
    }

    // ── Discover service + characteristic ─────────────────────────────────────
    BLERemoteService* remoteSvc =
        drClient->getService(BLEUUID(DR_SERVICE_UUID));
    if (!remoteSvc) {
        Serial.println("[BLE-C] Service not found");
        drClient->disconnect();
        return;
    }

    BLERemoteCharacteristic* remoteChr =
        remoteSvc->getCharacteristic(BLEUUID(DR_CHAR_UUID));
    if (!remoteChr) {
        Serial.println("[BLE-C] Char not found");
        drClient->disconnect();
        return;
    }

    // ── Subscribe to notifications ────────────────────────────────────────────
    if (remoteChr->canNotify()) {
        remoteChr->registerForNotify(
            [](BLERemoteCharacteristic* pChr, uint8_t* data,
               size_t len, bool isNotify) {
                std::string val((char*)data, len);
                parseDrPayload(val);
                loraSendDR(latestDrStr);
            }
        );
        Serial.println("[BLE-C] Subscribed to DR notifications");
    }

    // ── Wait for notify (mirroring v3 CENTRAL_READ_MS window) ─────────────────
    uint32_t readStart = millis();
    while (drClient->isConnected() &&
           millis() - readStart < (uint32_t)CENTRAL_READ_MS) {
        drainTeensyUart();   // keep draining UART — same as v3
        delay(10);
    }

    drClient->disconnect();
    Serial.println("[BLE-C] DR poll done");
}

// =============================================================================
// ── SETUP ─────────────────────────────────────────────────────────────────────
// =============================================================================

void setup() {
    Serial.begin(115200);
    delay(800);
    Serial.println("\n[SYS] KARNA Hub v4 booting...");

    // ── UART from Teensy ───────────────────────────────────────────────────────
    Serial1.begin(TEENSY_BAUD, SERIAL_8N1, TEENSY_RX_PIN, -1);
    Serial.println("[SYS] UART1 ready (Teensy TX → GPIO 44)");

    // ── LoRa ──────────────────────────────────────────────────────────────────
    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
    LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
    if (!LoRa.begin(LORA_FREQ)) {
        Serial.println("[LORA] INIT FAILED");
    } else {
        LoRa.setTxPower(LORA_TX_PWR);
        LoRa.setSpreadingFactor(9);
        LoRa.setSignalBandwidth(125E3);
        LoRa.setCodingRate4(5);
        Serial.println("[LORA] SX1278 ready @ 433 MHz");
    }

    // ── BLEDevice init (same call as PDR unit doc 1) ──────────────────────────
    BLEDevice::init(DISPLAY_DEVICE_NAME);  // advertise as "GPS_TRACKER"
    BLEDevice::setMTU(64);

    // ── Create server ─────────────────────────────────────────────────────────
    bleServer = BLEDevice::createServer();
    bleServer->setCallbacks(new HubServerCallbacks());

    // ── Service 1: GPS_TRACKER → display unit ────────────────────────────────
    displaySvc  = bleServer->createService(DISPLAY_SERVICE_UUID);
    displayChar = displaySvc->createCharacteristic(
        DISPLAY_CHAR_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    displayChar->addDescriptor(new BLE2902());
    displaySvc->start();

    // ── Service 2: KARNA_HUB → Python laptop ─────────────────────────────────
    hubSvc    = bleServer->createService(HUB_SERVICE_UUID);
    hubGsChar = hubSvc->createCharacteristic(
        HUB_GS_CHAR_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    hubGsChar->addDescriptor(new BLE2902());

    hubDrChar = hubSvc->createCharacteristic(
        HUB_DR_CHAR_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    hubDrChar->addDescriptor(new BLE2902());
    hubSvc->start();

    // ── Advertising — identical pattern to PDR unit (doc 1) ──────────────────
    BLEAdvertising* adv = BLEDevice::getAdvertising();
    adv->addServiceUUID(DISPLAY_SERVICE_UUID);
    adv->addServiceUUID(HUB_SERVICE_UUID);
    adv->setScanResponse(true);
    adv->setMinPreferred(0x06);
    BLEDevice::startAdvertising();
    Serial.println("[BLE-P] Advertising as GPS_TRACKER + KARNA_HUB");

    lastCentralSwitch = millis();
    Serial.println("[SYS] Hub v4 ready\n");
}

// =============================================================================
// ── MAIN LOOP ─────────────────────────────────────────────────────────────────
// =============================================================================

void loop() {
    // 1. Always drain Teensy UART first (same priority as v3)
    drainTeensyUart();

    // 2. LoRa RX  (future mesh / ack — same as v3)
    int pktSize = LoRa.parsePacket();
    if (pktSize > 0) {
        char pkt[260] = {0}; int i = 0;
        while (LoRa.available() && i < 259) pkt[i++] = (char)LoRa.read();
        Serial.print("[LORA] RX: "); Serial.println(pkt);
    }

    // 3. Peripheral duties — always active, no phase gating
    runPeripheral();

    // 4. Periodic DR unit poll (replaces phase switch — same CENTRAL_POLL_MS)
    if (millis() - lastCentralSwitch >= (uint32_t)CENTRAL_POLL_MS) {
        lastCentralSwitch = millis();
        pollDrUnit();   // blocking up to CENTRAL_TIMEOUT + CENTRAL_READ_MS
    }
}
