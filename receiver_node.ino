// =============================================================================
//  receiver_node.ino   —   KARNA LoRa Receiver Node  (v1)
//  Hardware : ESP32 DevKit  +  SX1278 (433 MHz)  +  SSD1306 OLED 128×64
//
//  Roles:
//    1. Receive LoRa packets from Hub (node 0) or other receivers
//    2. Relay each unique packet exactly ONCE (mesh flood with seq dedup)
//    3. Display latest GS (gunshot) and DR (dead reckoning) data on OLED
//
//  NODE_ID: Change to 1, 2, or 3 before flashing each board.
//
//  LoRa packet format (sent by hub):
//    "KARNA,<src_id>,<seq>,<type>,<payload>"
//    type = GS  → payload = "GS,<az>,<x>,<y>,<range>,<conf>,<zcs>,<ms>"
//    type = DR  → payload = "DR,<stepCount>,<stepFlag>,<heading>,..."
//
//  Mesh dedup:
//    Each node tracks the last N seq numbers it has already relayed.
//    If a received packet's seq is in the set → drop (already relayed).
//    Otherwise → relay and add seq to set.
//
//  Wiring (same as your original receiver code):
//    SCK  → 19,  MISO → 18,  MOSI → 26,  SS → 25
//    RST  → 32,  DIO0 → 13
//    OLED SDA → 21,  SCL → 22
//
//  Libraries required:
//    - LoRa by sandeepmistry
//    - Adafruit SSD1306
//    - Adafruit GFX
// =============================================================================

#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// =============================================================================
// ── CONFIGURATION ─────────────────────────────────────────────────────────────
// =============================================================================

#define NODE_ID   1           // ← CHANGE TO 1, 2, or 3 before flashing

// LoRa pins (same as your original receiver)
#define LORA_SCK   19
#define LORA_MISO  18
#define LORA_MOSI  26
#define LORA_SS    25
#define LORA_RST   32
#define LORA_DIO0  13
#define LORA_FREQ  433E6

// OLED
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT  64
#define OLED_ADDR     0x3C

// Mesh: relay delay to avoid collisions (each node waits NODE_ID × slot ms)
#define RELAY_SLOT_MS   80    // ms per slot; node N waits N×80 ms before TX

// Dedup ring: remember last 16 sequence numbers
#define DEDUP_SIZE  16

// =============================================================================
// ── GLOBALS ───────────────────────────────────────────────────────────────────
// =============================================================================

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Dedup table
static uint32_t dedupSeqs[DEDUP_SIZE] = {0};
static int      dedupHead = 0;

// Relay queue (single pending relay; enough for our rate)
static char     relayBuf[260] = {0};
static bool     relayPending  = false;
static uint32_t relayAt       = 0;   // millis() to send relay

// OLED display state
static char  gs_az[12]    = "---";
static char  gs_xy[24]    = "x:-- y:--";
static char  gs_range[12] = "---";
static char  gs_conf[8]   = "---";
static char  dr_step[12]  = "---";
static char  dr_head[12]  = "---";
static char  dr_xy[24]    = "x:-- y:--";
static int   lastRSSI     = 0;
static uint32_t rxCount   = 0;
static uint32_t gsCount   = 0;
static uint32_t drCount   = 0;

// =============================================================================
// ── DEDUP ─────────────────────────────────────────────────────────────────────
// =============================================================================

static bool dedupSeen(uint32_t seq) {
    for (int i = 0; i < DEDUP_SIZE; i++)
        if (dedupSeqs[i] == seq && seq != 0) return true;
    return false;
}

static void dedupAdd(uint32_t seq) {
    dedupSeqs[dedupHead] = seq;
    dedupHead = (dedupHead + 1) % DEDUP_SIZE;
}

// =============================================================================
// ── PARSE PACKET ─────────────────────────────────────────────────────────────
// =============================================================================
// "KARNA,<src>,<seq>,<type>,<payload...>"
// Returns false if packet is not a valid KARNA packet.

static bool parsePacket(const char* pkt,
                         int& src, uint32_t& seq,
                         char* type, char* payload) {
    // Check prefix
    if (strncmp(pkt, "KARNA,", 6) != 0) return false;

    const char* p = pkt + 6;
    char* end;

    src = (int)strtol(p, &end, 10);
    if (*end != ',') return false; p = end + 1;

    seq = (uint32_t)strtoul(p, &end, 10);
    if (*end != ',') return false; p = end + 1;

    // type is 2 chars
    if (strlen(p) < 3) return false;
    type[0] = p[0]; type[1] = p[1]; type[2] = '\0';
    if (p[2] != ',') return false; p += 3;

    strncpy(payload, p, 250);
    payload[250] = '\0';
    return true;
}

// =============================================================================
// ── PARSE GS PAYLOAD ──────────────────────────────────────────────────────────
// "GS,<az>,<x>,<y>,<range>,<conf>,<zcs>,<ms>"

static void parseGS(const char* payload) {
    // payload starts with "GS,"
    if (strncmp(payload, "GS,", 3) != 0) return;
    const char* p = payload + 3;
    float az, x, y, range, conf, zcs;
    unsigned long ms;
    int n = sscanf(p, "%f,%f,%f,%f,%f,%f,%lu",
                   &az, &x, &y, &range, &conf, &zcs, &ms);
    if (n < 6) return;

    snprintf(gs_az,    sizeof(gs_az),    "%.1f deg", az);
    snprintf(gs_xy,    sizeof(gs_xy),    "X:%.1f Y:%.1f", x, y);
    snprintf(gs_range, sizeof(gs_range), "%.1f m", range);
    snprintf(gs_conf,  sizeof(gs_conf),  "%.0f%%", conf * 100.0f);
    gsCount++;
}

// =============================================================================
// ── PARSE DR PAYLOAD ──────────────────────────────────────────────────────────
// "DR,<stepCount>,<stepFlag>,<heading>,<ekf_x>,<ekf_y>,..."

static void parseDR(const char* payload) {
    if (strncmp(payload, "DR,", 3) != 0) return;
    const char* p = payload + 3;
    unsigned long steps;
    int sflag;
    float heading, ex, ey;
    int n = sscanf(p, "%lu,%d,%f,%f,%f", &steps, &sflag, &heading, &ex, &ey);
    if (n < 5) return;

    snprintf(dr_step, sizeof(dr_step), "%lu", steps);
    snprintf(dr_head, sizeof(dr_head), "%.1f deg", heading);
    snprintf(dr_xy,   sizeof(dr_xy),   "X:%.1f Y:%.1f", ex, ey);
    drCount++;
}

// =============================================================================
// ── OLED DISPLAY ──────────────────────────────────────────────────────────────
// Layout (128×64, font 1 = 6×8 px, font 2 = 12×16 px):
//   Row 0 (y=0)  : "Node N  RX:NNNN  RSSI:NNN"
//   Row 1 (y=10) : "GS: <az>"
//   Row 2 (y=18) : "  Pos <xy>"
//   Row 3 (y=26) : "  R:<range> C:<conf>"
//   Row 4 (y=36) : "DR: step <steps>"
//   Row 5 (y=44) : "  Hdg <heading>"
//   Row 6 (y=52) : "  <xy>"

static void updateOLED() {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);

    // Header
    display.setCursor(0, 0);
    display.print("N"); display.print(NODE_ID);
    display.print(" RX:"); display.print(rxCount);
    display.print(" RS:"); display.print(lastRSSI);

    // Gunshot
    display.setCursor(0, 10);
    display.print("GS Az:");
    display.print(gs_az);

    display.setCursor(0, 18);
    display.print(" ");
    display.print(gs_xy);

    display.setCursor(0, 26);
    display.print(" R:");
    display.print(gs_range);
    display.print(" C:");
    display.print(gs_conf);

    // Dead reckoning
    display.setCursor(0, 36);
    display.print("DR St:");
    display.print(dr_step);

    display.setCursor(0, 44);
    display.print(" Hdg:");
    display.print(dr_head);

    display.setCursor(0, 52);
    display.print(" ");
    display.print(dr_xy);

    display.display();
}

// =============================================================================
// ── SETUP ─────────────────────────────────────────────────────────────────────
// =============================================================================

void setup() {
    Serial.begin(115200);

    // OLED
    Wire.begin(21, 22);
    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        Serial.println("[OLED] Init failed");
        while (1);
    }
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 20);
    display.print("KARNA Node ");
    display.print(NODE_ID);
    display.setCursor(0, 36);
    display.print("Starting...");
    display.display();
    delay(1500);

    // LoRa — MUST match hub settings exactly
    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
    LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
    if (!LoRa.begin(LORA_FREQ)) {
        display.clearDisplay();
        display.setCursor(0, 20);
        display.println("LoRa FAILED");
        display.display();
        while (1);
    }
    LoRa.setTxPower(17);
    LoRa.setSpreadingFactor(9);       // must match hub
    LoRa.setSignalBandwidth(125E3);   // must match hub
    LoRa.setCodingRate4(5);           // must match hub

    Serial.print("[SYS] KARNA Receiver Node ");
    Serial.print(NODE_ID);
    Serial.println(" ready");

    updateOLED();
}

// =============================================================================
// ── MAIN LOOP ─────────────────────────────────────────────────────────────────
// =============================================================================

void loop() {
    // ── Relay pending packet (staggered by node ID) ───────────────────────────
    if (relayPending && millis() >= relayAt) {
        relayPending = false;
        LoRa.beginPacket();
        LoRa.print(relayBuf);
        LoRa.endPacket();
        Serial.print("[LORA] Relayed: "); Serial.println(relayBuf);
    }

    // ── Receive ───────────────────────────────────────────────────────────────
    int pktSize = LoRa.parsePacket();
    if (pktSize <= 0) return;

    char pkt[260] = {0};
    int i = 0;
    while (LoRa.available() && i < 259) pkt[i++] = (char)LoRa.read();
    pkt[i] = '\0';
    lastRSSI = LoRa.packetRssi();

    Serial.print("[LORA] RX (RSSI="); Serial.print(lastRSSI);
    Serial.print("): "); Serial.println(pkt);

    // Parse
    int src; uint32_t seq; char type[4]; char payload[256];
    if (!parsePacket(pkt, src, seq, type, payload)) {
        Serial.println("[LORA] Not a KARNA packet, ignored");
        return;
    }

    // Don't process our own relayed packets
    if (src == NODE_ID) return;

    // Dedup check
    if (dedupSeen(seq)) {
        Serial.print("[LORA] Dup seq "); Serial.println(seq);
        return;
    }
    dedupAdd(seq);
    rxCount++;

    // Handle by type
    if (strcmp(type, "GS") == 0) {
        parseGS(payload);
        Serial.print("[GS] Az="); Serial.print(gs_az);
        Serial.print(" R="); Serial.println(gs_range);
    } else if (strcmp(type, "DR") == 0) {
        parseDR(payload);
        Serial.print("[DR] Steps="); Serial.print(dr_step);
        Serial.print(" Hdg="); Serial.println(dr_head);
    } else {
        Serial.print("[LORA] Unknown type: "); Serial.println(type);
    }

    updateOLED();

    // ── Relay (re-broadcast with original src+seq, staggered) ────────────────
    // Rebuild packet as-is (preserve original src and seq for dedup on other nodes)
    snprintf(relayBuf, sizeof(relayBuf), "%s", pkt);
    relayPending = true;
    relayAt = millis() + (uint32_t)(NODE_ID) * RELAY_SLOT_MS;
}
