#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define NODE_ID 1          // 🔥 CHANGE TO 1 or 2

// ---------------- LoRa Pins ----------------
#define LORA_SCK   19
#define LORA_MISO  18
#define LORA_MOSI  26
#define LORA_SS    25
#define LORA_RST   32
#define LORA_DIO0  13

// ---------------- OLED ----------------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_ADDR 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ---------------- Counters ----------------
long packetCounter = 0;
long receivedCounter = 0;
int lastSender = 0;
int lastRSSI = 0;

unsigned long lastSendTime = 0;

void setup() {
  Serial.begin(115200);

  // ---------- OLED ----------
  Wire.begin(21, 22);

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("OLED failed");
    while (1);
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(10, 20);
  display.println("Starting Node...");
  display.display();
  delay(2000);

  // ---------- LoRa ----------
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(433E6)) {
    display.clearDisplay();
    display.setCursor(0, 20);
    display.println("LoRa Failed!");
    display.display();
    while (1);
  }

  updateDisplay();

  // Node 1 starts communication
  if (NODE_ID == 1) {
    delay(3000);
    sendPacket();
  }
}

void loop() {

  int packetSize = LoRa.parsePacket();

  if (packetSize) {

    String incoming = "";

    while (LoRa.available()) {
      incoming += (char)LoRa.read();
    }

    lastRSSI = LoRa.packetRssi();
    receivedCounter++;

    // Expected format: HELLO,X,Y
    int senderID = incoming.substring(6,7).toInt();
    lastSender = senderID;

    Serial.println("Received: " + incoming);
    Serial.print("RSSI: ");
    Serial.println(lastRSSI);

    updateDisplay();

    // Forward to other node
    int nextNode = (senderID == 1) ? 2 : 1;

    if (nextNode == NODE_ID) {
      delay(1000);
      sendPacket();
    }
  }
}

void sendPacket()
{
  packetCounter++;

  String message = "HELLO," + String(NODE_ID) + "," + String(packetCounter);

  LoRa.beginPacket();
  LoRa.print(message);
  LoRa.endPacket();

  Serial.println("Sent: " + message);

  updateDisplay();
}

void updateDisplay()
{
  display.clearDisplay();

  display.setCursor(0,0);
  display.println("LoRa 2-Node Mesh");

  display.setCursor(0,12);
  display.print("Node ID: ");
  display.println(NODE_ID);

  display.setCursor(0,24);
  display.print("Sent: ");
  display.println(packetCounter);

  display.setCursor(0,36);
  display.print("Recv: ");
  display.println(receivedCounter);

  display.setCursor(0,48);
  display.print("Last From: ");
  display.print(lastSender);

  display.setCursor(80,48);
  display.print("RSSI:");
  display.print(lastRSSI);

  display.display();
}