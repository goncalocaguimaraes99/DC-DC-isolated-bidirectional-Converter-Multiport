#include <NimBLEDevice.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>

// ── UUIDs ────────────────────────────────────────────────────────────────
#define SERVICE_UUID  "12345678-1234-1234-1234-123456789abc"
#define RX_CHAR_UUID  "12345678-1234-1234-1234-123456789abd"
#define TX_CHAR_UUID  "12345678-1234-1234-1234-123456789abe"

#define DEVICE_NAME   "ESP32-Agent"
#define LED_PIN       8

// ── UART para STM32 ──────────────────────────────────────────────────────
#define STM_BAUD    38400
#define STM_RX_PIN  4
#define STM_TX_PIN  5

// ── ST7735 SPI ───────────────────────────────────────────────────────────
// Ajusta os pinos ao teu wiring no ESP32-C3 Mini
#define TFT_CS    7
#define TFT_DC    6
#define TFT_RST   10

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

// ── Dados dos sensores (actualizados quando chega nova linha da STM32) ───
float dispV1 = 0, dispV2 = 0, dispV3 = 0;
float dispI2 = 0, dispI3 = 0;
int   dispMode = 0;
bool  lcdReady = false;
bool  lcdNeedsUpdate = false;

// ── BLE ──────────────────────────────────────────────────────────────────
NimBLECharacteristic* pRxChar = nullptr;
bool deviceConnected = false;

// ── UART buffer ──────────────────────────────────────────────────────────
String uartLine = "";
int    lastMode = 1;

// ── Cores (compatíveis com Adafruit_ST7735) ──────────────────────────────
#define COL_BG      ST77XX_BLACK
#define COL_LABEL   0x07FF   // cyan
#define COL_VALUE   ST77XX_WHITE
#define COL_DIM     0x8410   // cinzento

// ── LCD helpers ──────────────────────────────────────────────────────────

void lcdDrawLayout() {
  tft.fillScreen(COL_BG);

  // Título
  tft.setTextColor(COL_LABEL);
  tft.setTextSize(1);
  tft.setCursor(2, 2);
  tft.print("AI MONITOR");

  // Linha separadora
  tft.drawFastHLine(0, 13, tft.width(), COL_DIM);

  // Labels fixos (nunca se apagam, igual à STM32)
  tft.setTextColor(COL_LABEL);

  tft.setCursor(2, 20);  tft.print("V1:");
  tft.setCursor(2, 35);  tft.print("V2:");
  tft.setCursor(2, 50);  tft.print("V3:");

  tft.drawFastHLine(0, 65, tft.width(), COL_DIM);

  tft.setCursor(2, 72);  tft.print("I2:");
  tft.setCursor(2, 87);  tft.print("I3:");

  // Mode label
  tft.drawFastHLine(0, 102, tft.width(), COL_DIM);
  tft.setCursor(2, 108); tft.print("Mode:");

  lcdReady = true;
}

// Escreve um valor float numa posição, apagando o anterior
void lcdWriteFloat(int16_t x, int16_t y, float val, const char* unit = "V") {
  // Apaga área do valor anterior
  tft.fillRect(x, y, 90, 10, COL_BG);
  // Escreve novo valor
  tft.setTextColor(COL_VALUE);
  tft.setTextSize(1);
  tft.setCursor(x, y);
  tft.print(val, 3);
  tft.print(unit);
}

void lcdWriteMode(int mode) {
  tft.fillRect(40, 108, 60, 10, COL_BG);
  tft.setTextColor(COL_VALUE);
  tft.setTextSize(1);
  tft.setCursor(40, 108);
  tft.print(mode);
}

void lcdUpdate() {
  if (!lcdReady) return;
  lcdWriteFloat(25, 20,  dispV1, "V");
  lcdWriteFloat(25, 35,  dispV2, "V");
  lcdWriteFloat(25, 50,  dispV3, "V");
  lcdWriteFloat(25, 72,  dispI2, "A");
  lcdWriteFloat(25, 87,  dispI3, "A");
  lcdWriteMode(dispMode);
}

// ── UART parser ──────────────────────────────────────────────────────────

float extractFloat(const String& line, const char* label) {
  int pos = line.indexOf(label);
  if (pos < 0) return 0.0f;
  pos += strlen(label);
  while (pos < (int)line.length() && line[pos] == ' ') pos++;
  return line.substring(pos).toFloat();
}

void parseAndSend(String line) {
  line.trim();
  if (line.length() == 0) return;

  Serial.print("[STM] ");
  Serial.println(line);

  if (line.startsWith("Mode:")) {
    lastMode = line.substring(line.indexOf(':') + 1).toInt();
    return;
  }

  if (line.indexOf("V1:") >= 0) {
    float v1 = extractFloat(line, "V1:");
    float v2 = extractFloat(line, "V2:");
    float v3 = extractFloat(line, "V3:");
    float i2 = extractFloat(line, "I2:");
    float i3 = extractFloat(line, "I3:");

    // Actualiza variáveis do display
    dispV1 = v1; dispV2 = v2; dispV3 = v3;
    dispI2 = i2; dispI3 = i3; dispMode = lastMode;
    lcdNeedsUpdate = true;

    // Envia via BLE
    if (deviceConnected && pRxChar) {
      char payload[128];
      snprintf(payload, sizeof(payload),
        "{\"mode\":%d,\"v1\":%.3f,\"v2\":%.3f,\"v3\":%.3f,\"i2\":%.3f,\"i3\":%.3f}",
        lastMode, v1, v2, v3, i2, i3);
      pRxChar->setValue(payload);
      pRxChar->notify();
      Serial.print("[BLE TX] ");
      Serial.println(payload);
    }
    return;
  }

  if (line.startsWith("STOP") || line.startsWith("RUN")) {
    if (deviceConnected && pRxChar) {
      char payload[64];
      snprintf(payload, sizeof(payload), "{\"status\":\"%s\"}", line.c_str());
      pRxChar->setValue(payload);
      pRxChar->notify();
    }
  }
}

// ── BLE Callbacks ────────────────────────────────────────────────────────

class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override {
    deviceConnected = true;
    Serial.println("[BLE] Pi connected!");
    digitalWrite(LED_PIN, HIGH);
    NimBLEDevice::stopAdvertising();
  }
  void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
    deviceConnected = false;
    Serial.print("[BLE] Disconnected, reason=");
    Serial.println(reason);
    digitalWrite(LED_PIN, LOW);
    NimBLEDevice::startAdvertising();
    Serial.println("[BLE] Advertising restarted.");
  }
};

class CommandCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* pChar, NimBLEConnInfo& connInfo) override {
    String cmd = pChar->getValue().c_str();
    cmd.trim();
    Serial.print("[BLE RX] '");
    Serial.print(cmd);
    Serial.println("'");

    if      (cmd == "start") { Serial1.write('r'); Serial.println("[STM TX] 'r'"); }
    else if (cmd == "stop")  { Serial1.write('s'); Serial.println("[STM TX] 's'"); }
    else if (cmd == "1")     { Serial1.write('1'); Serial.println("[STM TX] '1'"); }
    else if (cmd == "2")     { Serial1.write('2'); Serial.println("[STM TX] '2'"); }
    else if (cmd == "3")     { Serial1.write('3'); Serial.println("[STM TX] '3'"); }
    else if (cmd == "led_on")  { digitalWrite(LED_PIN, HIGH); }
    else if (cmd == "led_off") { digitalWrite(LED_PIN, LOW);  }
    else {
      Serial.print("[BLE RX] Unknown: '");
      Serial.print(cmd);
      Serial.println("'");
    }
  }
};

// ── Setup ────────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  Serial1.begin(STM_BAUD, SERIAL_8N1, STM_RX_PIN, STM_TX_PIN);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // LCD init
  SPI.begin(2, -1, 3, 7);  // SCK, MISO, MOSI, SS
pinMode(TFT_RST, OUTPUT);
digitalWrite(TFT_RST, LOW);
delay(100);
digitalWrite(TFT_RST, HIGH);
delay(100);
tft.initR(INITR_BLACKTAB);
  //tft.initR(INITR_BLACKTAB);   // ou INITR_GREENTAB / INITR_REDTAB conforme o teu módulo
  tft.setRotation(0);          // 0=portrait, 1=landscape — ajusta ao teu mounting
  lcdDrawLayout();
// Quick display test
/*
  while (true) {
    tft.fillScreen(ST77XX_BLACK);
    delay(500);
    tft.fillScreen(ST77XX_RED);
    delay(500);
    tft.setCursor(10, 10);
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(2);
    tft.print("HELLO");
    delay(1000);
  }*/
  // BLE init
  NimBLEDevice::init(DEVICE_NAME);
  NimBLEServer* pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());
  pServer->advertiseOnDisconnect(false);

  NimBLEService* pService = pServer->createService(SERVICE_UUID);

  pRxChar = pService->createCharacteristic(
    RX_CHAR_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
  );

  NimBLECharacteristic* pTxChar = pService->createCharacteristic(
    TX_CHAR_UUID,
    NIMBLE_PROPERTY::WRITE
  );
  pTxChar->setCallbacks(new CommandCallbacks());

  pService->start();

  NimBLEAdvertising* pAdv = NimBLEDevice::getAdvertising();
  pAdv->setName(DEVICE_NAME);
  pAdv->addServiceUUID(SERVICE_UUID);
  pAdv->setMinInterval(160);
  pAdv->setMaxInterval(320);
  pAdv->start(0);

  Serial.println("=== ESP32 Agent ready ===");
}

// ── Loop ─────────────────────────────────────────────────────────────────

uint32_t lastAdvCheck = 0;

void loop() {
  // Lê UART da STM32
  while (Serial1.available()) {
    char c = (char)Serial1.read();
    if (c == '\n') {
      parseAndSend(uartLine);
      uartLine = "";
    } else if (c != '\r') {
      uartLine += c;
      if (uartLine.length() > 256) uartLine = "";
    }
  }

  // Actualiza LCD se houver novos dados
  if (lcdNeedsUpdate) {
    lcdNeedsUpdate = false;
    lcdUpdate();
  }

  // Watchdog BLE advertising
  if (!deviceConnected && millis() - lastAdvCheck > 5000) {
    lastAdvCheck = millis();
    if (!NimBLEDevice::getAdvertising()->isAdvertising()) {
      Serial.println("[BLE] Restarting advertising...");
      NimBLEDevice::startAdvertising();
    }
  }
}

// ── LIGAÇÕES ST7735 ───────────────────────────────────────────────────────
//
//  ST7735      ESP32-C3 Mini
//  VCC    ──►  3.3V
//  GND    ──►  GND
//  SCL    ──►  GPIO2  (SPI CLK)
//  SDA    ──►  GPIO3  (SPI MOSI)
//  RES    ──►  GPIO10 (TFT_RST)
//  DC     ──►  GPIO6  (TFT_DC)
//  CS     ──►  GPIO7  (TFT_CS)
//  BL     ──►  3.3V   (backlight sempre ligado)
//
// Instala no Arduino IDE (Library Manager):
//   "Adafruit ST7735 and ST7789 Library"
//   "Adafruit GFX Library"
//
// Se o ecrã aparecer com cores invertidas, muda initR para INITR_GREENTAB.
// Se o layout aparecer rotacionado, ajusta setRotation(0..3).
