#include <Wire.h>
#include "MAX30100_PulseOximeter.h"
#include <MPU9250.h>
#include <NimBLEDevice.h>
#include <math.h>

// ==========================================
// 0) BLE UUID'LER
// ==========================================
static const char* BLE_SERVICE_UUID        = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
static const char* BLE_CHAR_VITALS_UUID    = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"; // Notify (BINARY VITALS)
static const char* BLE_CHAR_ALERTS_UUID    = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"; // Notify (STRING ALERTS)
static const char* BLE_CHAR_COMMAND_UUID   = "6E400004-B5A3-F393-E0A9-E50E24DCCA9E"; // Write

static NimBLECharacteristic* gVitalsChar  = nullptr;
static NimBLECharacteristic* gAlertsChar  = nullptr;
static NimBLECharacteristic* gCommandChar = nullptr;

// ==========================================
// 1) AYARLAR / PINLER
// ==========================================
#define SDA_PIN 8
#define SCL_PIN 9
#define ALARM_PIN 10
#define BUTTON_PIN 0

// Daha hızlı tetik istersen 300 yapabilirsin
#define REPORTING_PERIOD_MS 1000

PulseOximeter pox;
MPU9250 mpu;

// --- ZAMANLAYICILAR ---
uint32_t tsLastReport = 0;
uint32_t lastBeatTime = 0;
unsigned long lastToneTime = 0;
unsigned long sonHareketZamani = 0;
unsigned long sonYazdirmaZamani = 0;

// Panic buton için
static unsigned long gLastPanicPress = 0;
static volatile bool gPanicEventPending = false;

// --- DEĞİŞKENLER ---
int beatCount = 0;
float bpmHistory[5] = {0};
int historyIndex = 0;
int stableReadings = 0;
bool toneState = false;
bool isEmergency = false;
bool alarmCaldimi = false;

// --- EŞİKLER ---
const int BPM_LOW_LIMIT = 40;
const int BPM_HIGH_LIMIT = 120;
const float HAREKET_ESIGI = 0.15;
const float DUSME_ESIGI = 3.00;
const unsigned long HAREKETSIZLIK_SINIRI = 20000; // 20 saniye

// --- BLE SEQ ---
static uint32_t gSeqVitals = 0;
static uint32_t gSeqAlert  = 0;

// Debug
static const bool DEBUG_BLE_TX = true;

// ==========================================
// 1.5) MOTION STATE
// ==========================================
static float gLastTotalG = 1.0f;
static unsigned long gLastMotionMillis = 0;
static unsigned long gInactivitySeconds = 0;
static bool gIsActive = true;

// ==========================================
// 2) BLE - ALERT STRING (FRAME + LENGTH)
// ==========================================
static inline String FrameMsg(const String& payload) {
  return "<" + payload + ">";
}

static void BLE_NotifyAlertPayload(const String& payloadNoFrame) {
  if (!gAlertsChar) return;

  String framed = FrameMsg(payloadNoFrame);
  gAlertsChar->setValue((uint8_t*)framed.c_str(), framed.length());
  gAlertsChar->notify();
  delay(5);

  if (DEBUG_BLE_TX) {
    Serial.print("[BLE_TX][ALERTS] ");
    Serial.println(framed);
  }
}

// ==========================================
// 2.1) BLE - VITALS BINARY (12 BYTE TEK PAKET + FLAGS)
// ==========================================
// Paket (12 byte):
// [0]=0xA1 header
// [1..2]=seq (uint16 LE)
// [3]=bpm (uint8)
// [4]=spo2 (uint8)
// [5..6]=g*100 (int16 LE)
// [7]=active (1/0)
// [8..9]=inactSec (uint16 LE)
// [10]=flags (uint8)  bit0 = PANIC_BUTTON
// [11]=checksum XOR (0..10)
static void BLE_NotifyVitalsBinary(uint16_t seq,
                                  uint8_t bpm,
                                  uint8_t spo2,
                                  float g,
                                  bool active,
                                  uint16_t inactSec,
                                  uint8_t flags) {
  if (!gVitalsChar) return;

  uint8_t p[12];
  p[0] = 0xA1;

  p[1] = (uint8_t)(seq & 0xFF);
  p[2] = (uint8_t)((seq >> 8) & 0xFF);

  p[3] = bpm;
  p[4] = spo2;

  int16_t g100 = (int16_t)lroundf(g * 100.0f);
  p[5] = (uint8_t)(g100 & 0xFF);
  p[6] = (uint8_t)((g100 >> 8) & 0xFF);

  p[7] = active ? 1 : 0;

  p[8] = (uint8_t)(inactSec & 0xFF);
  p[9] = (uint8_t)((inactSec >> 8) & 0xFF);

  p[10] = flags;

  uint8_t cs = 0;
  for (int i = 0; i < 11; i++) cs ^= p[i];
  p[11] = cs;

  gVitalsChar->setValue(p, sizeof(p));
  gVitalsChar->notify();
  delay(5);

  if (DEBUG_BLE_TX) {
    Serial.print("[BLE_TX][VITALS_BIN] seq=");
    Serial.print(seq);
    Serial.print(" bpm=");
    Serial.print(bpm);
    Serial.print(" spo2=");
    Serial.print(spo2);
    Serial.print(" g=");
    Serial.print(g, 2);
    Serial.print(" act=");
    Serial.print(active ? "1" : "0");
    Serial.print(" inact=");
    Serial.print(inactSec);
    Serial.print(" flags=");
    Serial.println(flags, HEX);
  }
}

// ==========================================
// 2.2) BLE CALLBACKS
// ==========================================
class MyServerCallbacks : public NimBLEServerCallbacks {
public:
  void onConnect(NimBLEServer* pServer) {
    (void)pServer;
    Serial.println("[BLE] CONNECTED");
    BLE_NotifyAlertPayload("SEQ:" + String(gSeqAlert++) + ",STATUS:CONNECTED");
  }

  void onDisconnect(NimBLEServer* pServer) {
    (void)pServer;
    Serial.println("[BLE] DISCONNECTED -> advertising restart");
    NimBLEDevice::startAdvertising();
  }
};

class CommandCallbacks : public NimBLECharacteristicCallbacks {
public:
  void onWrite(NimBLECharacteristic* pCharacteristic) {
    std::string v = pCharacteristic->getValue();
    if (v.empty()) return;

    char cmd = v[0];
    Serial.print("[BLE] CMD received: "); Serial.println(cmd);

    // İstersen mobil buradan da panic tetikleyebilsin:
    // 'A' -> panic event, 'S' -> stop/info
    if (cmd == 'A' || cmd == 'a') {
      gPanicEventPending = true;
      BLE_NotifyAlertPayload("SEQ:" + String(gSeqAlert++) + ",ALARM:MANUAL");
      tone(ALARM_PIN, 3000, 150);
    } else if (cmd == 'S' || cmd == 's') {
      // sadece bilgi amaçlı stop mesajı
      BLE_NotifyAlertPayload("SEQ:" + String(gSeqAlert++) + ",ALARM:STOP");
      noTone(ALARM_PIN);
    }
  }
};

static void BLE_Init(const char* deviceName) {
  NimBLEDevice::init(deviceName);
  NimBLEDevice::setPower(ESP_PWR_LVL_P6);

  // MTU isteği (telefon kabul ederse büyür)
  NimBLEDevice::setMTU(185);

  NimBLEServer* pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  NimBLEService* pService = pServer->createService(BLE_SERVICE_UUID);

  gVitalsChar = pService->createCharacteristic(
    BLE_CHAR_VITALS_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
  );

  gAlertsChar = pService->createCharacteristic(
    BLE_CHAR_ALERTS_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
  );

  gCommandChar = pService->createCharacteristic(
    BLE_CHAR_COMMAND_UUID,
    NIMBLE_PROPERTY::WRITE
  );
  gCommandChar->setCallbacks(new CommandCallbacks());

  pService->start();

  NimBLEAdvertising* pAdv = NimBLEDevice::getAdvertising();
  pAdv->setName(deviceName);
  pAdv->addServiceUUID(BLE_SERVICE_UUID);
  pAdv->start();

  Serial.println("[BLE] Advertising started");
}

// ==========================================
// 3) SENSÖR / ALARM FONKSİYONLARI
// ==========================================
void akilliBekleme(unsigned long ms) {
  unsigned long baslangic = millis();
  while (millis() - baslangic < ms) {
    pox.update();
  }
}

void onBeatDetected() {
  beatCount++;
  lastBeatTime = millis();
}

float getFilteredBPM() {
  float sum = 0;
  int validCount = 0;
  float minV = 1000, maxV = 0;

  for (int i = 0; i < 5; i++) {
    if (bpmHistory[i] > 0) {
      if (bpmHistory[i] < minV) minV = bpmHistory[i];
      if (bpmHistory[i] > maxV) maxV = bpmHistory[i];
    }
  }

  for (int i = 0; i < 5; i++) {
    if (bpmHistory[i] > 0 && bpmHistory[i] != minV && bpmHistory[i] != maxV) {
      sum += bpmHistory[i];
      validCount++;
    }
  }

  if (validCount > 0) return sum / validCount;
  return 0;
}

void playAlarmSound() {
  if (millis() - lastToneTime > 600) {
    lastToneTime = millis();
    toneState = !toneState;
    if (toneState) tone(ALARM_PIN, 1200);
    else tone(ALARM_PIN, 1800);
  }
}

void stopAlarm() {
  noTone(ALARM_PIN);
}

// ==========================================
// 4) SETUP
// ==========================================
void setup() {
  Serial.begin(115200);

  pinMode(ALARM_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  stopAlarm();

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  BLE_Init("CDTP Bracelet");

  delay(500);
  Serial.println(">>> SISTEM BASLATILIYOR...");

  if (!mpu.setup(0x68)) {
    if (!mpu.setup(0x69)) {
      Serial.println("HATA: MPU9250 yok!");
      BLE_NotifyAlertPayload("SEQ:" + String(gSeqAlert++) + ",ERROR:MPU9250");
      while (1) { delay(1000); }
    }
  }
  mpu.calibrateAccelGyro();

  sonHareketZamani = millis();
  gLastMotionMillis = sonHareketZamani;

  if (!pox.begin()) {
    Serial.println("HATA: MAX30100 yok!");
    BLE_NotifyAlertPayload("SEQ:" + String(gSeqAlert++) + ",ERROR:MAX30100");
  } else {
    pox.setIRLedCurrent(MAX30100_LED_CURR_24MA);
    pox.setOnBeatDetectedCallback(onBeatDetected);
  }

  Serial.println(">>> HAZIR.");
  BLE_NotifyAlertPayload("SEQ:" + String(gSeqAlert++) + ",STATUS:READY");
}

// ==========================================
// 5) LOOP
// ==========================================
void loop() {
  pox.update();

  // =====================================================
  // BUTON (TEK SEFERLIK PANIC EVENT)
  // - Basınca: ALARM:MANUAL gönder + gPanicEventPending=true
  // - Sonraki VITALS paketinde flags bit0=1 gider
  // - Ardından otomatik ALARM:STOP gönderilir
  // =====================================================
  if (digitalRead(BUTTON_PIN) == LOW) {
    if (millis() - gLastPanicPress > 800) { // debounce
      gLastPanicPress = millis();

      gPanicEventPending = true;

      Serial.println("!!! PANIC BUTTON PRESSED -> SMS TRIGGER !!!");
      tone(ALARM_PIN, 3000, 150);

      // string alert (mobil SMS tetik için kolay)
      BLE_NotifyAlertPayload("SEQ:" + String(gSeqAlert++) + ",ALARM:MANUAL");
    }
  }

  // ==========================================
  // MPU: G + Activity + Inactivity Seconds
  // ==========================================
  if (mpu.update()) {
    unsigned long suAnkiZaman = millis();
    float ax = mpu.getAccX();
    float ay = mpu.getAccY();
    float az = mpu.getAccZ();

    float toplamIvme = sqrt(ax * ax + ay * ay + az * az);
    float sapma = fabs(toplamIvme - 1.0f);

    gLastTotalG = toplamIvme;

    // --- DÜŞME ---
    if (toplamIvme > DUSME_ESIGI) {
      Serial.println("\n!!! DUSME ALGILANDI !!!");
      BLE_NotifyAlertPayload("SEQ:" + String(gSeqAlert++) + ",ALERT:FALL");

      tone(ALARM_PIN, 2500);
      akilliBekleme(500);
      noTone(ALARM_PIN);

      sonHareketZamani = suAnkiZaman;
      gLastMotionMillis = suAnkiZaman;
      gInactivitySeconds = 0;
      gIsActive = true;
      alarmCaldimi = false;
      return;
    }

    // --- HAREKET ---
    if (sapma > HAREKET_ESIGI) {
      sonHareketZamani = suAnkiZaman;
      gLastMotionMillis = suAnkiZaman;
      gInactivitySeconds = 0;
      gIsActive = true;
      alarmCaldimi = false;
    } else {
      unsigned long gecen = suAnkiZaman - sonHareketZamani;
      gInactivitySeconds = (gecen / 1000);
      gIsActive = (gecen < 1000);
    }

    // --- HAREKETSIZLIK ALARMI ---
    unsigned long gecenSure = suAnkiZaman - sonHareketZamani;
    if (gecenSure >= HAREKETSIZLIK_SINIRI && !alarmCaldimi) {
      Serial.println("\n!!! 20 SN HAREKETSIZLIK ALARMI !!!");
      BLE_NotifyAlertPayload("SEQ:" + String(gSeqAlert++) + ",ALERT:INACTIVITY");

      tone(ALARM_PIN, 1500);
      akilliBekleme(3000);
      noTone(ALARM_PIN);

      alarmCaldimi = true;
      mpu.update(); mpu.update();
    }

    // Serial debug (MPU)
    if (suAnkiZaman - sonYazdirmaZamani >= 1000) {
      Serial.print("[MPU] G: "); Serial.print(toplamIvme, 2);
      Serial.print(" | Durum: ");
      if (gecenSure < 1000) Serial.print("Hareketli");
      else { Serial.print("Hareketsiz ("); Serial.print(gecenSure / 1000); Serial.print("s)"); }
      Serial.println();

      sonYazdirmaZamani = suAnkiZaman;
    }
  }

  // ==========================================
  // MAX30100 + BLE VITALS (BINARY)
  // ==========================================
  if (millis() - tsLastReport > REPORTING_PERIOD_MS) {
    float bpm = pox.getHeartRate();
    uint8_t spo2 = pox.getSpO2();

    bool recentBeat = (millis() - lastBeatTime) < 2000;
    bool fingerDetected = (spo2 > 0 && recentBeat);

    Serial.print("[MAX] ");

    int outBpm = 0;
    int outSpo2 = 0;

    if (fingerDetected) {
      if (bpm > 30 && bpm < 220) {
        bpmHistory[historyIndex] = bpm;
        historyIndex = (historyIndex + 1) % 5;
      }

      float filteredBPM = getFilteredBPM();

      if (beatCount > 3 && filteredBPM > 30 && filteredBPM < 220) stableReadings++;
      else stableReadings = 0;

      outBpm = (int)filteredBPM;
      outSpo2 = (int)spo2;

      Serial.print("Nabiz: "); Serial.print(outBpm);
      Serial.print(" | SpO2: %"); Serial.print(outSpo2);

      if (stableReadings >= 3) {
        if (filteredBPM < BPM_LOW_LIMIT || filteredBPM > BPM_HIGH_LIMIT) {
          Serial.print(" -> KRITIK!");
          BLE_NotifyAlertPayload("SEQ:" + String(gSeqAlert++) + ",ALERT:HR_CRITICAL");
          isEmergency = true;
        } else {
          Serial.print(" -> Normal");
          isEmergency = false;
        }
      } else {
        Serial.print(" (Olculuyor...)");
        isEmergency = false;
      }
    } else {
      Serial.print("PARMAK YOK");
      outBpm = 0;
      outSpo2 = 0;

      BLE_NotifyAlertPayload("SEQ:" + String(gSeqAlert++) + ",STATUS:NO_FINGER");

      beatCount = 0;
      stableReadings = 0;
      for (int i = 0; i < 5; i++) bpmHistory[i] = 0;
      isEmergency = false;
    }

    // flags: bit0 = PANIC_BUTTON
    uint8_t flags = 0;
    if (gPanicEventPending) flags |= 0x01;

    BLE_NotifyVitalsBinary(
      (uint16_t)(gSeqVitals++),
      (uint8_t)outBpm,
      (uint8_t)outSpo2,
      gLastTotalG,
      gIsActive,
      (uint16_t)gInactivitySeconds,
      flags
    );

    // Panic event bir kez paket içinde gönderildiyse otomatik STOP mesajı yolla
    if (gPanicEventPending) {
      gPanicEventPending = false;
      BLE_NotifyAlertPayload("SEQ:" + String(gSeqAlert++) + ",ALARM:STOP");
    }

    Serial.println();
    tsLastReport = millis();
  }

  // --- ALARM SES ---
  // Bu projede panic tek atımlık. Sürekli alarm istersen burada playAlarmSound açarız.
  // Şimdilik: sadece HR critical durumunda ses çalsın (istersen kapat)
  if (isEmergency) playAlarmSound();
  else stopAlarm();
}
