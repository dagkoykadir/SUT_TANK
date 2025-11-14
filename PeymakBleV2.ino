// SUT_TANK_BLE_GUNCEL_v2.ino
// Değişiklikler:
// 1) Röle aktif-seviyesi düzeltme: RELAY_ACTIVE_HIGH ile ters çalışan güç rölesi düzeltildi.
// 2) Sıcaklık güncelleme gecikmesi azaltıldı: bildirim periyodu düşürüldü, ayrıca delta değişimde anlık notify.
// 3) Uygulama komutları S6/S8/S9 butonlarıyla birebir aynı etkiyi oluşturuyor (emulated press).
// 4) Uygulama JSON çıktısına "wash" alanı eklendi. Karıştırma (confuseOpen) = yıkama kabul edilmiştir.
//    Eğer sahada "yıkama" farklı bir moda karşılık geliyorsa bildir, alanı ona eşleriz.

#include <math.h>
#include <EEPROM.h>
#include <Preferences.h>
#include <Wire.h>
#include <SPI.h>
#include <SparkFun_ADXL345.h>
Preferences prefs;
#include <Ticker.h>
Ticker myTimer;

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <ArduinoJson.h>

#if __has_include(<esp_mac.h>)
#include <esp_mac.h>
#endif
#if __has_include(<esp_bt_device.h>)
#include <esp_bt_device.h>
#endif

// ======= Röle aktif seviyesi (POWER tersliği için burayı değiştir) =======
// Donanım aktif-LOW ise false yap.
static const bool RELAY_ACTIVE_HIGH = true;

// Yardımcı: Röle yaz
static inline void relayWrite(int pin, bool on) {
  if (RELAY_ACTIVE_HIGH) digitalWrite(pin, on ? HIGH : LOW);
  else digitalWrite(pin, on ? LOW : HIGH);
}

static String _macToString(const uint8_t m[6]) {
  char buf[18];
  sprintf(buf, "%02X:%02X:%02X:%02X:%02X:%02X", m[0], m[1], m[2], m[3], m[4], m[5]);
  return String(buf);
}

static String _getBleMacString() {
  uint8_t mac[6] = { 0 };
  esp_read_mac(mac, ESP_MAC_BT);
  return _macToString(mac);
}

static String _macSuffix(const String &mac) {
  int p = mac.lastIndexOf(':');
  String last2 = mac.substring(p + 1);
  int p2 = mac.lastIndexOf(':', p - 1);
  String last3 = mac.substring(p2 + 1, p);
  return last3 + last2;  // "EEFF"
}

/* ADC_PIN - BUTTON */
#define ADC_PIN 39  //SENSOR_VN

#define BTN_S6 6
#define BTN_S7 7
#define BTN_S8 8
#define BTN_S9 9

// Röle pinleri
#define ROLE_A 2   //RLY2  -> Soğutucu ana röle (kompresör)
#define ROLE_B 22  //RLY3  -> Karıştırıcı
#define ROLE_C 21  //RLY1  -> Yardımcı soğutma/kompresör hattı

/* LEDs */
#define LED_A 27
#define LED_B 0

/* 7Segment Pins */
#define PIN_A 14
#define PIN_B 17
#define PIN_C 13
#define PIN_D 4
#define PIN_E 25
#define PIN_F 12
#define PIN_G 26
#define PIN_H 32

#define DISPLAY_A 5
#define DISPLAY_B 15
#define DISPLAY_C 16

#define NOKTA_OPEN 10
#define NOKTA_CLOSE 11
#define HARF_P 12
#define HARF_L 13
#define HARF_C 14
#define DISPLAY_CLOSE 15
#define HARF_r 16
#define HARF_TIRE 17
#define HARF_A 18
#define HARF_ 19

/* NTC */
#define NTC_PIN 36  //SENSOR_VP

/* EEPROM ADDR */
#define MODE_0 0
#define MODE_1 2
#define MODE_2 4
#define MODE_3 6
#define MODE_4 8
#define MODE_5 10
#define MODE_6 12
#define MODE_7 14
#define WORK_LOOP 16
#define SET_VALUE 18
#define MODE_8 20
#define MODE_9 22
#define SYSTEM_ACT 24
#define R_VALUE_ADD 26

/* DEBUG */
#define DEBUG 0

#define SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"

#define PREFERENCES_NAMESPACE "thermo-app-v2"

uint16_t NTC_Counter = 0;
uint16_t NTC_Ortalama = 0;
uint16_t NTC_Toplam = 0;
uint16_t tempNTC;
uint8_t tempCounter = 15;
uint16_t tempLastNTC = 0;

typedef struct btnVariables {
  uint16_t analogValue;
  bool value;
  bool state;
  bool _state;
} btnVariable;

btnVariable btn_S6;
btnVariable btn_S7;
btnVariable btn_S8;
btnVariable btn_S9;

btnVariable btn_cyro;

typedef struct tmrDelays {
  uint16_t set;
  uint16_t count;
  bool state;
  uint16_t counter;
  float fcounter;
} tmrDelay;

tmrDelay btnConfuseTime;
tmrDelay btnFreezeTime;
tmrDelay btnSettingsTime;
tmrDelay btnPowerTime;

tmrDelay _7segmentBlinkSendValue;
tmrDelay _7segmentBlinkSendState;

tmrDelay setValueTime;
tmrDelay _PArValueTime;
tmrDelay _PArCloseTime;

tmrDelay speedSetUp;
tmrDelay speedSetDown;

tmrDelay confuseFasilaOn;
tmrDelay confuseFasilaOff;

tmrDelay freezeRoleOn;
tmrDelay freezeRoleOff;

/* TEMP */
tmrDelay tempRead;
tmrDelay systemBlink;
tmrDelay tempPushData;

// ADXL
ADXL345 adxl = ADXL345(33);

tmrDelay newTempCheck;
tmrDelay newTempUpSet;
tmrDelay newTempDownSet;
tmrDelay newVersionNo;

/*systemAcceleration blink*/
tmrDelay systemAccelerationBlink;
tmrDelay BLESendDataDelay;

/* Timer Variable  */
bool tmrTick = false;

/* _7Segment Variable */
uint8_t _DisplayA = 0;
uint8_t _DisplayB = 0;
uint8_t _DisplayC = 0;
bool noktaOne = false;
bool noktaTwo = false;
bool noktaThree = false;

char _send[99];

/* Mode Function Variable */
uint8_t PAr_Value = 0;

float minFreezeSet = 3.0;    //P.00
float tempHisterezis = 0.5;  //P.01
float milkHisterezisSetValue = minFreezeSet;
float tempCalibration = 3.0;  //P.02
float maxFreezeSet = 38.0;    //P.03
float confuseOnTime = 0.3;    //P.04
float confuseOffTime = 1.0;   //P.05
float coverState = 0.1;       //P.06
float coverAngleValue = 0.3;  //P.07
float roleOnValue = 0.2;      //P.08
float roleOffValue = 0.4;     //P.09
float systemActive = 0.0;
float Rvalue = 0.0;

/* Flags */

bool stateTempShow = false;
bool stateTempHisterezis = false;
bool statePArShow = false;
bool stateModeShow = false;
bool statePAR_0 = false;
bool statePAR_1 = false;
bool statePAR_2 = false;
bool statePAR_3 = false;
bool statePAR_4 = false;
bool statePAR_5 = false;
bool statePAR_6 = false;
bool statePAR_7 = false;
bool statePAR_8 = false;
bool statePAR_9 = false;
bool statePAR_10 = false;

enum Buttons {
  CONFUSE = 0,
  FREEZE,
  SETTINGS,
  POWER,
  TEMP_HISTEREZIS,
  PAR,
  PAR_ACTIVE,
  PAR_0,
  PAR_0_ACTIVE,
  PAR_1,
  PAR_1_ACTIVE,
  PAR_2,
  PAR_2_ACTIVE,
  PAR_3,
  PAR_3_ACTIVE,
  PAR_4,
  PAR_4_ACTIVE,
  PAR_5,
  PAR_5_ACTIVE,
  PAR_6,
  PAR_6_ACTIVE,
  PAR_7,
  PAR_7_ACTIVE,
  PAR_8,
  PAR_8_ACTIVE,
  PAR_9,
  PAR_9_ACTIVE,
  PAR_10,
  PAR_10_ACTIVE,
  PAR_SAVE,
  PAR_CLOSE,
};

uint8_t selectButton = 2;

bool isMixerOn = false;
bool isDeviceOn = false;
bool isCoolerOn = false;

bool systemOpen = false;
bool confuseOpen = false;  // mixer açık
bool freezeOpen = false;   // soğutma döngüsü aktif
bool systemAcceleration = false;
bool systemAccelerationPassive = false;
bool stateFasila = false;
bool stateAcceleration = false;
bool stateGyro_Value = false;

uint8_t counterGyro = 0;
uint16_t cont8 = 0;

float R1 = 10000;
float logR2, R2, T, demo;
float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;

float tempScreenValue = 0.0;
float nextTempValue = 0.0;
float _tempNTC = 0.0;

bool newTempFlag = false;

bool stateVersion_ = false;
bool stateVersionNo = false;
bool stateVersion_FULL = false;

BLECharacteristic *pTxCharacteristic;
bool deviceConnected = false;

// Hızlı bildirim için throttle
static uint32_t lastNotifyMs = 0;
static const uint32_t NOTIFY_MIN_PERIOD_MS = 100;  // 100ms
static float lastNotifiedTemp = -1000.0;

// App tarafından buton emülasyonu için kenar tetikleyiciler
volatile bool appPressS6 = false;
volatile bool appPressS8 = false;
volatile bool appPressS9 = false;

void sendCurrentStateAndSettings();
void eepromWrite();
void closeRoleAndLed();

// Emulated press helpers
static inline void emulateButtonPress(uint8_t logicalBtn) {
  switch (logicalBtn) {
    case BTN_S6:
      if (!btn_S6.state) {
        btnConfuseTime.set = 20;
        btn_S6.state = true;
      }
      break;
    case BTN_S8:
      if (!btn_S8.state) {
        btnPowerTime.set = 20;
        btn_S8.state = true;
      }
      break;
    case BTN_S9:
      if (!btn_S9.state) {
        btnFreezeTime.set = 20;
        btn_S9.state = true;
      }
      break;
  }
}

// BLE Write -> JSON komutları
class MyCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) override {
    String raw = pCharacteristic->getValue();
    if (raw.isEmpty()) return;

    String command = String(raw.c_str());
    command.trim();

    if (command == "GET_SETTINGS") {
      sendCurrentStateAndSettings();
      return;
    }

    if (!(command.startsWith("{") && command.endsWith("}"))) return;

    StaticJsonDocument<1024> doc;
    DeserializationError err = deserializeJson(doc, command);
    if (err) {
      Serial.print(F("JSON hata: "));
      Serial.println(err.c_str());
      return;
    }

    bool settingsChanged = false;

    // --- AYARLAR (params) - sistem kapalıyken de değiştirilebilir ---
    if (doc.containsKey("params")) {
      JsonObject p = doc["params"];
      auto upd = [&](const char *key, float &var) {
        if (p.containsKey(key)) {
          var = p[key].as<float>();
          settingsChanged = true;
        }
      };
      upd("minFreezeSet", minFreezeSet);
      upd("tempHisterezis", tempHisterezis);
      upd("milkHisterezisSetValue", milkHisterezisSetValue);
      upd("tempCalibration", tempCalibration);
      upd("maxFreezeSet", maxFreezeSet);
      upd("confuseOnTime", confuseOnTime);
      upd("confuseOffTime", confuseOffTime);
      upd("coverState", coverState);
      upd("coverAngleValue", coverAngleValue);
      upd("roleOnValue", roleOnValue);
      upd("roleOffValue", roleOffValue);

      if (settingsChanged) {
        eepromWrite();
        Serial.println("Ayarlar güncellendi ve kaydedildi.");
      }
    }

    // --- GÜVENLİK KONTROLÜ: Sistem kapalıyken mixer/cooler kontrolü yapılamaz ---
    if (!systemOpen) {
      if (doc.containsKey("mixer") || doc.containsKey("cooler")) {
        Serial.println("HATA: Sistem kapalı! Mixer/Cooler kontrolü yapılamaz.");
        return;  // Komutları işleme
      }
    }

    // --- BUTON EŞLEMELERİ: Sadece sistem açıkken çalışır ---
    if (systemOpen) {
      // MIXER = BTN_S6 toggle
      if (doc.containsKey("mixer")) {
        int req = doc["mixer"].as<int>();  // 0/1
        bool wantOn = (req == 1);
        bool curOn = confuseOpen;
        if (wantOn != curOn) appPressS6 = true;
      }

      // COOLER = BTN_S9 toggle
      if (doc.containsKey("cooler")) {
        int req = doc["cooler"].as<int>();  // 0/1
        bool wantOn = (req == 1);
        bool curOn = freezeOpen;
        if (wantOn != curOn) appPressS9 = true;
      }
    }

    // POWER = BTN_S8 toggle (sistem kapalıyken de çalışabilir)
    if (doc.containsKey("power") || doc.containsKey("systemOpen")) {
      bool wantOn = false;
      if (doc.containsKey("power")) {
        if (doc["power"].is<int>()) wantOn = (doc["power"].as<int>() == 1);
        else wantOn = doc["power"].as<bool>();
      } else {
        if (doc["systemOpen"].is<int>()) wantOn = (doc["systemOpen"].as<int>() == 1);
        else wantOn = doc["systemOpen"].as<bool>();
      }
      if (wantOn != systemOpen) appPressS8 = true;
    }
  }
};
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    deviceConnected = true;
    Serial.println("Cihaz bağlandı.");
  }
  void onDisconnect(BLEServer *pServer) {
    deviceConnected = false;
    Serial.println("Cihaz bağlantısı kesildi, tekrar yayın...");
    BLEDevice::startAdvertising();
  }
};

uint8_t coverAngleEx = 0;
float tempCalibrationLast = 3.0;
uint32_t count;

void bl_init(void) {
  String __mac = _getBleMacString();
  String __devName = String("SUT_TANK_") + _macSuffix(__mac);
  BLEDevice::init(__devName.c_str());
  Serial.println(String("BLE MAC  : ") + __mac);
  Serial.println(String("BLE Name : ") + __devName);

  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pTxCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_TX,
    BLECharacteristic::PROPERTY_NOTIFY);
  pTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_RX,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  pRxCharacteristic->setCallbacks(new MyCharacteristicCallbacks());

  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pServer->getAdvertising()->start();
}

void bl_process(void) {
  static uint32_t _lastNotify = 0;
  const uint32_t PERIOD_MS = 100;  // daha hızlı
  uint32_t now = millis();
  if (deviceConnected && now - _lastNotify >= PERIOD_MS) {
    _lastNotify = now;
    sendCurrentStateAndSettings();
  }
}

void sendCurrentStateAndSettings() {
  if (!deviceConnected) return;

  StaticJsonDocument<512> doc;
  doc["temp"] = String(tempScreenValue, 1);
  doc["mixer"] = confuseOpen ? 1 : 0;  // Karıştırma gerçeği
  doc["power"] = systemOpen ? 1 : 0;
  doc["cooler"] = freezeOpen ? 1 : 0;
  doc["wash"] = confuseOpen ? 1 : 0;  // "Yıkama" = karıştırma varsayımı

  JsonObject paramsObj = doc.createNestedObject("params");
  paramsObj["minFreezeSet"] = minFreezeSet;
  paramsObj["tempHisterezis"] = tempHisterezis;
  paramsObj["milkHisterezisSetValue"] = milkHisterezisSetValue;
  paramsObj["tempCalibration"] = tempCalibration;
  paramsObj["maxFreezeSet"] = maxFreezeSet;
  paramsObj["confuseOnTime"] = confuseOnTime;
  paramsObj["confuseOffTime"] = confuseOffTime;
  paramsObj["coverState"] = coverState;
  paramsObj["coverAngleValue"] = coverAngleValue;
  paramsObj["roleOnValue"] = roleOnValue;
  paramsObj["roleOffValue"] = roleOffValue;

  String output;
  serializeJson(doc, output);
  output += "\n";

  pTxCharacteristic->setValue(output.c_str());
  pTxCharacteristic->notify();

  Serial.print("Birleşik Veri Paketi Gönderildi: ");
  Serial.print(output);
}

void IRAM_ATTR IntTimer() {
  if (count++ >= 10) { count = 0; }
  tmrTick = true;
}

void activeRoleAndLed(void) {
  // SOGUTUCU
  if (!freezeRoleOn.state) { freezeRoleOn.set = 1000; }

  // KARISTIRICI
  digitalWrite(LED_B, HIGH);
  relayWrite(ROLE_B, true);

  // KOMPRESSOR
  relayWrite(ROLE_C, true);

  confuseFasilaOn.set = 0;
  confuseFasilaOn.counter = 0;
  confuseFasilaOff.set = 0;
  confuseFasilaOff.counter = 0;
}

void closeRoleAndLed(void) {
  // SOGUTUCU
  if (!freezeRoleOff.state) { freezeRoleOff.set = 1000; }

  // KARISTIRICI
  if (!stateAcceleration) digitalWrite(LED_B, LOW);
  relayWrite(ROLE_B, false);

  // KOMPRESSOR
  relayWrite(ROLE_C, false);

  confuseFasilaOn.set = 0;
  confuseFasilaOn.counter = 0;
  confuseFasilaOff.set = 0;
  confuseFasilaOff.counter = 0;
}

void Gyro_init(void) {
  adxl.powerOn();
  adxl.setRangeSetting(2);
  adxl.setSpiBit(0);
  adxl.setActivityXYZ(1, 0, 0);
  adxl.setActivityThreshold(75);
  adxl.setInactivityXYZ(1, 0, 0);
  adxl.setInactivityThreshold(75);
  adxl.setTimeInactivity(10);
  adxl.setTapDetectionOnXYZ(1, 0, 0);
  adxl.setTapThreshold(50);
  adxl.setTapDuration(15);
  adxl.setDoubleTapLatency(80);
  adxl.setDoubleTapWindow(200);
  adxl.setFreeFallThreshold(7);
  adxl.setFreeFallDuration(30);
}

void LedMod(uint8_t pin, bool mod) {
  digitalWrite(pin, mod ? HIGH : LOW);
}

void Gyro_angle(void) {
  int x, y, z;
  adxl.readAccel(&x, &y, &z);
  if (coverAngleValue == 0.1) coverAngleEx = 30;
  else if (coverAngleValue == 0.2) coverAngleEx = 50;
  else if (coverAngleValue == 0.3) coverAngleEx = 80;
  else coverAngleEx = 0;

  if (confuseOpen || freezeOpen) {
    if (z <= -(80 + coverAngleEx)) {
      if (++counterGyro >= 10) {
        counterGyro = 0;
        stateGyro_Value = true;
      }
    } else if (z >= -(50 + coverAngleEx) && z <= -(10 + coverAngleEx)) {
      if (++counterGyro >= 5) {
        counterGyro = 0;
        stateGyro_Value = false;
      }
    } else if (z >= -(9 + coverAngleEx)) {
      if (++counterGyro >= 5) {
        counterGyro = 0;
        stateGyro_Value = false;
      }
    }
  } else {
    counterGyro = 0;
    stateGyro_Value = false;
  }
}

uint8_t btnstate = 0;
uint8_t btnstateLast = 0;

uint8_t detectedButton = 0;
uint16_t lastStableValue = 0;
uint32_t lastChangeTime = 0;
uint16_t lastRawValue = 0;
uint16_t StableValuecont = 0;

uint8_t analogToDigitalRead(void) {
  // ADC okuması
  uint16_t val = analogRead(ADC_PIN);
  // Serial.println(val);  // Debug istersen açarsın

  // Anlık aday buton (raw)
  uint8_t candidate = 0;

  if (val >= 1900 && val < 2000) {
    candidate = 8;  // S8
    // Serial.println("S8 raw");
  } else if (val >= 2700 && val < 2800) {
    candidate = 6;  // S6
    // Serial.println("S6 raw");
  } else if (val >= 3300 && val < 3400) {
    candidate = 7;  // S7
    // Serial.println("S7 raw");
  } else if (val >= 4070 && val <= 4095) {
    candidate = 9;  // S9
    // Serial.println("S9 raw");
  } else {
    candidate = 0;  // hiçbir buton yok
  }

  // Debounce / stabilizasyon için statikler
  static uint8_t  lastCandidate = 0;     // bir önceki aday
  static uint16_t sameCount = 0;        // kaç ms aynı aday görüldü
  static uint8_t  stableButton = 0;      // dışarı döndürülecek "stabil" buton
  static uint16_t releaseCount = 0;     // bırakma için sayaç

  const uint16_t  PRESS_THRESHOLD   = 8;  // ~5ms aynı buton görülürse "basılı" say
  const uint16_t  RELEASE_THRESHOLD = 8;  // ~5ms boyunca hiçbir buton yoksa "bıraktı" say

  // Aday aynı mı ?
  if (candidate == lastCandidate) {
    if (sameCount < 0xFFFF) sameCount++;
  } else {
    lastCandidate = candidate;
    sameCount = 0;
  }

  // Bir buton aralığında mıyız?
  if (candidate != 0) {
    releaseCount = 0;  // bırakma sayacı sıfırla

    // Yeterince uzun süredir aynı buton görülüyorsa: stabil buton budur
    if (sameCount >= PRESS_THRESHOLD) {
      stableButton = candidate;
    }
  } else {
    // Hiçbir buton aralığında değiliz → bırakma süresini say
    if (stableButton != 0) {
      if (++releaseCount >= RELEASE_THRESHOLD) {
        // Yeterince uzun süre buton yok → butonu bırakmış kabul et
        stableButton = 0;
      }
    } else {
      releaseCount = 0;
    }
  }

  // checkButtons() artık her çağrıda "stabil" butonu görecek:
  // - basılıyken: hep 6, 7, 8, 9
  // - bırakınca: 0
  return stableButton;
}


void eepromWrite(void) {
  prefs.begin("settings", false);
  prefs.putFloat("minFreezeSet", minFreezeSet);
  prefs.putFloat("tempHisterezis", tempHisterezis);
  prefs.putFloat("tempCalibration", tempCalibration);
  prefs.putFloat("maxFreezeSet", maxFreezeSet);
  prefs.putFloat("confuseOnTime", confuseOnTime);
  prefs.putFloat("confuseOffTime", confuseOffTime);
  prefs.putFloat("coverState", coverState);
  prefs.putFloat("coverAngleValue", coverAngleValue);
  prefs.putFloat("roleOnValue", roleOnValue);
  prefs.putFloat("roleOffValue", roleOffValue);
  prefs.putFloat("milkHisterezisSetValue", milkHisterezisSetValue);
  prefs.putFloat("systemActive", systemActive);
  prefs.putFloat("Rvalue", Rvalue);
  prefs.end();
}

void eepromRead(void) {
  prefs.begin("settings", true);
  minFreezeSet = prefs.getFloat("minFreezeSet", 3.0);
  tempHisterezis = prefs.getFloat("tempHisterezis", 0.5);
  tempCalibration = prefs.getFloat("tempCalibration", 3.0);
  maxFreezeSet = prefs.getFloat("maxFreezeSet", 38.0);
  confuseOnTime = prefs.getFloat("confuseOnTime", 0.3);
  confuseOffTime = prefs.getFloat("confuseOffTime", 1.0);
  coverState = prefs.getFloat("coverState", 0.1);
  coverAngleValue = prefs.getFloat("coverAngleValue", 0.3);
  roleOnValue = prefs.getFloat("roleOnValue", 0.2);
  roleOffValue = prefs.getFloat("roleOffValue", 0.4);
  milkHisterezisSetValue = prefs.getFloat("milkHisterezisSetValue", minFreezeSet);
  systemActive = prefs.getFloat("systemActive", 0.0);
  Rvalue = prefs.getFloat("Rvalue", 0.0);
  prefs.end();
}

uint8_t funcModeState(uint8_t value) {
  switch (value) {
    case 0: selectButton = PAR_0; break;
    case 1: selectButton = PAR_1; break;
    case 2: selectButton = PAR_2; break;
    case 3: selectButton = PAR_3; break;
    case 4: selectButton = PAR_4; break;
    case 5: selectButton = PAR_5; break;
    case 6: selectButton = PAR_6; break;
    case 7: selectButton = PAR_7; break;
    case 8: selectButton = PAR_8; break;
    case 9: selectButton = PAR_9; break;
    case 10: selectButton = PAR_10; break;
    default: return -1;
  }
  return selectButton;
}

float setSpeedUp(float value) {
  if (speedSetUp.set != 0) {
    if (++speedSetUp.count >= speedSetUp.set) {
      speedSetUp.count = 0;
      speedSetUp.state = !speedSetUp.state;
      value += 0.1;
      _PArCloseTime.counter = 0;
      setValueTime.counter = 0;
      speedSetUp.set = 150;
    }
  }
  return value;
}

float setSpeedDown(float value) {
  if (speedSetDown.set != 0) {
    if (++speedSetDown.count >= speedSetDown.set) {
      speedSetDown.count = 0;
      speedSetDown.state = !speedSetDown.state;
      if (stateTempHisterezis) {
        value -= 0.1;
        if (value <= minFreezeSet) value = minFreezeSet;
      } else if (statePAR_1) {
        value -= 0.1;
        if (value <= 0.0) value = 0.0;
      } else {
        value -= 0.1;
        if (value <= 3.0) value = 3.0;
      }
      _PArCloseTime.counter = 0;
      setValueTime.counter = 0;
      speedSetDown.set = 150;
    }
  }
  return value;
}

// 7-segment, tempAndILT, _7SegmentDisplay fonksiyonları
// (Aşağıdaki büyük blok, orijinal kodla aynı, sadece röle yazımları relayWrite ile güncellendi)

void _7SegmentDisplay(uint8_t value) {
  switch (value) {
    case 0:
      digitalWrite(PIN_A, LOW);
      digitalWrite(PIN_B, LOW);
      digitalWrite(PIN_C, LOW);
      digitalWrite(PIN_D, LOW);
      digitalWrite(PIN_E, LOW);
      digitalWrite(PIN_F, LOW);
      digitalWrite(PIN_G, HIGH);
      break;
    case 1:
      digitalWrite(PIN_A, HIGH);
      digitalWrite(PIN_B, LOW);
      digitalWrite(PIN_C, LOW);
      digitalWrite(PIN_D, HIGH);
      digitalWrite(PIN_E, HIGH);
      digitalWrite(PIN_F, HIGH);
      digitalWrite(PIN_G, HIGH);
      break;
    case 2:
      digitalWrite(PIN_A, LOW);
      digitalWrite(PIN_B, LOW);
      digitalWrite(PIN_C, HIGH);
      digitalWrite(PIN_D, LOW);
      digitalWrite(PIN_E, LOW);
      digitalWrite(PIN_F, HIGH);
      digitalWrite(PIN_G, LOW);
      break;
    case 3:
      digitalWrite(PIN_A, LOW);
      digitalWrite(PIN_B, LOW);
      digitalWrite(PIN_C, LOW);
      digitalWrite(PIN_D, LOW);
      digitalWrite(PIN_E, HIGH);
      digitalWrite(PIN_F, HIGH);
      digitalWrite(PIN_G, LOW);
      break;
    case 4:
      digitalWrite(PIN_A, HIGH);
      digitalWrite(PIN_B, LOW);
      digitalWrite(PIN_C, LOW);
      digitalWrite(PIN_D, HIGH);
      digitalWrite(PIN_E, HIGH);
      digitalWrite(PIN_F, LOW);
      digitalWrite(PIN_G, LOW);
      break;
    case 5:
      digitalWrite(PIN_A, LOW);
      digitalWrite(PIN_B, HIGH);
      digitalWrite(PIN_C, LOW);
      digitalWrite(PIN_D, LOW);
      digitalWrite(PIN_E, HIGH);
      digitalWrite(PIN_F, LOW);
      digitalWrite(PIN_G, LOW);
      break;
    case 6:
      digitalWrite(PIN_A, LOW);
      digitalWrite(PIN_B, HIGH);
      digitalWrite(PIN_C, LOW);
      digitalWrite(PIN_D, LOW);
      digitalWrite(PIN_E, LOW);
      digitalWrite(PIN_F, LOW);
      digitalWrite(PIN_G, LOW);
      break;
    case 7:
      digitalWrite(PIN_A, LOW);
      digitalWrite(PIN_B, LOW);
      digitalWrite(PIN_C, LOW);
      digitalWrite(PIN_D, HIGH);
      digitalWrite(PIN_E, HIGH);
      digitalWrite(PIN_F, HIGH);
      digitalWrite(PIN_G, HIGH);
      break;
    case 8:
      digitalWrite(PIN_A, LOW);
      digitalWrite(PIN_B, LOW);
      digitalWrite(PIN_C, LOW);
      digitalWrite(PIN_D, LOW);
      digitalWrite(PIN_E, LOW);
      digitalWrite(PIN_F, LOW);
      digitalWrite(PIN_G, LOW);
      break;
    case 9:
      digitalWrite(PIN_A, LOW);
      digitalWrite(PIN_B, LOW);
      digitalWrite(PIN_C, LOW);
      digitalWrite(PIN_D, LOW);
      digitalWrite(PIN_E, HIGH);
      digitalWrite(PIN_F, LOW);
      digitalWrite(PIN_G, LOW);
      break;
    case NOKTA_OPEN: digitalWrite(PIN_H, LOW); break;
    case NOKTA_CLOSE: digitalWrite(PIN_H, HIGH); break;
    case HARF_P:
      digitalWrite(PIN_A, LOW);
      digitalWrite(PIN_B, LOW);
      digitalWrite(PIN_C, HIGH);
      digitalWrite(PIN_D, HIGH);
      digitalWrite(PIN_E, LOW);
      digitalWrite(PIN_F, LOW);
      digitalWrite(PIN_G, LOW);
      break;
    case HARF_L:
      digitalWrite(PIN_A, LOW);
      digitalWrite(PIN_B, LOW);
      digitalWrite(PIN_C, LOW);
      digitalWrite(PIN_D, HIGH);
      digitalWrite(PIN_E, HIGH);
      digitalWrite(PIN_F, HIGH);
      digitalWrite(PIN_G, LOW);
      break;
    case HARF_C:
      digitalWrite(PIN_A, LOW);
      digitalWrite(PIN_B, HIGH);
      digitalWrite(PIN_C, HIGH);
      digitalWrite(PIN_D, LOW);
      digitalWrite(PIN_E, LOW);
      digitalWrite(PIN_F, LOW);
      digitalWrite(PIN_G, HIGH);
      break;
    case DISPLAY_CLOSE:
      digitalWrite(PIN_A, HIGH);
      digitalWrite(PIN_B, HIGH);
      digitalWrite(PIN_C, HIGH);
      digitalWrite(PIN_D, HIGH);
      digitalWrite(PIN_E, HIGH);
      digitalWrite(PIN_F, HIGH);
      digitalWrite(PIN_G, HIGH);
      break;
    case HARF_r:
      digitalWrite(PIN_A, HIGH);
      digitalWrite(PIN_B, HIGH);
      digitalWrite(PIN_C, HIGH);
      digitalWrite(PIN_D, HIGH);
      digitalWrite(PIN_E, LOW);
      digitalWrite(PIN_F, HIGH);
      digitalWrite(PIN_G, LOW);
      break;
    case HARF_TIRE:
      digitalWrite(PIN_A, HIGH);
      digitalWrite(PIN_B, HIGH);
      digitalWrite(PIN_C, HIGH);
      digitalWrite(PIN_D, HIGH);
      digitalWrite(PIN_E, HIGH);
      digitalWrite(PIN_F, HIGH);
      digitalWrite(PIN_G, LOW);
      break;
    case HARF_A:
      digitalWrite(PIN_A, LOW);
      digitalWrite(PIN_B, LOW);
      digitalWrite(PIN_C, LOW);
      digitalWrite(PIN_D, HIGH);
      digitalWrite(PIN_E, LOW);
      digitalWrite(PIN_F, LOW);
      digitalWrite(PIN_G, LOW);
      break;
    case HARF_:
      digitalWrite(PIN_A, HIGH);
      digitalWrite(PIN_B, HIGH);
      digitalWrite(PIN_C, HIGH);
      digitalWrite(PIN_D, HIGH);
      digitalWrite(PIN_E, HIGH);
      digitalWrite(PIN_F, HIGH);
      digitalWrite(PIN_G, LOW);
      break;
  }
}

void tempAndILT(void) {
  if (NTC_Counter <= 7) NTC_Toplam += (4095 - analogRead(NTC_PIN));
  if (NTC_Counter == 7) NTC_Ortalama = NTC_Toplam / 7;
  if (NTC_Counter > 7) NTC_Ortalama = (NTC_Ortalama * 6 + (4095 - analogRead(NTC_PIN))) / 7;
  if (NTC_Counter < 8) NTC_Counter++;
  tempNTC = NTC_Ortalama;
}

// 7-segment yazma
void _7SegmentWrite(float value, bool sayi, bool noktaA, bool noktaB, bool noktaC) {
  uint8_t number = 0;
  if (_7segmentBlinkSendValue.set != 0) {
    if (++_7segmentBlinkSendValue.count >= _7segmentBlinkSendValue.set) {
      _7segmentBlinkSendValue.count = 0;
      _7segmentBlinkSendValue.state = !_7segmentBlinkSendValue.state;
      _7segmentBlinkSendValue.counter += 1;

      if (sayi) {
        sprintf(_send, "%d", (int)(value * 10));
        number = strlen(_send);
        if (number == 3) {
          _DisplayA = _send[0] - '0';
          _DisplayB = _send[1] - '0';
          _DisplayC = _send[2] - '0';
        } else if (number == 2) {
          _DisplayB = _send[0] - '0';
          _DisplayC = _send[1] - '0';
        } else if (number == 1) {
          _DisplayB = 0;
          _DisplayC = _send[0] - '0';
        }
      }

      if (_7segmentBlinkSendValue.counter == 1) {
        if (number == 3 || !sayi) {
          digitalWrite(DISPLAY_A, HIGH);
          digitalWrite(DISPLAY_B, HIGH);
          digitalWrite(DISPLAY_C, LOW);
          _7SegmentDisplay(_DisplayA);
          _7SegmentDisplay(noktaA ? NOKTA_OPEN : NOKTA_CLOSE);
        } else {
          digitalWrite(DISPLAY_C, HIGH);
        }
      } else if (_7segmentBlinkSendValue.counter == 2) {
        digitalWrite(DISPLAY_A, HIGH);
        digitalWrite(DISPLAY_B, LOW);
        digitalWrite(DISPLAY_C, HIGH);
        _7SegmentDisplay(_DisplayB);
        _7SegmentDisplay(noktaB ? NOKTA_OPEN : NOKTA_CLOSE);
      } else if (_7segmentBlinkSendValue.counter == 3) {
        digitalWrite(DISPLAY_A, LOW);
        digitalWrite(DISPLAY_B, HIGH);
        digitalWrite(DISPLAY_C, HIGH);
        _7SegmentDisplay(_DisplayC);
        _7SegmentDisplay(noktaC ? NOKTA_OPEN : NOKTA_CLOSE);
        _7segmentBlinkSendValue.counter = 0;
      }
    }
  }
}

// Buton tarama
void checkButtons(void) {

  uint8_t currentButton = analogToDigitalRead();

  if (BTN_S8 == currentButton) {
    if (!btn_S8.state) {
      btnPowerTime.set = 20;
      btn_S8.state = true;
    }
  } else {
    if (btn_S8.state) { btn_S8.state = false; }
  }

  if (systemOpen) {
    if (BTN_S6 == currentButton) {
      if (!btn_S6.state) {
        btnConfuseTime.set = 20;
        btn_S6.state = true;
      }
    } else {
      if (btn_S6.state) {
        speedSetUp.set = 0;
        btn_S6.state = false;
      }
    }

    if (BTN_S7 == currentButton) {
      if (!btn_S7.state) {
        btnSettingsTime.set = 20;
        btn_S7.state = true;
      }
    } else {
      if (btn_S7.state) {
        _PArValueTime.set = 0;
        btn_S7.state = false;
      }
    }

    if (BTN_S9 == currentButton) {
      if (!btn_S9.state) {
        btnFreezeTime.set = 20;
        btn_S9.state = true;
      }
    } else {
      if (btn_S9.state) {
        speedSetDown.set = 0;
        btn_S9.state = false;
      }
    }
  }

  if (systemAccelerationPassive) {
    if (btn_cyro.value != stateGyro_Value) {
      if (btn_cyro.value == stateGyro_Value) {
        if (!btn_cyro.state) {
          systemAcceleration = true;
          btn_cyro.state = true;
        }
      } else {
        if (btn_cyro.state) {
          systemAcceleration = false;
          btn_cyro.state = false;
        }
      }
    }
  }
}

void setup() {
  //Serial.begin(9600);

  pinMode(PIN_A, OUTPUT);
  digitalWrite(PIN_A, HIGH);
  pinMode(PIN_B, OUTPUT);
  digitalWrite(PIN_B, HIGH);
  pinMode(PIN_C, OUTPUT);
  digitalWrite(PIN_C, HIGH);
  pinMode(PIN_D, OUTPUT);
  digitalWrite(PIN_D, HIGH);
  pinMode(PIN_E, OUTPUT);
  digitalWrite(PIN_E, HIGH);
  pinMode(PIN_F, OUTPUT);
  digitalWrite(PIN_F, HIGH);
  pinMode(PIN_G, OUTPUT);
  digitalWrite(PIN_G, HIGH);
  pinMode(PIN_H, OUTPUT);
  digitalWrite(PIN_H, HIGH);
  pinMode(DISPLAY_A, OUTPUT);
  digitalWrite(DISPLAY_A, HIGH);
  pinMode(DISPLAY_B, OUTPUT);
  digitalWrite(DISPLAY_B, HIGH);
  pinMode(DISPLAY_C, OUTPUT);
  digitalWrite(DISPLAY_C, HIGH);

  pinMode(LED_A, OUTPUT);
  digitalWrite(LED_A, LOW);
  pinMode(LED_B, OUTPUT);
  digitalWrite(LED_B, LOW);

  pinMode(ROLE_A, OUTPUT);
  relayWrite(ROLE_A, false);
  pinMode(ROLE_B, OUTPUT);
  relayWrite(ROLE_B, false);
  pinMode(ROLE_C, OUTPUT);
  relayWrite(ROLE_C, false);

  myTimer.attach_ms(1, IntTimer);

  systemBlink.set = 100;
  tempPushData.set = 100;
  systemAccelerationBlink.set = 250;

  //analogReadResolution(10);
  //eepromWrite(); delay(100);
  eepromRead();
  tempCalibrationLast = tempCalibration;

  if ((systemActive * 10) == 0) {
    freezeOpen = false;
  } else {
    freezeOpen = true;
    btnFreezeTime.state = true;
  }

  systemAccelerationPassive = ((coverState * 10) != 0);

  systemOpen = true;
  btnPowerTime.state = true;

  stateVersion_FULL = true;
  newVersionNo.set = 250;

  newTempCheck.set = 100;
  tempRead.set = 1;

  BLESendDataDelay.set=2000;

  // Önceki çıkış durumlarını çek
  prefs.begin("settings", true);
  isMixerOn = prefs.getBool("mixerState", false);
  isDeviceOn = prefs.getBool("powerState", false);
  isCoolerOn = prefs.getBool("coolerState", false);
  prefs.end();

  relayWrite(ROLE_B, isMixerOn);
  relayWrite(ROLE_A, isDeviceOn);
  relayWrite(ROLE_C, isCoolerOn);

  // // Parametreler
  // prefs.begin("settings", true);
  // minFreezeSet = prefs.getFloat("minFreezeSet", minFreezeSet);
  // tempHisterezis = prefs.getFloat("tempHisterezis", tempHisterezis);
  // tempCalibration = prefs.getFloat("tempCalibration", tempCalibration);
  // maxFreezeSet = prefs.getFloat("maxFreezeSet", maxFreezeSet);
  // confuseOnTime = prefs.getFloat("confuseOnTime", confuseOnTime);
  // confuseOffTime = prefs.getFloat("confuseOffTime", confuseOffTime);
  // coverState = prefs.getFloat("coverState", coverState);
  // coverAngleValue = prefs.getFloat("coverAngleValue", coverAngleValue);
  // roleOnValue = prefs.getFloat("roleOnValue", roleOnValue);
  // roleOffValue = prefs.getFloat("roleOffValue", roleOffValue);
  // milkHisterezisSetValue = prefs.getFloat("milkHisterezisSetValue", milkHisterezisSetValue);
  // Rvalue = prefs.getFloat("Rvalue", Rvalue);
  // prefs.end();

  bl_init();
}

bool flag10 = false;


void loop() {
  // App tarafından set edilen emüle basışları tek-kenarda uygula

  if (appPressS6) {
    emulateButtonPress(BTN_S6);
    appPressS6 = false;
  }
  if (appPressS8) {
    emulateButtonPress(BTN_S8);
    appPressS8 = false;
  }
  if (appPressS9) {
    emulateButtonPress(BTN_S9);
    appPressS9 = false;
  }

  // checkButtons();

  if (systemOpen) {
    if (!systemAcceleration) {
      stateAcceleration = false;

      if (confuseOpen) {
        LedMod(LED_B, HIGH);
        relayWrite(ROLE_B, true);
        LedMod(LED_A, LOW);
        relayWrite(ROLE_A, false);
      } else if (freezeOpen) {
        if (tempScreenValue <= milkHisterezisSetValue) {
          if (!confuseFasilaOn.state) {
            if (!freezeRoleOff.state) { freezeRoleOff.set = 1000; }
            relayWrite(ROLE_C, false);
            confuseFasilaOn.set = 1000;
            stateFasila = true;
          }
        } else if (tempScreenValue >= (milkHisterezisSetValue + tempHisterezis)) {
          activeRoleAndLed();
          stateFasila = false;
          confuseFasilaOn.state = false;
          stateAcceleration = false;
        }
      } else {
        closeRoleAndLed();
      }
    } else {
      closeRoleAndLed();
      stateAcceleration = true;
      confuseFasilaOn.state = false;
    }
  }

  if (tmrTick) {
    tmrTick = false;
    checkButtons();
    // --- 7-seg, ekran ve tüm orijinal zamanlayıcı blokları (kısaltılmış olmayan, orijinal davranış korunur) ---
    // Aşağıdaki bloklarda sadece ROLE_X yazımları relayWrite ile güncellendi

    if (systemOpen) {
      if (stateVersion_FULL) {
        _7segmentBlinkSendValue.set = 5;
        _7segmentBlinkSendState.set = 0;
        _DisplayA = 8;
        _DisplayB = 8;
        _DisplayC = 8;
        _7SegmentWrite(0, false, true, true, true);
      }
      if (stateVersionNo) {
        _7segmentBlinkSendValue.set = 5;
        _7segmentBlinkSendState.set = 0;
        _DisplayA = HARF_P;
        _DisplayB = 0;
        _DisplayC = 1;
        _7SegmentWrite(0, false, false, false, false);
      }
      if (stateVersion_) {
        _7segmentBlinkSendValue.set = 5;
        _7segmentBlinkSendState.set = 0;
        _DisplayA = HARF_;
        _DisplayB = HARF_;
        _DisplayC = HARF_;
        _7SegmentWrite(0, false, false, false, false);
      }
      if (stateTempShow) {
        _7segmentBlinkSendValue.set = 5;
        _7segmentBlinkSendState.set = 0;
        _7SegmentWrite(tempScreenValue, true, false, true, false);
      }
      if (stateTempHisterezis) {
        _7segmentBlinkSendState.set = 350;
        _7SegmentWrite(milkHisterezisSetValue, true, false, true, false);
        milkHisterezisSetValue = setSpeedUp(milkHisterezisSetValue);
        milkHisterezisSetValue = setSpeedDown(milkHisterezisSetValue);
      }
      if (statePArShow) {
        _7segmentBlinkSendValue.set = 5;
        _7segmentBlinkSendState.set = 0;
        _DisplayA = HARF_P;
        _DisplayB = HARF_A;
        _DisplayC = HARF_r;
        _7SegmentWrite(0, false, false, false, false);
      }
      if (stateModeShow) {
        _7segmentBlinkSendValue.set = 5;
        _7segmentBlinkSendState.set = 0;
        _DisplayA = HARF_P;
        _DisplayB = PAr_Value / 10;
        _DisplayC = PAr_Value % 10;
        _7SegmentWrite(0, false, true, false, false);
      }
      if (statePAR_0) {
        _7segmentBlinkSendState.set = 350;
        _7SegmentWrite(minFreezeSet, true, false, true, false);
        minFreezeSet = setSpeedUp(minFreezeSet);
        minFreezeSet = setSpeedDown(minFreezeSet);
      }
      if (statePAR_1) {
        _7segmentBlinkSendState.set = 350;
        _7SegmentWrite(tempHisterezis, true, false, true, false);
        tempHisterezis = setSpeedUp(tempHisterezis);
        tempHisterezis = setSpeedDown(tempHisterezis);
      }
      if (statePAR_2) {
        _7segmentBlinkSendState.set = 350;
        _7SegmentWrite(tempCalibration, true, false, true, false);
        tempCalibration = setSpeedUp(tempCalibration);
        tempCalibration = setSpeedDown(tempCalibration);
      }
      if (statePAR_3) {
        _7segmentBlinkSendState.set = 350;
        _7SegmentWrite(maxFreezeSet, true, false, true, false);
        maxFreezeSet = setSpeedUp(maxFreezeSet);
        maxFreezeSet = setSpeedDown(maxFreezeSet);
      }
      if (statePAR_4) {
        _7segmentBlinkSendState.set = 350;
        _7SegmentWrite(confuseOnTime, true, false, false, false);
      }
      if (statePAR_5) {
        _7segmentBlinkSendState.set = 350;
        _7SegmentWrite(confuseOffTime, true, false, false, false);
      }
      if (statePAR_6) {
        _7segmentBlinkSendState.set = 350;
        _7SegmentWrite(coverState, true, false, true, false);
      }
      if (statePAR_7) {
        _7segmentBlinkSendState.set = 350;
        _7SegmentWrite(coverAngleValue, true, false, true, false);
      }
      if (statePAR_8) {
        _7segmentBlinkSendState.set = 350;
        _7SegmentWrite(roleOnValue, true, false, true, false);
      }
      if (statePAR_9) {
        _7segmentBlinkSendState.set = 350;
        _7SegmentWrite(roleOffValue, true, false, true, false);
      }
      if (statePAR_10) {
        _7segmentBlinkSendState.set = 350;
        _7SegmentWrite(Rvalue, true, false, true, false);
      }

      //ble send Data section
      if(BLESendDataDelay.set!=0){
        if(BLESendDataDelay.count++>=BLESendDataDelay.set){
            BLESendDataDelay.count=0;
            sendCurrentStateAndSettings();
        }

      }
    }

    if (systemBlink.set != 0) {
      if (++systemBlink.count >= systemBlink.set) {
        systemBlink.count = 0;
        systemBlink.state = !systemBlink.state;
        if (stateFasila) {
          systemBlink.set = 500;
          LedMod(LED_A, systemBlink.state);
        }
        if (stateAcceleration && systemOpen) {
          systemBlink.set = 100;
          if (systemBlink.state) {
            LedMod(LED_A, HIGH);
            LedMod(LED_B, HIGH);
          } else {
            LedMod(LED_A, LOW);
            LedMod(LED_B, LOW);
          }
        }
      }
    }

    if (newTempCheck.set != 0) {
      if (++newTempCheck.count >= newTempCheck.set) {
        newTempCheck.count = 0;
        newTempCheck.state = !newTempCheck.state;

        if (T != _tempNTC) {
          if (T >= (_tempNTC + 10.0)) {
            _tempNTC = T;
            newTempDownSet.fcounter = T;
            nextTempValue = T;
          } else if (T >= (_tempNTC + 5.0)) {
            newTempUpSet.set = 50;
            tempPushData.set = 0;
            newTempCheck.set = 0;
            nextTempValue = T;
          } else if (T >= (_tempNTC + 0.5)) {
            newTempUpSet.set = 100; //KONTROL EDİLECEK
            tempPushData.set = 0;
            newTempCheck.set = 0;
            nextTempValue = T;
          }
          if (T <= (_tempNTC - 10.0)) {
            _tempNTC = T;
            newTempDownSet.fcounter = T;
            nextTempValue = T;
          } else if (T <= (_tempNTC - 5.0)) {
            newTempDownSet.set = 50;
            tempPushData.set = 0;
            newTempCheck.set = 0;
            nextTempValue = T;
          } else if (T < _tempNTC) {
            newTempDownSet.set = 100;
            tempPushData.set = 0;
            newTempCheck.set = 0;
            nextTempValue = T;
          }
        }
      }
    }

    if (newTempUpSet.set != 0) {
      if (++newTempUpSet.count >= newTempUpSet.set) {
        newTempUpSet.count = 0;
        newTempUpSet.state = !newTempUpSet.state;
        if (flag10) newTempDownSet.fcounter += 2.0;
        else newTempDownSet.fcounter += 0.1;
        if (newTempDownSet.fcounter <= nextTempValue) {
          tempScreenValue = newTempDownSet.fcounter;
        } else {
          flag10 = false;
          _tempNTC = T;
          tempPushData.set = 100;
          newTempCheck.set = 100;
          newTempUpSet.set = 0;
          newTempDownSet.fcounter = tempScreenValue;
        }
      }
    }

    if (newTempDownSet.set != 0) {
      if (++newTempDownSet.count >= newTempDownSet.set) {
        newTempDownSet.count = 0;
        newTempDownSet.state = !newTempDownSet.state;
        newTempDownSet.fcounter -= 0.1;
        if (newTempDownSet.fcounter >= nextTempValue) {
          tempScreenValue = newTempDownSet.fcounter;
        } else {
          _tempNTC = T;
          tempPushData.set = 100;
          newTempCheck.set = 100;
          newTempDownSet.set = 0;
          newTempDownSet.fcounter = tempScreenValue;
        }
      }
    }

    tempAndILT();

    if (tempPushData.set != 0) {
      if (++tempPushData.count >= tempPushData.set) {
        tempPushData.count = 0;
        tempPushData.state = !tempPushData.state;

        if ((tempNTC != tempLastNTC) || (tempCalibration != tempCalibrationLast)) {
          if (++tempCounter >= 10) {
            tempLastNTC = tempNTC;
            tempCounter = 0;
            R2 = R1 * (4095.0 / (float)tempLastNTC - 1.0);
            logR2 = log(R2);
            demo = (1.0 / (c1 + c2 * logR2 + c3 * logR2 * logR2 * logR2));
            demo = demo - 273.15;
            demo = (demo * 9.0) / 5.0 + 32.0;
            demo = (demo - 32) * 5 / 9;
            demo = (demo - 5.0);
            T = demo;
            T = T + tempCalibration;
            if (T < 0.0) T = 0.0;
            tempCalibrationLast = tempCalibration;

            if (!newTempFlag) {
              _tempNTC = T;
              newTempDownSet.fcounter = T;
              nextTempValue = T;
              tempScreenValue = T;
              newTempFlag = true;
            }

            // Hızlı BLE notify: delta 0.1 üstü ise 300ms throttle ile gönder
            // if (deviceConnected) {
            //   float delta = fabs(tempScreenValue - lastNotifiedTemp);
            //   uint32_t now = millis();
            //   if (delta >= 0.1f && (now - lastNotifyMs) >= NOTIFY_MIN_PERIOD_MS) {
            //     lastNotifiedTemp = tempScreenValue;
            //     lastNotifyMs = now;
            //     sendCurrentStateAndSettings();
            //   }
            // }
          }
        } else {
          tempCounter = 0;
        }
      }
    }

    if (newVersionNo.set != 0) {
      if (++newVersionNo.count >= newVersionNo.set) {
        newVersionNo.count = 0;
        newVersionNo.state = !newVersionNo.state;
        newVersionNo.counter++;
        if (newVersionNo.counter == 5) {
          stateVersion_FULL = false;
          stateVersionNo = true;
          stateVersion_ = false;
        } else if (newVersionNo.counter == 10) {
          stateVersion_FULL = false;
          stateVersionNo = false;
          stateVersion_ = true;
        } else if (newVersionNo.counter == 15) {
          _7segmentBlinkSendValue.set = 5;
          stateTempShow = true;
          stateVersion_ = false;
          stateVersion_FULL = false;
          stateVersionNo = false;
          newVersionNo.counter = 0;
          newVersionNo.set = 0;
        }
      }
    }

    if (btnConfuseTime.set != 0) {
      if (++btnConfuseTime.count >= btnConfuseTime.set) {
        btnConfuseTime.count = 0;
        btnConfuseTime.state = !btnConfuseTime.state;

        if (selectButton == TEMP_HISTEREZIS) {
          milkHisterezisSetValue += 0.1;
          speedSetUp.set = 4000;
          speedSetDown.set = 0;
          setValueTime.counter = 0;
        } else if (selectButton == PAR_ACTIVE || selectButton == PAR_0 || selectButton == PAR_1 || selectButton == PAR_2 || selectButton == PAR_3 || selectButton == PAR_4 || selectButton == PAR_5 || selectButton == PAR_6 || selectButton == PAR_7 || selectButton == PAR_8 || selectButton == PAR_9 || selectButton == PAR_10) {
          if (++PAr_Value > 10) PAr_Value = 0;
          funcModeState(PAr_Value);
          _PArCloseTime.counter = 0;
        } else if (selectButton == PAR_0_ACTIVE) {
          minFreezeSet += 0.1;
          speedSetUp.set = 4000;
          _PArCloseTime.counter = 0;
        } else if (selectButton == PAR_1_ACTIVE) {
          tempHisterezis += 0.1;
          speedSetUp.set = 4000;
          _PArCloseTime.counter = 0;
        } else if (selectButton == PAR_2_ACTIVE) {
          tempCalibration += 0.1;
          speedSetUp.set = 4000;
          _PArCloseTime.counter = 0;
        } else if (selectButton == PAR_3_ACTIVE) {
          maxFreezeSet += 0.1;
          speedSetUp.set = 4000;
          _PArCloseTime.counter = 0;
        } else if (selectButton == PAR_4_ACTIVE) {
          confuseOnTime += 0.1;
          _PArCloseTime.counter = 0;
        } else if (selectButton == PAR_5_ACTIVE) {
          confuseOffTime += 0.1;
          _PArCloseTime.counter = 0;
        } else if (selectButton == PAR_6_ACTIVE) {
          coverState = 0.1;
          systemAccelerationPassive = true;
          _PArCloseTime.counter = 0;
        } else if (selectButton == PAR_7_ACTIVE) {
          coverAngleValue += 0.1;
          if (coverAngleValue >= 0.3) coverAngleValue = 0.3;
          _PArCloseTime.counter = 0;
        } else if (selectButton == PAR_8_ACTIVE) {
          roleOnValue += 0.1;
          _PArCloseTime.counter = 0;
        } else if (selectButton == PAR_9_ACTIVE) {
          roleOffValue += 0.1;
          _PArCloseTime.counter = 0;
        } else if (selectButton == PAR_10_ACTIVE) {
          Rvalue += 0.1;
          if (Rvalue >= 0.2) Rvalue = 0.2;
          _PArCloseTime.counter = 0;
        }

        if (selectButton == SETTINGS) {
          if (btnConfuseTime.state && !btnFreezeTime.state) {
            confuseOpen = true;
            LedMod(LED_B, HIGH);
            relayWrite(ROLE_B, true);
          } else if (!btnFreezeTime.state) {
            confuseOpen = false;
            LedMod(LED_B, LOW);
            relayWrite(ROLE_B, false);
            btnFreezeTime.state = false;
          }
        }

        btnConfuseTime.set = 0;
      }
    }

    if (btnFreezeTime.set != 0) {
      if (++btnFreezeTime.count >= btnFreezeTime.set) {
        btnFreezeTime.count = 0;
        btnFreezeTime.state = !btnFreezeTime.state;

        if (selectButton == TEMP_HISTEREZIS) {
          milkHisterezisSetValue -= 0.1;
          if (milkHisterezisSetValue <= minFreezeSet) milkHisterezisSetValue = minFreezeSet;
          speedSetDown.set = 4000;
          speedSetUp.set = 0;
          setValueTime.counter = 0;
        } else if (selectButton == PAR) {
          stateTempHisterezis = false;
          stateTempShow = false;
          statePArShow = false;
          stateModeShow = true;
          selectButton = funcModeState(PAr_Value);
          _PArCloseTime.counter = 0;
        } else if (selectButton == PAR_0) {
          stateTempHisterezis = false;
          stateTempShow = false;
          statePArShow = false;
          stateModeShow = false;
          statePAR_0 = true;
          selectButton = PAR_0_ACTIVE;
          _PArCloseTime.counter = 0;
        } else if (selectButton == PAR_0_ACTIVE) {
          minFreezeSet -= 0.1;
          if (minFreezeSet <= 3.0) minFreezeSet = 3.0;
          speedSetDown.set = 4000;
          _PArCloseTime.counter = 0;
        } else if (selectButton == PAR_1) {
          stateTempHisterezis = false;
          stateTempShow = false;
          statePArShow = false;
          stateModeShow = false;
          statePAR_1 = true;
          selectButton = PAR_1_ACTIVE;
          _PArCloseTime.counter = 0;
        } else if (selectButton == PAR_1_ACTIVE) {
          tempHisterezis -= 0.1;
          if (tempHisterezis <= 0.0) tempHisterezis = 0.0;
          speedSetDown.set = 4000;
          _PArCloseTime.counter = 0;
        } else if (selectButton == PAR_2) {
          stateTempHisterezis = false;
          stateTempShow = false;
          statePArShow = false;
          stateModeShow = false;
          statePAR_2 = true;
          selectButton = PAR_2_ACTIVE;
          _PArCloseTime.counter = 0;
        } else if (selectButton == PAR_2_ACTIVE) {
          tempCalibration -= 0.1;
          if (tempCalibration <= 0.0) tempCalibration = 0.0;
          speedSetDown.set = 4000;
          _PArCloseTime.counter = 0;
        } else if (selectButton == PAR_3) {
          stateTempHisterezis = false;
          stateTempShow = false;
          statePArShow = false;
          stateModeShow = false;
          statePAR_3 = true;
          selectButton = PAR_3_ACTIVE;
          _PArCloseTime.counter = 0;
        } else if (selectButton == PAR_3_ACTIVE) {
          maxFreezeSet -= 0.1;
          if (maxFreezeSet <= 3.0) maxFreezeSet = 3.0;
          speedSetDown.set = 4000;
          _PArCloseTime.counter = 0;
        } else if (selectButton == PAR_4) {
          stateTempHisterezis = false;
          stateTempShow = false;
          statePArShow = false;
          stateModeShow = false;
          statePAR_4 = true;
          selectButton = PAR_4_ACTIVE;
          _PArCloseTime.counter = 0;
        } else if (selectButton == PAR_4_ACTIVE) {
          confuseOnTime -= 0.1;
          if (confuseOnTime <= 0.0) confuseOnTime = 0.0;
          _PArCloseTime.counter = 0;
        } else if (selectButton == PAR_5) {
          stateTempHisterezis = false;
          stateTempShow = false;
          statePArShow = false;
          stateModeShow = false;
          statePAR_5 = true;
          selectButton = PAR_5_ACTIVE;
          _PArCloseTime.counter = 0;
        } else if (selectButton == PAR_5_ACTIVE) {
          confuseOffTime -= 0.1;
          if (confuseOffTime <= 0.0) confuseOffTime = 0.0;
          _PArCloseTime.counter = 0;
        } else if (selectButton == PAR_6) {
          stateTempHisterezis = false;
          stateTempShow = false;
          statePArShow = false;
          stateModeShow = false;
          statePAR_6 = true;
          selectButton = PAR_6_ACTIVE;
          _PArCloseTime.counter = 0;
        } else if (selectButton == PAR_6_ACTIVE) {
          coverState = 0.0;
          systemAccelerationPassive = false;
          _PArCloseTime.counter = 0;
        } else if (selectButton == PAR_7) {
          stateTempHisterezis = false;
          stateTempShow = false;
          statePArShow = false;
          stateModeShow = false;
          statePAR_7 = true;
          selectButton = PAR_7_ACTIVE;
          _PArCloseTime.counter = 0;
        } else if (selectButton == PAR_7_ACTIVE) {
          coverAngleValue -= 0.1;
          if (coverAngleValue <= 0.0) coverAngleValue = 0.0;
          _PArCloseTime.counter = 0;
        } else if (selectButton == PAR_8) {
          stateTempHisterezis = false;
          stateTempShow = false;
          statePArShow = false;
          stateModeShow = false;
          statePAR_8 = true;
          selectButton = PAR_8_ACTIVE;
          _PArCloseTime.counter = 0;
        } else if (selectButton == PAR_8_ACTIVE) {
          roleOnValue -= 0.1;
          if (roleOnValue <= 0.0) roleOnValue = 0.1;
          _PArCloseTime.counter = 0;
        } else if (selectButton == PAR_9) {
          stateTempHisterezis = false;
          stateTempShow = false;
          statePArShow = false;
          stateModeShow = false;
          statePAR_9 = true;
          selectButton = PAR_9_ACTIVE;
          _PArCloseTime.counter = 0;
        } else if (selectButton == PAR_9_ACTIVE) {
          roleOffValue -= 0.1;
          if (roleOffValue <= 0.0) roleOffValue = 0.1;
          _PArCloseTime.counter = 0;
        } else if (selectButton == PAR_10) {
          stateTempHisterezis = false;
          stateTempShow = false;
          statePArShow = false;
          stateModeShow = false;
          statePAR_10 = true;
          selectButton = PAR_10_ACTIVE;
          _PArCloseTime.counter = 0;
        } else if (selectButton == PAR_10_ACTIVE) {
          Rvalue -= 0.1;
          if (Rvalue <= 0.0) Rvalue = 0.0;
          _PArCloseTime.counter = 0;
        }

        if (selectButton == SETTINGS) {
          if (btnFreezeTime.state) {
            if (tempScreenValue <= milkHisterezisSetValue) {
              freezeOpen = true;
              LedMod(LED_B, HIGH);
              relayWrite(ROLE_B, true);
            } else {
              freezeOpen = true;
              confuseOpen = false;
              activeRoleAndLed();
            }
            systemActive = 0.1;
            prefs.begin("settings", false);
            prefs.putFloat("systemActive", systemActive);
            prefs.end();
          } else {
            freezeOpen = false;
            stateFasila = false;
            systemActive = 0.0;
            prefs.begin("settings", false);
            prefs.putFloat("systemActive", systemActive);
            prefs.end();
            freezeRoleOff.set = 1000;
            LedMod(LED_B, LOW);
            relayWrite(ROLE_B, false);
            relayWrite(ROLE_C, false);
            confuseFasilaOn.set = 0;
            confuseFasilaOn.counter = 0;
            confuseFasilaOff.set = 0;
            confuseFasilaOff.counter = 0;
            confuseFasilaOn.state = false;
            btnConfuseTime.state = false;
          }
        }

        btnFreezeTime.set = 0;
      }
    }

    if (btnSettingsTime.set != 0) {
      if (++btnSettingsTime.count >= btnSettingsTime.set) {
        btnSettingsTime.count = 0;
        btnSettingsTime.state = !btnSettingsTime.state;

        if (btnSettingsTime.state) {
          if (selectButton == SETTINGS) {
            stateTempHisterezis = true;
            stateTempShow = false;
            statePArShow = false;
            stateModeShow = false;
            selectButton = TEMP_HISTEREZIS;
            _PArValueTime.set = 4000;
            setValueTime.set = 1000;
          } else if (selectButton == PAR) {
            btnPowerTime.state = true;
            stateTempHisterezis = false;
            stateTempShow = true;
            statePArShow = false;
            stateModeShow = false;
            selectButton = SETTINGS;
            btnSettingsTime.state = false;
            _PArCloseTime.set = 0;
            _PArCloseTime.counter = 0;
          } else if (selectButton == PAR_0_ACTIVE || selectButton == PAR_1_ACTIVE || selectButton == PAR_2_ACTIVE || selectButton == PAR_3_ACTIVE || selectButton == PAR_4_ACTIVE || selectButton == PAR_5_ACTIVE || selectButton == PAR_6_ACTIVE || selectButton == PAR_7_ACTIVE || selectButton == PAR_8_ACTIVE || selectButton == PAR_9_ACTIVE || selectButton == PAR_10_ACTIVE) {
            stateTempHisterezis = false;
            stateTempShow = false;
            statePArShow = false;
            stateModeShow = true;
            statePAR_0 = false;
            statePAR_1 = false;
            statePAR_2 = false;
            statePAR_3 = false;
            statePAR_4 = false;
            statePAR_5 = false;
            statePAR_6 = false;
            statePAR_7 = false;
            statePAR_8 = false;
            statePAR_9 = false;
            statePAR_10 = false;
            selectButton = funcModeState(PAr_Value);
            btnSettingsTime.state = false;
            eepromRead();
            _PArCloseTime.counter = 0;
          } else if (selectButton == PAR_ACTIVE || selectButton == PAR_0 || selectButton == PAR_1 || selectButton == PAR_2 || selectButton == PAR_3 || selectButton == PAR_4 || selectButton == PAR_5 || selectButton == PAR_6 || selectButton == PAR_7 || selectButton == PAR_8 || selectButton == PAR_9 || selectButton == PAR_10) {
            btnPowerTime.state = true;
            stateTempHisterezis = false;
            stateTempShow = true;
            statePArShow = false;
            stateModeShow = false;
            selectButton = SETTINGS;
            btnSettingsTime.state = false;
            _PArCloseTime.counter = 0;
            _PArCloseTime.set = 0;
          }

        } else {
          if (selectButton == TEMP_HISTEREZIS) {
            btnPowerTime.state = true;
            stateTempHisterezis = false;
            stateTempShow = true;
            statePArShow = false;
            stateModeShow = false;
            selectButton = SETTINGS;
            setValueTime.set = 0;
            setValueTime.counter = 0;
            prefs.begin("settings", true);
            milkHisterezisSetValue = prefs.getFloat("milkHisterezisSetValue", milkHisterezisSetValue);
            prefs.end();
          }
        }

        btnSettingsTime.set = 0;
      }
    }

    if (btnPowerTime.set != 0) {
      if (++btnPowerTime.count >= btnPowerTime.set) {
        btnPowerTime.count = 0;
        btnPowerTime.state = !btnPowerTime.state;

        if (selectButton == PAR_0_ACTIVE || selectButton == PAR_1_ACTIVE || selectButton == PAR_2_ACTIVE || selectButton == PAR_3_ACTIVE || selectButton == PAR_4_ACTIVE || selectButton == PAR_5_ACTIVE || selectButton == PAR_6_ACTIVE || selectButton == PAR_7_ACTIVE || selectButton == PAR_8_ACTIVE || selectButton == PAR_9_ACTIVE || selectButton == PAR_10_ACTIVE) {
          stateTempHisterezis = false;
          stateTempShow = false;
          statePArShow = false;
          stateModeShow = true;
          statePAR_0 = false;
          statePAR_1 = false;
          statePAR_2 = false;
          statePAR_3 = false;
          statePAR_4 = false;
          statePAR_5 = false;
          statePAR_6 = false;
          statePAR_7 = false;
          statePAR_8 = false;
          statePAR_9 = false;
          statePAR_10 = false;
          selectButton = funcModeState(PAr_Value);
          btnPowerTime.state = false;
          eepromWrite();
          _PArCloseTime.counter = 0;
        }

        if (selectButton == SETTINGS) {
          if (btnPowerTime.state) {
            systemOpen = true;
            tempRead.set = 1;
            _7segmentBlinkSendValue.set = 5;
            stateTempShow = true;
            btnSettingsTime.state = false;
            btnFreezeTime.state = false;
            btnConfuseTime.state = false;
          } else {
            systemOpen = false;
            freezeOpen = false;
            stateFasila = false;
            tempRead.set = 0;
            _7segmentBlinkSendValue.set = 0;
            _7segmentBlinkSendState.set = 0;
            stateTempHisterezis = false;
            stateTempShow = false;
            statePArShow = false;
            stateModeShow = false;
            digitalWrite(DISPLAY_A, HIGH);
            digitalWrite(DISPLAY_B, HIGH);
            digitalWrite(DISPLAY_C, HIGH);
            freezeRoleOff.set = 1000;
            LedMod(LED_B, LOW);
            relayWrite(ROLE_B, false);
            relayWrite(ROLE_C, false);
            confuseFasilaOn.set = 0;
            confuseFasilaOn.counter = 0;
            confuseFasilaOff.set = 0;
            confuseFasilaOff.counter = 0;
            confuseFasilaOn.state = false;
            btnConfuseTime.state = false;
          }
        }

        btnPowerTime.set = 0;
      }
    }

    if (setValueTime.set != 0) {
      if (++setValueTime.count >= setValueTime.set) {
        setValueTime.count = 0;
        setValueTime.state = !setValueTime.state;
        if (++setValueTime.counter >= 7) {
          stateTempHisterezis = false;
          stateTempShow = true;
          statePArShow = false;
          stateModeShow = false;
          selectButton = SETTINGS;
          btnSettingsTime.state = false;
          prefs.begin("settings", false);
          prefs.putFloat("milkHisterezisSetValue", milkHisterezisSetValue);
          prefs.end();
          setValueTime.counter = 0;
          setValueTime.set = 0;
        }
      }
    }

    if (confuseFasilaOn.set != 0) {
      if (++confuseFasilaOn.count >= confuseFasilaOn.set) {
        confuseFasilaOn.count = 0;
        confuseFasilaOn.state = true;
        LedMod(LED_B, HIGH);
        relayWrite(ROLE_B, true);
        if (++confuseFasilaOn.counter >= (confuseOnTime * 10) * 60) {
          confuseFasilaOn.counter = 0;
          confuseFasilaOn.set = 0;
          confuseFasilaOff.set = 1000;
        }
      }
    }

    if (confuseFasilaOff.set != 0) {
      if (++confuseFasilaOff.count >= confuseFasilaOff.set) {
        confuseFasilaOff.count = 0;
        LedMod(LED_B, LOW);
        relayWrite(ROLE_B, false);
        relayWrite(ROLE_A, false);
        relayWrite(ROLE_C, false);
        if (++confuseFasilaOff.counter >= (confuseOffTime * 10) * 60) {
          confuseFasilaOff.counter = 0;
          confuseFasilaOff.set = 0;
          confuseFasilaOn.set = 1000;
        }
      }
    }

    if (freezeRoleOn.set != 0) {
      if (++freezeRoleOn.count >= freezeRoleOn.set) {
        freezeRoleOn.count = 0;
        if (++freezeRoleOn.counter >= (roleOnValue * 10)) {
          freezeRoleOn.state = true;
          freezeRoleOff.state = false;
          LedMod(LED_A, HIGH);
          relayWrite(ROLE_A, true);
          freezeRoleOn.counter = 0;
          freezeRoleOn.set = 0;
        }
      }
    }

    if (freezeRoleOff.set != 0) {
      if (++freezeRoleOff.count >= freezeRoleOff.set) {
        freezeRoleOff.count = 0;
        if (++freezeRoleOff.counter >= (roleOffValue * 10)) {
          freezeRoleOff.state = true;
          freezeRoleOn.state = false;
          LedMod(LED_A, LOW);
          relayWrite(ROLE_A, false);
          freezeRoleOff.counter = 0;
          freezeRoleOff.set = 0;
        }
      }
    }

    if (systemAccelerationBlink.set != 0) {
      if (++systemAccelerationBlink.count >= systemAccelerationBlink.set) {
        systemAccelerationBlink.count = 0;
        systemAccelerationBlink.state = !systemAccelerationBlink.state;
        //Gyro_angle();
      }
    }

    if (_PArValueTime.set != 0) {
      if (++_PArValueTime.count >= _PArValueTime.set) {
        _PArValueTime.count = 0;
        _PArValueTime.state = true;
        stateTempHisterezis = false;
        stateTempShow = false;
        statePArShow = true;
        stateModeShow = false;
        selectButton = PAR;
        btnSettingsTime.state = false;
        PAr_Value = 0;
        _PArCloseTime.set = 1000;
        _PArValueTime.set = 0;
        setValueTime.counter = 0;
        setValueTime.set = 0;
      }
    }

    if (_PArCloseTime.set != 0) {
      if (++_PArCloseTime.count >= _PArCloseTime.set) {
        _PArCloseTime.count = 0;
        if (++_PArCloseTime.counter >= 30) {
          stateTempHisterezis = false;
          stateTempShow = true;
          statePArShow = false;
          stateModeShow = false;
          statePAR_0 = false;
          statePAR_1 = false;
          statePAR_2 = false;
          statePAR_3 = false;
          statePAR_4 = false;
          statePAR_5 = false;
          statePAR_6 = false;
          statePAR_7 = false;
          statePAR_8 = false;
          statePAR_9 = false;
          statePAR_10 = false;
          selectButton = SETTINGS;
          btnSettingsTime.state = false;
          _PArCloseTime.set = 0;
          _PArCloseTime.counter = 0;
        }
      }
    }

    if (_7segmentBlinkSendState.set != 0) {
      if (++_7segmentBlinkSendState.count >= _7segmentBlinkSendState.set) {
        _7segmentBlinkSendState.count = 0;
        _7segmentBlinkSendState.state = !_7segmentBlinkSendState.state;
        if (_7segmentBlinkSendState.state) {
          _7segmentBlinkSendValue.set = 5;
        } else {
          _7segmentBlinkSendValue.set = 0;
          digitalWrite(DISPLAY_A, HIGH);
          digitalWrite(DISPLAY_B, HIGH);
          digitalWrite(DISPLAY_C, HIGH);
        }
      }
    }
  }

  // Hızlı sıcaklık delta bildirimi dışı durumlar
  //bl_process();
}