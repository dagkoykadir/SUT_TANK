#include <math.h>
#include <EEPROM.h> 
#include <Preferences.h> // ESP32’ye özel kalıcı bellek çözümü
#include <Wire.h> //I2C için standart kütüphane
#include <SPI.h>
#include <SparkFun_ADXL345.h>         // SparkFun ADXL345 Library
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

static String _macToString(const uint8_t m[6]) {
  char buf[18];
  sprintf(buf, "%02X:%02X:%02X:%02X:%02X:%02X", m[0], m[1], m[2], m[3], m[4], m[5]);
  return String(buf);
}

static String _getBleMacString() {
  uint8_t mac[6] = {0};
  esp_read_mac(mac, ESP_MAC_BT);
  return _macToString(mac);
}

static String _macSuffix(const String& mac) {
  int p = mac.lastIndexOf(':');
  String last2 = mac.substring(p + 1);
  int p2 = mac.lastIndexOf(':', p - 1);
  String last3 = mac.substring(p2 + 1, p);
  return last3 + last2; // "EEFF"
}

/* ADC_PIN - BUTTON */
#define ADC_PIN   39 //SENSOR_VN

#define BTN_S6    6
#define BTN_S7    7
#define BTN_S8    8
#define BTN_S9    9

/* ROLEs*//* eski kod*/
/*
#define ROLE_A    6
#define ROLE_B    7
#define ROLE_C    A1
*/
// yeni kod


#define ROLE_A    2 //RLY2 
#define ROLE_B    22 //RLY3 
#define ROLE_C    21 //RLY1 

/* LEDs */
#define LED_A   27
#define LED_B   0

/* 7Segment Pins */
#define  PIN_A          14 //D5 
#define  PIN_B          17 //D3 
#define  PIN_C          13 //C4 
#define  PIN_D          4 //B1 
#define  PIN_E          25 //B0 
#define  PIN_F          12 //D4 
#define  PIN_G          26 //C3 
#define  PIN_H          32 //C5 

#define  DISPLAY_A      5 //D0 
#define  DISPLAY_B      15 //D1 
#define  DISPLAY_C      16 //D2 

#define  NOKTA_OPEN     10
#define  NOKTA_CLOSE    11
#define  HARF_P         12
#define  HARF_L         13
#define  HARF_C         14
#define  DISPLAY_CLOSE  15
#define  HARF_r         16
#define  HARF_TIRE      17
#define  HARF_A         18
#define  HARF_          19

/* NTC */
#define NTC_PIN         36 //SENSOR_VP

/* EEPROM ADDR */
#define MODE_0          0
#define MODE_1          2
#define MODE_2          4
#define MODE_3          6
#define MODE_4          8
#define MODE_5          10
#define MODE_6          12
#define MODE_7          14
#define WORK_LOOP       16
#define SET_VALUE       18
#define MODE_8          20
#define MODE_9          22
#define SYSTEM_ACT      24
#define R_VALUE_ADD      26

/* DEBUG */
#define DEBUG  0


#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
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

tmrDelay  btnConfuseTime;
tmrDelay  btnFreezeTime;
tmrDelay  btnSettingsTime;
tmrDelay  btnPowerTime;

tmrDelay  _7segmentBlinkSendValue;
tmrDelay  _7segmentBlinkSendState;

tmrDelay  setValueTime;
tmrDelay  _PArValueTime;
tmrDelay  _PArCloseTime;

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

//auto timer = timer_create_default();
//Timer<1, micros> timer;
ADXL345 adxl = ADXL345(33);

tmrDelay newTempCheck;
tmrDelay newTempUpSet;
tmrDelay newTempDownSet;
tmrDelay newTempRead;

tmrDelay newVersionNo;
tmrDelay newVersion_;

/*systemAcceleration blink*/
tmrDelay systemAccelerationBlink;

/* Timer Variable  */
bool tmrTick = false;
uint16_t globalCounter = 0;

/* _7Segment Variable */
uint8_t _DisplayA = 0;
uint8_t _DisplayB = 0;
uint8_t _DisplayC = 0;
bool noktaOne = false;
bool noktaTwo = false;
bool noktaThree = false;

char _send[99];

/* Mode Function Variable
   Freeze = donmak
   confuse = karistirmak
*/
uint8_t PAr_Value = 0;

float minFreezeSet = 3.0;           //P.00
float tempHisterezis = 0.5;         //P.01
float milkHisterezisSetValue = minFreezeSet; //Sut histerezis Set Degeri
float tempCalibration = 3.0;        //P.02
float maxFreezeSet = 38.0;          //P.03
float confuseOnTime = 0.3;          //P.04
float confuseOffTime = 1.0;         //P.05
float coverState = 0.1;             //P.06
float coverAngleValue = 0.3;        //P.07
float roleOnValue = 0.2;            //P.08
float roleOffValue = 0.4;           //P.09
float systemActive = 0.0;           // Sistem Durumunu Kontrol eder.
float Rvalue =0.0;

float eepromMinFreezeSet = 0.0;
float eepromTempHisterezis = 0.0;
float eepromTempCalibration = 0.0;
float eepromMaxFreezeSet = 0.0;
float eepromConfuseOnTime = 0.0;
float eepromConfuseOffTime = 0.0;
float eepromCoverState = 0.0;
float eepromCoverAngleValue = 0.0;
float eepromSetValue = 0.0;
float eepromRoleOnValue = 0.0;
float eepromRoleOffValue = 0.0;
float eepromSystemActive = 0.0;
float eepromRvalue =0.0;
/* Settings button Variable */
bool flagHisterezis = false;

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

float avgTemp = 0.0;

bool isMixerOn = false;
bool isDeviceOn = false;
bool isCoolerOn = false;

bool systemOpen = false;
bool confuseOpen = false;
bool freezeOpen = false;
bool systemAcceleration = false;
bool systemAccelerationPassive = false;
bool stateFasila = false;
bool stateAcceleration = false;
bool stateGyro_Value = false;

uint8_t counterGyro = 0;
uint16_t cont8 = 0;

float R1 = 10000;//6000 di de
float logR2, R2, T,demo;
float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;

float newTempValue = 0.0;
float tempScreenValue = 0.0;
float nextTempValue = 0.0;
float _tempNTC = 0.0;

bool newTempFlag = false;

bool stateVersion_ = false;
bool stateVersionNo = false;
bool stateVersion_FULL = false;

BLECharacteristic *pTxCharacteristic;
bool deviceConnected = false;

void sendCurrentStateAndSettings();
void eepromWrite();

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

    // --- AYARLAR (params) ---
    if (doc.containsKey("params")) {
      JsonObject p = doc["params"];

      auto upd = [&](const char* key, float &var){
        if (p.containsKey(key)) { var = p[key].as<float>(); settingsChanged = true; }
      };

      upd("minFreezeSet",           minFreezeSet);
      upd("tempHisterezis",         tempHisterezis);
      upd("milkHisterezisSetValue", milkHisterezisSetValue);
      upd("tempCalibration",        tempCalibration);
      upd("maxFreezeSet",           maxFreezeSet);
      upd("confuseOnTime",          confuseOnTime);
      upd("confuseOffTime",         confuseOffTime);
      upd("coverState",             coverState);
      upd("coverAngleValue",        coverAngleValue);
      upd("roleOnValue",            roleOnValue);
      upd("roleOffValue",           roleOffValue);

      if (settingsChanged) {
        eepromWrite();  // Tek noktadan kalıcıya yaz
        Serial.println("Ayarlar güncellendi ve kaydedildi.");
      }
    }

    // --- ANLIK KONTROLLER ---
    // Mixer -> ROLE_B, Power -> ROLE_A, Cooler -> ROLE_C
    if (doc.containsKey("mixer")) {
      isMixerOn = (doc["mixer"].as<int>() == 1);
      digitalWrite(ROLE_B, isMixerOn ? HIGH : LOW);
      prefs.begin("settings", false);
      prefs.putBool("mixerState", isMixerOn);
      prefs.end();
    }

    if (doc.containsKey("power")) {
      isDeviceOn = (doc["power"].as<int>() == 1);
      digitalWrite(ROLE_A, isDeviceOn ? HIGH : LOW);
      prefs.begin("settings", false);
      prefs.putBool("powerState", isDeviceOn);
      prefs.end();
    }

    if (doc.containsKey("cooler")) {
      isCoolerOn = (doc["cooler"].as<int>() == 1);
      digitalWrite(ROLE_C, isCoolerOn ? HIGH : LOW);
      prefs.begin("settings", false);
      prefs.putBool("coolerState", isCoolerOn);
      prefs.end();
    }
  }
};



class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("Cihaz bağlandı.");
  }
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("Cihaz bağlantısı kesildi, tekrar yayın yapılıyor...");
    BLEDevice::startAdvertising();
  }
};


//omer addde ,
uint8_t coverAngleEx=0;
float tempCalibrationLast=3.0;
uint32_t count;
uint32_t count1;

void bl_init(void){
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
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
  pTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
                                             CHARACTERISTIC_UUID_RX,
                                             BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
                                           );
  pRxCharacteristic->setCallbacks(new MyCharacteristicCallbacks());

  pService->start();
  
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pServer->getAdvertising()->start();

}

void bl_process(void){
  static uint32_t _lastNotify = 0;
  const uint32_t PERIOD_MS = 1000;
  if (deviceConnected) {
    uint32_t now = millis();
    if (now - _lastNotify >= PERIOD_MS) {
      _lastNotify = now;
      sendCurrentStateAndSettings();
    }
  }
}

void sendCurrentStateAndSettings() {
    if (!deviceConnected) return;

    StaticJsonDocument<512> doc;
    
    // Anlık durumları ekle
    doc["temp"] = String(_tempNTC, 1); // Rastgele sıcaklık
    doc["mixer"] = isMixerOn ? 1 : 0;
    doc["power"] = isDeviceOn ? 1 : 0;
    doc["cooler"] = isCoolerOn ? 1 : 0;

    // Ayar parametrelerini JSON'a ekle
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
    /*
    paramsObj["minFreezeSet"] = params.minFreezeSet;
    paramsObj["tempHisterezis"] = params.tempHisterezis;
    paramsObj["milkHisterezisSetValue"] = params.milkHisterezisSetValue;
    paramsObj["tempCalibration"] = params.tempCalibration;
    paramsObj["maxFreezeSet"] = params.maxFreezeSet;
    paramsObj["confuseOnTime"] = params.confuseOnTime;
    paramsObj["confuseOffTime"] = params.confuseOffTime;
    paramsObj["coverState"] = params.coverState;
    paramsObj["coverAngleValue"] = params.coverAngleValue;
    paramsObj["roleOnValue"] = params.roleOnValue;
    paramsObj["roleOffValue"] = params.roleOffValue;
    */
    String output;
    serializeJson(doc, output);
    output += "\n";


    pTxCharacteristic->setValue(output.c_str());
    pTxCharacteristic->notify();

    Serial.print("Birleşik Veri Paketi Gönderildi: ");
    Serial.print(output);
}

void IRAM_ATTR IntTimer() {

  if(count++ >=  10){
    //checkButtons();
    count=0;
   // count1++; Serial.println(count1);
  }

  tmrTick = true;
//
  
}


void activeRoleAndLed ( void ) {
  /* SOGUTUCU */
  //LedMod(LED_A,HIGH);
  //digitalWrite(ROLE_A,HIGH);
  if ( !freezeRoleOn.state ) {
    freezeRoleOn.set = 1000;
  }

  /* KARISITIRIC */
  LedMod(LED_B, HIGH);
  digitalWrite(ROLE_B, HIGH);
  /* KOMPRESSOR*/
  digitalWrite(ROLE_C, HIGH);

  confuseFasilaOn.set = 0;
  confuseFasilaOn.counter = 0;
  confuseFasilaOff.set = 0;
  confuseFasilaOff.counter = 0;
}

void closeRoleAndLed ( void ) {
  /* SOGUTUCU */
  //LedMod(LED_A,LOW);
  //digitalWrite(ROLE_A,LOW);
  if ( !freezeRoleOff.state ) {
    freezeRoleOff.set = 1000;
  }
  /* KARISTIRICI */
  //yeni eklendi
  if(!stateAcceleration) LedMod(LED_B, LOW);
 
  digitalWrite(ROLE_B, LOW);
  /* KOMPRESSOR*/
  digitalWrite(ROLE_C, LOW);

  confuseFasilaOn.set = 0;
  confuseFasilaOn.counter = 0;
  confuseFasilaOff.set = 0;
  confuseFasilaOff.counter = 0;
}

void Gyro_init(void) {

  adxl.powerOn(); // Power on the ADXL345
  adxl.setRangeSetting(2);// Accepted values are 2g, 4g, 8g or 16g
  adxl.setSpiBit(0); //SPI = 1, I2C = 0
  adxl.setActivityXYZ(1, 0, 0);       // Set to activate movement detection in the axes "adxl.setActivityXYZ(X, Y, Z);" (1 == ON, 0 == OFF)
  adxl.setActivityThreshold(75);      // 62.5mg per increment   // Set activity   // Inactivity thresholds (0-255)

  adxl.setInactivityXYZ(1, 0, 0);     // Set to detect inactivity in all the axes "adxl.setInactivityXYZ(X, Y, Z);" (1 == ON, 0 == OFF)
  adxl.setInactivityThreshold(75);    // 62.5mg per increment   // Set inactivity // Inactivity thresholds (0-255)
  adxl.setTimeInactivity(10);         // How many seconds of no activity is inactive?

  adxl.setTapDetectionOnXYZ(1, 0, 0); // Detect taps in the directions turned ON "adxl.setTapDetectionOnX(X, Y, Z);" (1 == ON, 0 == OFF)

//  Set values for what is considered a TAP and what is a DOUBLE TAP (0-255)
  adxl.setTapThreshold(50);           // 62.5 mg per increment
  adxl.setTapDuration(15);            // 625 μs per increment
  adxl.setDoubleTapLatency(80);       // 1.25 ms per increment
  adxl.setDoubleTapWindow(200);       // 1.25 ms per increment

 //  Set values for what is considered FREE FALL (0-255)
  adxl.setFreeFallThreshold(7);       // (5 - 9) recommended - 62.5mg per increment
  adxl.setFreeFallDuration(30);       // (20 - 70) recommended - 5ms per increment

}

void LedMod(uint8_t pin , bool mod) {
  digitalWrite(pin, mod ? HIGH : LOW);
}

void Gyro_angle(void) {
  int x, y, z;
  adxl.readAccel(&x, &y, &z);         // Read the accelerometer values and store them in variables declared above x,y,z

 /*Serial.print("x  :");
    Serial.print(x);
    Serial.print("   Y  :");
    Serial.print(y);
    Serial.print("   z  :");
    Serial.println(z);
    Serial.print("  cover  :");
    Serial.println(coverAngleEx);*/
  if(coverAngleValue == 0.1){
     coverAngleEx=30;//20
  }else if(coverAngleValue == 0.2){
     coverAngleEx=50;//50
  }else if (coverAngleValue==0.3){
    coverAngleEx=80;
  }else {
    coverAngleEx=0;
  }
  if ( confuseOpen || freezeOpen ) {
    if ( z <= -(80+coverAngleEx) ) {
      if ( ++counterGyro >= 10) {
        counterGyro = 0;
        Serial.println("z <= -(80+coverAngleEx)");
        stateGyro_Value = true;
      }
    } else if ( z >= -(50+coverAngleEx) && z <= -(10+coverAngleEx) ) {
      if ( ++counterGyro >= 5 ) {
        counterGyro = 0;
        Serial.println("z >= -(50+coverAngleEx) && z <= -(10+coverAngleEx)");
        stateGyro_Value = false;
      }
    } else if ( z >= -(9+coverAngleEx) ) {
      if ( ++counterGyro >= 5 ) {
        counterGyro = 0;
        //Serial.println("z >= -(9+coverAngleEx)");
        stateGyro_Value = false;
      }
    }
  } else {
    counterGyro = 0;
    stateGyro_Value = false;
  }

}
uint8_t btnstate=0;
uint8_t btnstateLast=0;
/*
#define BTN_S6    2
#define BTN_S7    3
#define BTN_S8    1
#define BTN_S9    4
*/

  uint8_t detectedButton = 0;
  uint16_t lastStableValue = 0; //ani geçişlerde diğer butonlar algılanmaz
  uint32_t lastChangeTime = 0;
  uint16_t lastRawValue = 0; //Önceki anaolog 
uint16_t StableValuecont =0;
uint8_t analogToDigitalRead(void) {


  uint16_t lastStableValue = analogRead(ADC_PIN);
  //Serial.print("lastStableValue =  ")  ;
  //Serial.println(lastStableValue);
  if (lastStableValue < 2000 && lastStableValue >= 1900) {//lastStableValue < 1915 && lastStableValue >= 1895
    if(cont8++>20){
      cont8=0;
       detectedButton = 8;
    }
    
    
  } else if (lastStableValue < 2800 && lastStableValue >= 2700) {//lastStableValue < 2680 && lastStableValue >= 2660
        if(cont8++>20){
      cont8=0;
       detectedButton = 6;
    }
    
  } else if (lastStableValue < 3400 && lastStableValue >= 3300) {//lastStableValue < 3285 && lastStableValue >= 3260
        if(cont8++>20){
      cont8=0;
       detectedButton = 7;
    }
   
  } else if (lastStableValue < 4096 && lastStableValue >= 4070) {
        if(cont8++>20){
      cont8=0;
       detectedButton = 9;
    }
    
  }
  else{
          cont8=0;
       detectedButton = 0;
  }
  return detectedButton;

}


void eepromWrite(void) {
  prefs.begin("settings", false); // "settings" bir namespace gibi çalışır

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

  prefs.end(); // Kaydı kapat
}


void eepromRead(void) {
  prefs.begin("settings", true); // Sadece okuma modu

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


uint8_t funcModeState ( uint8_t value ) {

  switch (value) {
    case 0:
      selectButton = PAR_0;
      break;
    case 1:
      selectButton = PAR_1;
      break;
    case 2:
      selectButton = PAR_2;
      break;
    case 3:
      selectButton = PAR_3;
      break;
    case 4:
      selectButton = PAR_4;
      break;
    case 5:
      selectButton = PAR_5;
      break;
    case 6:
      selectButton = PAR_6;
      break;
    case 7:
      selectButton = PAR_7;
      break;
    case 8:
      selectButton = PAR_8;
      break;
    case 9:
      selectButton = PAR_9;
      break;
    case 10:
      selectButton = PAR_10;
      break;      
  }
  return selectButton;
}

float setSpeedUp ( float value ) {

  if ( speedSetUp.set != 0 ) {
    if ( ++speedSetUp.count >= speedSetUp.set ) {
      speedSetUp.count = 0;
      speedSetUp.state = !speedSetUp.state;

      value += 0.1;

      _PArCloseTime.counter = 0;
      setValueTime.counter = 0;

#if DEBUG
      Serial.println("Girdi UP");
      Serial.println(value);
#endif

      speedSetUp.set = 150;
    }
  }

  return value;
}

float setSpeedDown ( float value ) {

  if ( speedSetDown.set != 0 ) {
    if ( ++speedSetDown.count >= speedSetDown.set ) {
      speedSetDown.count = 0;
      speedSetDown.state = !speedSetDown.state;

      if ( stateTempHisterezis ) {

        value -= 0.1;

        if ( value <= minFreezeSet ) {
          value = minFreezeSet;
        }

      }
      else if ( statePAR_1 ) {
        value -= 0.1;

        if ( value <= 0.0 ) {
          value = 0.0;
        }
      }
      else {

        value -= 0.1;

        if ( value <= 3.0 ) {
          value = 3.0;
        }
      }


      _PArCloseTime.counter = 0;
      setValueTime.counter = 0;

#if DEBUG
      Serial.println("Girdi DOWN");
      Serial.println(value);
#endif

      speedSetDown.set = 150;
    }
  }

  return value;
}

void _7SegmentWrite ( float value, bool sayi, bool noktaA, bool noktaB, bool noktaC ) {

  uint8_t number = 0;

  if ( _7segmentBlinkSendValue.set != 0 ) {
    if ( ++_7segmentBlinkSendValue.count >= _7segmentBlinkSendValue.set ) {
      _7segmentBlinkSendValue.count = 0;
      _7segmentBlinkSendValue.state = !_7segmentBlinkSendValue.state;
      _7segmentBlinkSendValue.counter += 1;

      if ( sayi ) {

        sprintf(_send, "%d", (int)(value * 10));
        number = strlen(_send);

        if ( number == 3 ) {
          _DisplayA = _send[0] - '0';
          _DisplayB = _send[1] - '0';
          _DisplayC = _send[2] - '0';
        } else if ( number == 2 ) {
          _DisplayB = _send[0] - '0';
          _DisplayC = _send[1] - '0';
        } else if ( number == 1 ) {
          _DisplayB = 0;
          _DisplayC = _send[0] - '0';
        }
      }


      if ( _7segmentBlinkSendValue.counter == 1 ) {
        if ( number == 3 || !sayi ) {
          digitalWrite(DISPLAY_A, HIGH);
          digitalWrite(DISPLAY_B, HIGH);
          digitalWrite(DISPLAY_C, LOW);
          _7SegmentDisplay(_DisplayA);

          if ( noktaA ) {
            _7SegmentDisplay(NOKTA_OPEN);
          } else {
            _7SegmentDisplay(NOKTA_CLOSE);
          }

        } else {
          digitalWrite(DISPLAY_C, HIGH);
        }
      } else if ( _7segmentBlinkSendValue.counter == 2 ) {

        digitalWrite(DISPLAY_A, HIGH);
        digitalWrite(DISPLAY_B, LOW);
        digitalWrite(DISPLAY_C, HIGH);
        _7SegmentDisplay(_DisplayB);

        if ( noktaB ) {
          _7SegmentDisplay(NOKTA_OPEN);
        } else {
          _7SegmentDisplay(NOKTA_CLOSE);
        }

      } else if ( _7segmentBlinkSendValue.counter == 3 ) {
        digitalWrite(DISPLAY_A, LOW);
        digitalWrite(DISPLAY_B, HIGH);
        digitalWrite(DISPLAY_C, HIGH);
        _7SegmentDisplay(_DisplayC);

        if ( noktaC ) {
          _7SegmentDisplay(NOKTA_OPEN);
        } else {
          _7SegmentDisplay(NOKTA_CLOSE);
        }

        _7segmentBlinkSendValue.counter = 0;
      }
    }
  }
}

void checkButtons ( void ) {
  if ( systemOpen ) {

    if ( BTN_S6 == analogToDigitalRead() ) {
      if ( !btn_S6.state ) {
        btnConfuseTime.set = 20;
        btn_S6.state = true;
      }
    } else {
      if ( btn_S6.state ) {
        speedSetUp.set = 0;
        btn_S6.state = false;
      }
    }

    if ( BTN_S7 == analogToDigitalRead() ) {
      if ( !btn_S7.state ) {
        btnSettingsTime.set = 20;
        btn_S7.state = true;
      }
    } else {
      if ( btn_S7.state ) {
        _PArValueTime.set = 0;
        btn_S7.state = false;
      }
    }

    if ( BTN_S9 == analogToDigitalRead() ) {
      if ( !btn_S9.state ) {
        btnFreezeTime.set = 20;
        btn_S9.state = true;
      }
    } else {
      if ( btn_S9.state ) {
        speedSetDown.set = 0;
        btn_S9.state = false;
      }
    }

  }

  if ( BTN_S8 == analogToDigitalRead() ) {
    if ( !btn_S8.state ) {
      btnPowerTime.set = 20;
      btn_S8.state = true;
    }
  } else {
    if ( btn_S8.state ) {
      btn_S8.state = false;
    }
  }


  if ( systemAccelerationPassive ) {
    if ( btn_cyro.value != stateGyro_Value ) {
      if ( btn_cyro.value = stateGyro_Value ) {
        if ( !btn_cyro.state ) {

          systemAcceleration = true;
          btn_cyro.state = true;

        }
      } else {
        if ( btn_cyro.state ) {

          systemAcceleration = false;
          btn_cyro.state = false;
        }
      }
    }
  }

}

void _7SegmentDisplay ( uint8_t value ) {
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
    case NOKTA_OPEN:
      digitalWrite(PIN_H, LOW);
      break;
    case NOKTA_CLOSE:
      digitalWrite(PIN_H, HIGH);
      break;
    case HARF_P:  //P
      digitalWrite(PIN_A, LOW);
      digitalWrite(PIN_B, LOW);
      digitalWrite(PIN_C, HIGH);
      digitalWrite(PIN_D, HIGH);
      digitalWrite(PIN_E, LOW);
      digitalWrite(PIN_F, LOW);
      digitalWrite(PIN_G, LOW);

      break;
    case HARF_L:  //L
      digitalWrite(PIN_A, LOW);
      digitalWrite(PIN_B, LOW);
      digitalWrite(PIN_C, LOW);
      digitalWrite(PIN_D, HIGH);
      digitalWrite(PIN_E, HIGH);
      digitalWrite(PIN_F, HIGH);
      digitalWrite(PIN_G, LOW);

      break;
    case HARF_C:  //C
      digitalWrite(PIN_A, LOW);
      digitalWrite(PIN_B, HIGH);
      digitalWrite(PIN_C, HIGH);
      digitalWrite(PIN_D, LOW);
      digitalWrite(PIN_E, LOW);
      digitalWrite(PIN_F, LOW);
      digitalWrite(PIN_G, HIGH);

      break;
    case DISPLAY_CLOSE:  //Display Kapali
      digitalWrite(PIN_A, HIGH);
      digitalWrite(PIN_B, HIGH);
      digitalWrite(PIN_C, HIGH);
      digitalWrite(PIN_D, HIGH);
      digitalWrite(PIN_E, HIGH);
      digitalWrite(PIN_F, HIGH);
      digitalWrite(PIN_G, HIGH); 

      break;
    case HARF_r:  //r
      digitalWrite(PIN_A, HIGH);
      digitalWrite(PIN_B, HIGH);
      digitalWrite(PIN_C, HIGH);
      digitalWrite(PIN_D, HIGH);
      digitalWrite(PIN_E, LOW);
      digitalWrite(PIN_F, HIGH);
      digitalWrite(PIN_G, LOW);

      break;
    case HARF_TIRE:  //-
      digitalWrite(PIN_A, HIGH);
      digitalWrite(PIN_B, HIGH);
      digitalWrite(PIN_C, HIGH);
      digitalWrite(PIN_D, HIGH);
      digitalWrite(PIN_E, HIGH);
      digitalWrite(PIN_F, HIGH);
      digitalWrite(PIN_G, LOW);

      break;
    case HARF_A:  //A
      digitalWrite(PIN_A, LOW);
      digitalWrite(PIN_B, LOW);
      digitalWrite(PIN_C, LOW);
      digitalWrite(PIN_D, HIGH);
      digitalWrite(PIN_E, LOW);
      digitalWrite(PIN_F, LOW);
      digitalWrite(PIN_G, LOW);

      break;
    case HARF_:  //-
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

void tempAndILT ( void ) {

  if ( NTC_Counter <= 7)//12 omer added
    NTC_Toplam += (4095-analogRead(NTC_PIN));

  if ( NTC_Counter == 7 )
    NTC_Ortalama = NTC_Toplam / 7;

  if ( NTC_Counter > 7)
    NTC_Ortalama = ( NTC_Ortalama * 6 + (4095-analogRead(NTC_PIN)) ) / 7;

  if ( NTC_Counter < 8)
    NTC_Counter++;

  tempNTC = NTC_Ortalama;
  // tempNTC= 1023-analogRead(NTC_PIN);
  //Serial.println(tempNTC);

}

void setup() {

  Serial.begin(9600);

  /* 7Segment */
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

  /* LED A = SOGUTUCU */
  pinMode(LED_A, OUTPUT);
  digitalWrite(LED_A, LOW);

  /* LED B = KARISTIRICI */
  pinMode(LED_B, OUTPUT);
  digitalWrite(LED_B, LOW);

  /* Role */
  pinMode(ROLE_A, OUTPUT);
  digitalWrite(ROLE_A, LOW);

  pinMode(ROLE_B, OUTPUT);
  digitalWrite(ROLE_B, LOW);

  pinMode(ROLE_C, OUTPUT);
  digitalWrite(ROLE_C, LOW);

  myTimer.attach_ms(1, IntTimer); 

  /* Cyro */
  //Gyro_init();

  systemBlink.set = 100;
  tempPushData.set = 100;

  systemAccelerationBlink.set = 250;
// 
  //analogReadResolution(10); 
  //eepromWrite();    delay(100);
  eepromRead();
  //omer added
  tempCalibrationLast=tempCalibration;

  if ( (systemActive * 10) == 0 ) {
    freezeOpen = false;
  } else {
    freezeOpen = true;
    btnFreezeTime.state = true;
  }

  if ( (coverState * 10) == 0 ) { //Kapat Durumunu Kontrol Ediyoruz. ( Active/Passive )
    systemAccelerationPassive = false;
  } else {
    systemAccelerationPassive = true;
  }

  systemOpen = true;
  btnPowerTime.state = true;

  stateVersion_FULL = true;
  newVersionNo.set = 250;

  /* Temp Read Start */
  newTempCheck.set = 100;
  tempRead.set = 1;

  // Load persisted output states
  prefs.begin("settings", true);
  isMixerOn  = prefs.getBool("mixerState", false);
  isDeviceOn = prefs.getBool("powerState", false);
  isCoolerOn = prefs.getBool("coolerState", false);
  prefs.end();
  pinMode(ROLE_A, OUTPUT);
  pinMode(ROLE_B, OUTPUT);
  pinMode(ROLE_C, OUTPUT);
  digitalWrite(ROLE_B, isMixerOn ? HIGH : LOW);
  digitalWrite(ROLE_A, isDeviceOn ? HIGH : LOW);
  digitalWrite(ROLE_C, isCoolerOn ? HIGH : LOW);

  
  // Load persisted parameters
  prefs.begin("settings", true);
  minFreezeSet = prefs.getFloat("minFreezeSet", minFreezeSet);
  tempHisterezis = prefs.getFloat("tempHisterezis", tempHisterezis);
  tempCalibration = prefs.getFloat("tempCalibration", tempCalibration);
  maxFreezeSet = prefs.getFloat("maxFreezeSet", maxFreezeSet);
  confuseOnTime = prefs.getFloat("confuseOnTime", confuseOnTime);
  confuseOffTime = prefs.getFloat("confuseOffTime", confuseOffTime);
  coverState = prefs.getFloat("coverState", coverState);
  coverAngleValue = prefs.getFloat("coverAngleValue", coverAngleValue);
  roleOnValue = prefs.getFloat("roleOnValue", roleOnValue);
  roleOffValue = prefs.getFloat("roleOffValue", roleOffValue);
  milkHisterezisSetValue = prefs.getFloat("milkHisterezisSetValue", milkHisterezisSetValue);
  Rvalue = prefs.getFloat("Rvalue", Rvalue);
  prefs.end();
  bl_init();
}

bool flag10=false;
void loop() {



  /* Board buttons Control Function *///2.45 us
  checkButtons();
  if ( systemOpen ) {

    if ( !systemAcceleration ) {
      stateAcceleration = false;

      if ( confuseOpen ) {

        LedMod(LED_B, HIGH);
        digitalWrite(ROLE_B, HIGH);
        LedMod(LED_A, LOW);
        digitalWrite(ROLE_A, LOW);
      } else if ( freezeOpen ) {

        if ( tempScreenValue <= milkHisterezisSetValue ) {
          if ( !confuseFasilaOn.state ) {

            /* SOGUTUCU */
            if ( !freezeRoleOff.state ) {
              freezeRoleOff.set = 1000;
            }

            /* KOMPRESSOR */
            digitalWrite(ROLE_C, LOW);

            /* KARISTIRICI */
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

  if ( tmrTick ) {
    tmrTick = false;

    if ( systemOpen ) {

      if ( stateVersion_FULL ) {
        _7segmentBlinkSendValue.set = 5;
        _7segmentBlinkSendState.set = 0;

        _DisplayA = 8;
        _DisplayB = 8;
        _DisplayC = 8;
        _7SegmentWrite(0, false, true, true, true);
      }

      if ( stateVersionNo ) {
        _7segmentBlinkSendValue.set = 5;
        _7segmentBlinkSendState.set = 0;

        _DisplayA = HARF_P;
        _DisplayB = 0;
        _DisplayC = 1;
        _7SegmentWrite(0, false, false, false, false);
      }

      if ( stateVersion_ ) {
        _7segmentBlinkSendValue.set = 5;
        _7segmentBlinkSendState.set = 0;

        _DisplayA = HARF_;
        _DisplayB = HARF_;
        _DisplayC = HARF_;
        _7SegmentWrite(0, false, false, false, false);
      }

      /* Sicaklik Gosterme */
      if ( stateTempShow ) {
        _7segmentBlinkSendValue.set = 5;
        _7segmentBlinkSendState.set = 0;
        _7SegmentWrite(tempScreenValue, true, false, true, false);
      }

      /* Sogutucu Sicaklik Ayari */
      if ( stateTempHisterezis ) {
        _7segmentBlinkSendState.set = 350;
        _7SegmentWrite(milkHisterezisSetValue, true, false, true, false);

        milkHisterezisSetValue = setSpeedUp(milkHisterezisSetValue);
        milkHisterezisSetValue = setSpeedDown(milkHisterezisSetValue);
      }

      /* Mod Gecis Bildirme */
      if ( statePArShow ) {
        _7segmentBlinkSendValue.set = 5;
        _7segmentBlinkSendState.set = 0;

        _DisplayA = HARF_P;
        _DisplayB = HARF_A;
        _DisplayC = HARF_r;
        _7SegmentWrite(0, false, false, false, false);
      }

      /* Modlar Arasi Gecis Gosterge */
      if ( stateModeShow ) {
        _7segmentBlinkSendValue.set = 5;
        _7segmentBlinkSendState.set = 0;

        _DisplayA = HARF_P;
        _DisplayB = PAr_Value/10;
        _DisplayC = PAr_Value%10;
        _7SegmentWrite(0, false, true, false, false);
      }

      /* Sogutma Ayarlama Ekrani */
      if ( statePAR_0 ) {
        _7segmentBlinkSendState.set = 350;
        _7SegmentWrite(minFreezeSet, true, false, true, false);

        minFreezeSet = setSpeedUp(minFreezeSet);
        minFreezeSet = setSpeedDown(minFreezeSet);
      }

      /* Sicaklik Set Degeri Ayarlama Ekrani */
      if ( statePAR_1 ) {
        _7segmentBlinkSendState.set = 350;
        _7SegmentWrite(tempHisterezis, true, false, true, false);

        tempHisterezis = setSpeedUp(tempHisterezis);
        tempHisterezis = setSpeedDown(tempHisterezis);
      }

      /* Sicaklik Kalibre Etme Ekrani*/
      if ( statePAR_2 ) {
        _7segmentBlinkSendState.set = 350;
        _7SegmentWrite(tempCalibration, true, false, true, false);

        tempCalibration = setSpeedUp(tempCalibration);
        tempCalibration = setSpeedDown(tempCalibration);
      }

      /* MaxSogutma Set Degeri Ayar Ekrani */
      if ( statePAR_3 ) {
        _7segmentBlinkSendState.set = 350;
        _7SegmentWrite(maxFreezeSet, true, false, true, false);

        maxFreezeSet = setSpeedUp(maxFreezeSet);
        maxFreezeSet = setSpeedDown(maxFreezeSet);
      }

      /* Karistirici Calisma Suresi Ayarlama Ekrani */
      if ( statePAR_4 ) {
        _7segmentBlinkSendState.set = 350;
        _7SegmentWrite(confuseOnTime, true, false, false, false);
      }

      /* Karistirici Kapali Kalma Suresi Ayarlama Ekrani */
      if ( statePAR_5 ) {
        _7segmentBlinkSendState.set = 350;
        _7SegmentWrite(confuseOffTime, true, false, false, false);
      }

      /* Kapak Aktif/Pasif Ayar Ekrani */
      if ( statePAR_6 ) {
        _7segmentBlinkSendState.set = 350;
        _7SegmentWrite(coverState, true, false, true, false);
      }

      /* Kapak Aci Ayar Ekrani */
      if ( statePAR_7 ) {
        _7segmentBlinkSendState.set = 350;
        _7SegmentWrite(coverAngleValue, true, false, true, false);
      }

      /* Role Acma Ayar Ekrani */
      if ( statePAR_8 ) {
        _7segmentBlinkSendState.set = 350;
        _7SegmentWrite(roleOnValue, true, false, true, false);
      }

      /* Role Kapama Ayar Ekrani */
      if ( statePAR_9 ) {
        _7segmentBlinkSendState.set = 350;
        _7SegmentWrite(roleOffValue, true, false, true, false);
      }

      if ( statePAR_10 ) {
        _7segmentBlinkSendState.set = 350;
        _7SegmentWrite(Rvalue, true, false, true, false);
      }      

    }

    if ( systemBlink.set != 0 ) {
      if (++systemBlink.count >= systemBlink.set) {
        systemBlink.count = 0;
        systemBlink.state = !systemBlink.state;

        /* Sogutma islemi baslatildi ve sicaklik degeri hedef sicaklik degerine
            geldiginde "Fasila" karistirici dongusunu baslatir.
            Bu dongu PAR_4 ve PAR_5 Ekranlarindeki Ayarlanan sureye gore Calisir.
            Bu calisma Olayinin calistigini bildiren Durum "stateFasila"
        */
        if ( stateFasila ) {
          systemBlink.set=500;
          if ( systemBlink.state ) {
            LedMod(LED_A, HIGH);
          } else {
            LedMod(LED_A, LOW);
          }
        }

        /* Sistem Calisir durumda iken Kapak Acildiginde "stateAcceleration" Degeri true
           yapilir ve sistemin calismasi durdurulur.
        */
        if (stateAcceleration && systemOpen) {
          systemBlink.set=100;
          if ( systemBlink.state ) {
            LedMod(LED_A, HIGH);
            LedMod(LED_B, HIGH);
          } else {
            LedMod(LED_A, LOW);
            LedMod(LED_B, LOW);
          }
        }
      }
    }

    /* Sicaklik Durumu Kontrol Ediliyor. Anlik Sicaklik Degisimleri Stabil hale getirilik Ekrana yollaniyor. */
    if ( newTempCheck.set != 0 ) { //100ms
      if (++newTempCheck.count >= newTempCheck.set) {
        newTempCheck.count = 0;
        newTempCheck.state = !newTempCheck.state;


        if ( T != _tempNTC ) {

         if ( T >= (_tempNTC + 10.0) ) {

            
            /*newTempUpSet.set = 100;//omer added
            tempPushData.set = 0;
            newTempCheck.set = 0;
            flag10=true;
            _tempNTC = T;
            nextTempValue = T;*/
            _tempNTC = T;
            newTempDownSet.fcounter = T;
            nextTempValue = T;
            

          } else if ( T >= (_tempNTC + 5.0) ) {

            newTempUpSet.set = 50;//omer added
            tempPushData.set = 0;
            newTempCheck.set = 0;

            nextTempValue = T;

          } else if ( T >= (_tempNTC + 0.5) ) {

            newTempUpSet.set = 400;
            tempPushData.set = 0;
            newTempCheck.set = 0;

            nextTempValue = T;

          } if ( T <= (_tempNTC - 10.0) ) {

            _tempNTC = T;
            newTempDownSet.fcounter = T;
            nextTempValue = T;

          } else if ( T <= (_tempNTC - 5.0) ) {

            newTempDownSet.set = 50;//omer added
            tempPushData.set = 0;
            newTempCheck.set = 0;

            nextTempValue = T;

          } else if ( T <  _tempNTC  ) {

            newTempDownSet.set = 400;
            tempPushData.set = 0;
            newTempCheck.set = 0;

            nextTempValue = T;

          }
        }
      }
    }

    /* Sicaklik Degerini "newTempCheck.set" yapisindan elde edilen sicaklik degerine
       yavas yavas yükseltmeyi sagliyor.
    */
    if ( newTempUpSet.set != 0 ) {
      if (++newTempUpSet.count >= newTempUpSet.set) {
        newTempUpSet.count = 0;
        newTempUpSet.state = !newTempUpSet.state;

        //omer asdded
        if(flag10){
          newTempDownSet.fcounter += 2.0;
        }else{
          newTempDownSet.fcounter += 0.1;
        }

        if ( newTempDownSet.fcounter <= nextTempValue ) {
          tempScreenValue = newTempDownSet.fcounter;
        } else {
          flag10=false;
          _tempNTC = T;

          tempPushData.set = 100;
          newTempCheck.set = 100;

          newTempUpSet.set = 0;
          newTempDownSet.fcounter = tempScreenValue;
        }
      }
    }

    /* Sicaklik Degerini "newTempCheck.set" yapisindan elde edilen sicaklik degerine
       yavas yavas düsürmeyi sagliyor.
    */
    if ( newTempDownSet.set != 0 ) {
      if (++newTempDownSet.count >= newTempDownSet.set) {
        newTempDownSet.count = 0;
        newTempDownSet.state = !newTempDownSet.state;

        newTempDownSet.fcounter -= 0.1;

        if ( newTempDownSet.fcounter >= nextTempValue ) {
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

    /* Analog Okuma Yapıyor *///omer added
//    if ( tempRead.set != 0 ) {
//      if (++tempRead.count >= tempRead.set) {
//        tempRead.count = 0;
//        tempRead.state = !tempRead.state;

        tempAndILT();
//      }
//    }

    /* Anlik Sicaklik Okuyoruz */
    if ( tempPushData.set != 0 ) {
      if (++tempPushData.count >= tempPushData.set) {
        tempPushData.count = 0;
        tempPushData.state = !tempPushData.state;

       if ( (tempNTC != tempLastNTC) ||(tempCalibration !=tempCalibrationLast)) {

          if ( ++tempCounter >= 10 ) {

            tempLastNTC = tempNTC;
            tempCounter = 0;

            R2 = R1 * (4095.0 / (float)tempLastNTC - 1.0);
            logR2 = log(R2);
            demo= (1.0 / (c1 + c2 * logR2 + c3 * logR2 * logR2 * logR2));
            demo = demo - 273.15;
           demo = (demo * 9.0) / 5.0 + 32.0;
            demo = (demo - 32) * 5 / 9;
            demo = (demo - 5.0); 
            
            if(Rvalue!=0&&0){
              if(Rvalue==0.1){//5Kohm
                
                if(demo<=40){
                  T=demo-17.5;
                }
                if(40<=demo && demo<=50){
                  T=demo-19.5;
                }
                if(50<=demo){
                  T=demo-20.5;
                }
              }else if (Rvalue ==0.2){//3Kohm

               if(25<=demo && demo<35){
                  T=demo-23.5;
               }
               else if(30<=demo && demo<35){
                  T=demo-24;
                }
                else if(35<=demo && demo<40){
                  T=demo-25;
                }else if(40<=demo && demo<45){ 
                  T=demo-26;
                }else if(45<=demo && demo<50){
                  T=demo-28;
                }else if(50<=demo && demo<55){
                  T=demo-29;
                }else if(55<=demo && demo<60){
                  T=demo-31;
                }else if(60<=demo && demo<65){
                  T=demo-33;
                }else if(65<=demo && demo<70){
                  T=demo-34;
                }else if(70<=demo && demo<75){
                  T=demo-35;
                }else if(75<=demo && demo<80){
                  T=demo-35.5;
                }else if(80<=demo && demo<85){
                  T=demo-37;
                }else if(85<=demo && demo<90){
                  T=demo-39;
                }else if(90<=demo && demo<95){
                  T=demo-39.5;
                }else if(95<=demo && demo<100){
                  T=demo-41;
                }else if(90<=demo && demo<100){
                  T=demo-43;
                }else{
                  //T=T-60;
                }   T=T-3;                             
              }
            }else{
              T= demo;
            }
            //T=T-3;
            T=T+ tempCalibration ;
            if(T<0.0)T=0.0;//omer added
            tempCalibrationLast=tempCalibration;
            

            
            if ( !newTempFlag ) {
              _tempNTC = T;
              newTempDownSet.fcounter = T;
              nextTempValue = T;
              tempScreenValue = T;

              newTempFlag = true;
            }
          }
        } else {
          tempCounter = 0;
        }
      }
    }

    /* Cihazin Version Bilgisini Ekrana Yazdiriyoruz. Cihaz ilk Acilista */
    if ( newVersionNo.set != 0 ) {
      if (++newVersionNo.count >= newVersionNo.set) {
        newVersionNo.count = 0;
        newVersionNo.state = !newVersionNo.state;

        newVersionNo.counter++;

        if ( newVersionNo.counter == 5 ) {
          stateVersion_FULL = false;
          stateVersionNo = true;
          stateVersion_ = false;

        } else if ( newVersionNo.counter == 10 ) {
          stateVersion_FULL = false;
          stateVersionNo = false;
          stateVersion_ = true;

        } else if ( newVersionNo.counter == 15 ) {

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

    /* "Ayarlar" butonuna basildiktan sonra "Karistirici" butonuna basilirsa "btnConfuseTime.set" degeri calismaya baslar
       Burada "selectButton" degerine gore islemler yapilir. Up
    */
    if ( btnConfuseTime.set != 0 ) {
      if ( ++btnConfuseTime.count >= btnConfuseTime.set ) {
        btnConfuseTime.count = 0;
        btnConfuseTime.state = !btnConfuseTime.state;

        if ( selectButton == TEMP_HISTEREZIS ) {

          milkHisterezisSetValue += 0.1; //milkHisterezisSetValue
          speedSetUp.set = 4000;
          speedSetDown.set = 0;

#if DEBUG
          Serial.print("TEMP_HISTEREZIS UP :");
          Serial.println(selectButton);
#endif

          setValueTime.counter = 0;
        }
        else if ( selectButton == PAR_ACTIVE || selectButton == PAR_0 ||
                  selectButton == PAR_1 || selectButton == PAR_2 ||
                  selectButton == PAR_3 || selectButton == PAR_4 ||
                  selectButton == PAR_5 || selectButton == PAR_6 ||
                  selectButton == PAR_7 || selectButton == PAR_8 ||
                  selectButton == PAR_9 || selectButton == PAR_10) {

          if ( ++PAr_Value > 10 ) {
            PAr_Value = 0;
          }

          funcModeState(PAr_Value);

#if DEBUG
          Serial.print("MODE UP :");
          Serial.println(selectButton);
#endif

          _PArCloseTime.counter = 0;
        }
        else if ( selectButton == PAR_0_ACTIVE ) {                         //PAR 0 UP

          minFreezeSet += 0.1;
          speedSetUp.set = 4000;

#if DEBUG
          Serial.print("PAR 0 UP :");
          Serial.println(selectButton);
#endif

          _PArCloseTime.counter = 0;
        }
        else if ( selectButton == PAR_1_ACTIVE ) {                         //PAR 1 UP

          tempHisterezis += 0.1;
          speedSetUp.set = 4000;

#if DEBUG
          Serial.print("PAR 1 UP :");
          Serial.println(selectButton);
#endif

          _PArCloseTime.counter = 0;
        }
        else if ( selectButton == PAR_2_ACTIVE ) {                         //PAR 2 UP

          tempCalibration += 0.1;
          speedSetUp.set = 4000;

#if DEBUG
          Serial.print("PAR 2 UP :");
          Serial.println(selectButton);
#endif

          _PArCloseTime.counter = 0;
        }
        else if ( selectButton == PAR_3_ACTIVE ) {                         //PAR 3 UP

          maxFreezeSet += 0.1;
          speedSetUp.set = 4000;

#if DEBUG
          Serial.print("PAR 3 UP :");
          Serial.println(selectButton);
#endif

          _PArCloseTime.counter = 0;
        }
        else if ( selectButton == PAR_4_ACTIVE ) {                         //PAR 4 UP

          confuseOnTime += 0.1;
          //speedSetUp.set = 4000;

#if DEBUG
          Serial.print("PAR 4 UP :");
          Serial.println(selectButton);
#endif

          _PArCloseTime.counter = 0;
        }
        else if ( selectButton == PAR_5_ACTIVE ) {                         //PAR 5 UP

          confuseOffTime += 0.1;
          //speedSetUp.set = 4000;

#if DEBUG
          Serial.print("PAR 5 UP :");
          Serial.println(selectButton);
#endif

          _PArCloseTime.counter = 0;
        }
        else if ( selectButton == PAR_6_ACTIVE ) {                         //PAR 6 UP

          coverState = 0.1;
          systemAccelerationPassive = true;

#if DEBUG
          Serial.print("PAR 6 UP :");
          Serial.println(selectButton);
#endif

          _PArCloseTime.counter = 0;
        }
        else if ( selectButton == PAR_7_ACTIVE ) {                         //PAR 7 UP

          coverAngleValue += 0.1;
          if ( coverAngleValue >= 0.3 ) {
            coverAngleValue = 0.3;
          }

#if DEBUG
          Serial.print("PAR 7 UP :");
          Serial.println(selectButton);
#endif

          _PArCloseTime.counter = 0;
        }
        else if ( selectButton == PAR_8_ACTIVE ) {                         //PAR 8 UP

          roleOnValue += 0.1;

#if DEBUG
          Serial.print("PAR 8 UP :");
          Serial.println(selectButton);
#endif

          _PArCloseTime.counter = 0;
        }
        else if ( selectButton == PAR_9_ACTIVE ) {                         //PAR 9 UP

          roleOffValue += 0.1;

#if DEBUG
          Serial.print("PAR 9 UP :");
          Serial.println(selectButton);
#endif

          _PArCloseTime.counter = 0;
        }
        else if ( selectButton == PAR_10_ACTIVE ) {                         //PAR 10 UP

          Rvalue += 0.1;
          if(Rvalue>=0.2){
            Rvalue=0.2;
          }

#if DEBUG
          Serial.print("PAR 10 UP :");
          Serial.println(selectButton);
#endif

          _PArCloseTime.counter = 0;
        }        

        if ( selectButton == SETTINGS ) {
          if ( btnConfuseTime.state && !btnFreezeTime.state ) {

            confuseOpen = true;
            LedMod(LED_B, HIGH);
            digitalWrite(ROLE_B, HIGH);

          } else if (!btnFreezeTime.state) {

            confuseOpen = false;
            LedMod(LED_B, LOW);
            digitalWrite(ROLE_B, LOW);

            btnFreezeTime.state = false;
          }
        }

        btnConfuseTime.set = 0;
      }
    }

    /* "Ayarlar" butonuna basildiktan sonra "Sogutucu" butonuna basilirsa "btnFreezeTime.set" degeri calismaya baslar
       Burada "selectButton" degerine gore islemler yapilir. Down
    */
    if ( btnFreezeTime.set != 0 ) {
      if ( ++btnFreezeTime.count >= btnFreezeTime.set ) {
        btnFreezeTime.count = 0;
        btnFreezeTime.state = !btnFreezeTime.state;

        if ( selectButton == TEMP_HISTEREZIS ) {

          milkHisterezisSetValue -= 0.1;

          if ( milkHisterezisSetValue <= minFreezeSet ) {
            milkHisterezisSetValue = minFreezeSet;
          }

          speedSetDown.set = 4000;
          speedSetUp.set = 0;

#if DEBUG
          Serial.print("TEMP_HISTEREZIS DOWN :");
          Serial.println(selectButton);
#endif

          setValueTime.counter = 0;
        }
        else if ( selectButton == PAR ) {

          stateTempHisterezis = false;
          stateTempShow = false;
          statePArShow = false;
          stateModeShow = true;

          selectButton = funcModeState(PAr_Value); // PAR_ACTIVE;PAr_Value omer changed

#if DEBUG
          Serial.print("PAR_ACTIVE :");
          Serial.println(selectButton);
#endif

          _PArCloseTime.counter = 0;
        }
        else if ( selectButton == PAR_0 ) {  //PAR 0 SELECT|| selectButton == PAR_ACTIVE

          stateTempHisterezis = false;
          stateTempShow = false;
          statePArShow = false;
          stateModeShow = false;
          statePAR_0 = true;

          selectButton = PAR_0_ACTIVE;
#if DEBUG
          Serial.print("PAR_0_ACTIVE :");
          Serial.println(selectButton);
#endif

          _PArCloseTime.counter = 0;
        }
        else if ( selectButton == PAR_0_ACTIVE ) {                         //PAR 0 DOWN

          minFreezeSet -= 0.1;

          if ( minFreezeSet <= 3.0 ) {
            minFreezeSet = 3.0;
          }

          speedSetDown.set = 4000;

#if DEBUG
          Serial.print("PAR 0 DOWN :");
          Serial.println(selectButton);
#endif

          _PArCloseTime.counter = 0;
        }
        else if ( selectButton == PAR_1 ) {                                //PAR 1 SELECT

          stateTempHisterezis = false;
          stateTempShow = false;
          statePArShow = false;
          stateModeShow = false;
          statePAR_1 = true;

          selectButton = PAR_1_ACTIVE;
#if DEBUG
          Serial.print("PAR_1_ACTIVE :");
          Serial.println(selectButton);
#endif

          _PArCloseTime.counter = 0;
        }
        else if ( selectButton == PAR_1_ACTIVE ) {                         //PAR 1 DOWN

          tempHisterezis -= 0.1;

          if ( tempHisterezis <= 0.0 ) {
            tempHisterezis = 0.0;
          }

          speedSetDown.set = 4000;

#if DEBUG
          Serial.print("PAR 1 DOWN :");
          Serial.println(selectButton);
#endif

          _PArCloseTime.counter = 0;
        }
        else if ( selectButton == PAR_2 ) {                                //PAR 2 SELECT

          stateTempHisterezis = false;
          stateTempShow = false;
          statePArShow = false;
          stateModeShow = false;
          statePAR_2 = true;

          selectButton = PAR_2_ACTIVE;
#if DEBUG
          Serial.print("PAR_2_ACTIVE :");
          Serial.println(selectButton);
#endif

          _PArCloseTime.counter = 0;
        }
        else if ( selectButton == PAR_2_ACTIVE ) {                         //PAR 2 DOWN

          tempCalibration -= 0.1;

          if ( tempCalibration <= 0.0 ) {
            tempCalibration = 0.0;
          }

          speedSetDown.set = 4000;

#if DEBUG
          Serial.print("PAR 2 DOWN :");
          Serial.println(selectButton);
#endif

          _PArCloseTime.counter = 0;
        }
        else if ( selectButton == PAR_3 ) {                                //PAR 3 SELECT

          stateTempHisterezis = false;
          stateTempShow = false;
          statePArShow = false;
          stateModeShow = false;
          statePAR_3 = true;

          selectButton = PAR_3_ACTIVE;
#if DEBUG
          Serial.print("PAR_3_ACTIVE :");
          Serial.println(selectButton);
#endif

          _PArCloseTime.counter = 0;
        }
        else if ( selectButton == PAR_3_ACTIVE ) {                         //PAR 3 DOWN

          maxFreezeSet -= 0.1;

          if ( maxFreezeSet <= 3.0 ) {
            maxFreezeSet = 3.0;
          }

          speedSetDown.set = 4000;

#if DEBUG
          Serial.print("PAR 3 DOWN :");
          Serial.println(selectButton);
#endif

          _PArCloseTime.counter = 0;
        }
        else if ( selectButton == PAR_4 ) {                                //PAR 4 SELECT

          stateTempHisterezis = false;
          stateTempShow = false;
          statePArShow = false;
          stateModeShow = false;
          statePAR_4 = true;

          selectButton = PAR_4_ACTIVE;
#if DEBUG
          Serial.print("PAR_4_ACTIVE :");
          Serial.println(selectButton);
#endif

          _PArCloseTime.counter = 0;
        }
        else if ( selectButton == PAR_4_ACTIVE ) {                         //PAR 4 DOWN

          confuseOnTime -= 0.1;

          if ( confuseOnTime <= 0.0 ) {
            confuseOnTime = 0.0;
          }

          //speedSetDown.set = 4000;

#if DEBUG
          Serial.print("PAR 4 DOWN :");
          Serial.println(selectButton);
#endif

          _PArCloseTime.counter = 0;
        }
        else if ( selectButton == PAR_5 ) {                                //PAR 5 SELECT

          stateTempHisterezis = false;
          stateTempShow = false;
          statePArShow = false;
          stateModeShow = false;
          statePAR_5 = true;

          selectButton = PAR_5_ACTIVE;
#if DEBUG
          Serial.print("PAR_5_ACTIVE :");
          Serial.println(selectButton);
#endif

          _PArCloseTime.counter = 0;
        }
        else if ( selectButton == PAR_5_ACTIVE ) {                         //PAR 5 DOWN

          confuseOffTime -= 0.1;

          if ( confuseOffTime <= 0.0 ) {
            confuseOffTime = 0.0;
          }

          //speedSetDown.set = 4000;

#if DEBUG
          Serial.print("PAR 5 DOWN :");
          Serial.println(selectButton);
#endif

          _PArCloseTime.counter = 0;
        }
        else if ( selectButton == PAR_6 ) {                                //PAR 6 SELECT

          stateTempHisterezis = false;
          stateTempShow = false;
          statePArShow = false;
          stateModeShow = false;
          statePAR_6 = true;

          selectButton = PAR_6_ACTIVE;
#if DEBUG
          Serial.print("PAR_6_ACTIVE :");
          Serial.println(selectButton);
#endif

          _PArCloseTime.counter = 0;
        }
        else if ( selectButton == PAR_6_ACTIVE ) {                         //PAR 6 DOWN

          coverState = 0.0;
          systemAccelerationPassive = false;

#if DEBUG
          Serial.print("PAR 6 DOWN :");
          Serial.println(selectButton);
#endif

          _PArCloseTime.counter = 0;
        }
        else if ( selectButton == PAR_7 ) {                                //PAR 7 SELECT

          stateTempHisterezis = false;
          stateTempShow = false;
          statePArShow = false;
          stateModeShow = false;
          statePAR_7 = true;

          selectButton = PAR_7_ACTIVE;
#if DEBUG
          Serial.print("PAR_7_ACTIVE :");
          Serial.println(selectButton);
#endif

          _PArCloseTime.counter = 0;
        }
        else if ( selectButton == PAR_7_ACTIVE ) {                         //PAR 7 DOWN

          coverAngleValue -= 0.1;

          if ( coverAngleValue <= 0.0 ) {
            coverAngleValue = 0.0;
          }

#if DEBUG
          Serial.print("PAR 7 DOWN :");
          Serial.println(selectButton);
#endif

          _PArCloseTime.counter = 0;
        }
        else if ( selectButton == PAR_8 ) {                                //PAR 8 SELECT

          stateTempHisterezis = false;
          stateTempShow = false;
          statePArShow = false;
          stateModeShow = false;
          statePAR_8 = true;

          selectButton = PAR_8_ACTIVE;
#if DEBUG
          Serial.print("PAR_8_ACTIVE :");
          Serial.println(selectButton);
#endif

          _PArCloseTime.counter = 0;
        }
        else if ( selectButton == PAR_8_ACTIVE ) {                         //PAR 8 DOWN

          roleOnValue -= 0.1;

          if ( roleOnValue <= 0.0 ) {
            roleOnValue = 0.1;
          }

#if DEBUG
          Serial.print("PAR 8 DOWN :");
          Serial.println(selectButton);
#endif

          _PArCloseTime.counter = 0;
        }
        else if ( selectButton == PAR_9 ) {                                //PAR 9 SELECT

          stateTempHisterezis = false;
          stateTempShow = false;
          statePArShow = false;
          stateModeShow = false;
          statePAR_9 = true;

          selectButton = PAR_9_ACTIVE;
#if DEBUG
          Serial.print("PAR_9_ACTIVE :");
          Serial.println(selectButton);
#endif

          _PArCloseTime.counter = 0;
        }
        else if ( selectButton == PAR_9_ACTIVE ) {                         //PAR 9 DOWN

          roleOffValue -= 0.1;

          if ( roleOffValue <= 0.0 ) {
            roleOffValue = 0.1;
          }

#if DEBUG
          Serial.print("PAR 9 DOWN :");
          Serial.println(selectButton);
#endif

          _PArCloseTime.counter = 0;
        }
        else if ( selectButton == PAR_10 ) {                                //PAR 10 SELECT

          stateTempHisterezis = false;
          stateTempShow = false;
          statePArShow = false;
          stateModeShow = false;
          statePAR_10 = true;

          selectButton = PAR_10_ACTIVE;
#if DEBUG
          Serial.print("PAR_10_ACTIVE :");
          Serial.println(selectButton);
#endif

          _PArCloseTime.counter = 0;
        }
        else if ( selectButton == PAR_10_ACTIVE ) {                         //PAR 10 DOWN

          Rvalue -= 0.1;

          if ( Rvalue <= 0.0 ) {
            Rvalue = 0.0;
          }

#if DEBUG
          Serial.print("PAR 10 DOWN :");
          Serial.println(selectButton);
#endif

          _PArCloseTime.counter = 0;
        }

        if ( selectButton == SETTINGS ) {
          if ( btnFreezeTime.state ) {

            /* Yeni */
            if (tempScreenValue <= milkHisterezisSetValue) {
              freezeOpen = true;

              /* KARISITIRIC */
              LedMod(LED_B, HIGH);
              digitalWrite(ROLE_B, HIGH);

            } else {
              freezeOpen = true;
              confuseOpen = false;
              activeRoleAndLed();
            }



            systemActive = 0.1;
            /* EEPROM Write System State */
            //EEPROM.write(SYSTEM_ACT,systemActive*10);
            prefs.begin("settings", false);
            prefs.putFloat("systemActive", systemActive);
            prefs.end();




          } else {
            freezeOpen = false;
            stateFasila = false;

            systemActive = 0.0;
            /* EEPROM Write System State */
            //EEPROM.write(SYSTEM_ACT,systemActive*10);
            prefs.begin("settings", false);
            prefs.putFloat("systemActive", systemActive);
            prefs.end();

            /* SOGUTUCU */
            //LedMod(LED_A,LOW);
            //digitalWrite(ROLE_A,LOW);
            freezeRoleOff.set = 1000;

            /* KARISIRICI */
            LedMod(LED_B, LOW);
            digitalWrite(ROLE_B, LOW);
            /* KOMPRESSOR */
            digitalWrite(ROLE_C, LOW);

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

    /* "Ayarlar" Butonuna basildiginde iki farklı mod calisir.
      1. MODE : Bir kere Basildiginde "selectButton" Degeirne "TEMP_HISTEREZIS" degeri atanir
      ve Cihazin Sicaklik Ayari Yapilir.
      2. MODE : Uzun basilir ve Cihaz Parametre Ayarlama Ekrani "PAR" Ekran Moduna Gecer.
    */
    if ( btnSettingsTime.set != 0 ) {
      if ( ++btnSettingsTime.count >= btnSettingsTime.set ) {
        btnSettingsTime.count = 0;
        btnSettingsTime.state = !btnSettingsTime.state;

        if ( btnSettingsTime.state ) {
          if ( selectButton == SETTINGS ) {

            stateTempHisterezis = true;
            stateTempShow = false;
            statePArShow = false;
            stateModeShow = false;

            selectButton = TEMP_HISTEREZIS;
            _PArValueTime.set = 4000;
            setValueTime.set = 1000;


#if DEBUG
            Serial.print("TEMP_HISTEREZIS :");
            Serial.println(selectButton);
#endif

          } else if ( selectButton == PAR ) {

            btnPowerTime.state = true;

            stateTempHisterezis = false;
            stateTempShow = true;
            statePArShow = false;
            stateModeShow = false;

            selectButton = SETTINGS;
            btnSettingsTime.state = false;

            _PArCloseTime.set = 0;
            _PArCloseTime.counter = 0;


#if DEBUG
            Serial.print("SETTINGS :");
            Serial.println(selectButton);
#endif

          } else if ( selectButton == PAR_0_ACTIVE || selectButton == PAR_1_ACTIVE ||
                      selectButton == PAR_2_ACTIVE || selectButton == PAR_3_ACTIVE ||
                      selectButton == PAR_4_ACTIVE || selectButton == PAR_5_ACTIVE ||
                      selectButton == PAR_6_ACTIVE || selectButton == PAR_7_ACTIVE ||
                      selectButton == PAR_8_ACTIVE || selectButton == PAR_9_ACTIVE|| selectButton == PAR_10_ACTIVE) {
            //omer added
            //PAr_Value = 0;

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

            /* EEPROM'dan degerler alinir. */
            eepromRead();

#if DEBUG
            Serial.print("NO SAVE GO TO PAR_ACTIVE :");
            Serial.println(selectButton);
#endif

            _PArCloseTime.counter = 0;
          } else if ( selectButton == PAR_ACTIVE || selectButton == PAR_0 ||
                      selectButton == PAR_1 || selectButton == PAR_2 ||
                      selectButton == PAR_3 || selectButton == PAR_4 ||
                      selectButton == PAR_5 || selectButton == PAR_6 ||
                      selectButton == PAR_7 || selectButton == PAR_8 ||
                      selectButton == PAR_9 ||selectButton == PAR_10) {

            btnPowerTime.state = true;

            stateTempHisterezis = false;
            stateTempShow = true;
            statePArShow = false;
            stateModeShow = false;

            selectButton = SETTINGS;
            btnSettingsTime.state = false;

#if DEBUG
            Serial.print("SETTINGS :");
            Serial.println(selectButton);
#endif

            _PArCloseTime.counter = 0;
            _PArCloseTime.set = 0;
          }

        } else {
          if ( selectButton == TEMP_HISTEREZIS ) {

            btnPowerTime.state = true;

            stateTempHisterezis = false;
            stateTempShow = true;
            statePArShow = false;
            stateModeShow = false;

            selectButton = SETTINGS;
            setValueTime.set = 0;
            setValueTime.counter = 0;

            /* EEPROM 'dan degerler Okunur. */
            //eepromSetValue = EEPROM.read(SET_VALUE);
            //milkHisterezisSetValue = eepromSetValue/10;
            prefs.begin("settings", true);
            milkHisterezisSetValue = prefs.getFloat("milkHisterezisSetValue", milkHisterezisSetValue);
            prefs.end();


#if DEBUG
            Serial.print("SETTINGS :");
            Serial.println(selectButton);
#endif
          }
        }

        btnSettingsTime.set = 0;
      }
    }

    /* "Guc Acma Kapama" Butonuna basildiginda Cihaz Kapanir/Acilir.
      Ayarica Parametre Ekraninda iken Parametre Ayari Yapildiktan sonra
      Degeri kaydetmek icin "Guc Acma Kapama" Butonuna basilir.
    */
    if ( btnPowerTime.set != 0 ) {
      if ( ++btnPowerTime.count >= btnPowerTime.set ) {
        btnPowerTime.count = 0;
        btnPowerTime.state = !btnPowerTime.state;


        if ( selectButton == PAR_0_ACTIVE || selectButton == PAR_1_ACTIVE ||
             selectButton == PAR_2_ACTIVE || selectButton == PAR_3_ACTIVE ||
             selectButton == PAR_4_ACTIVE || selectButton == PAR_5_ACTIVE ||
             selectButton == PAR_6_ACTIVE || selectButton == PAR_7_ACTIVE ||
             selectButton == PAR_8_ACTIVE || selectButton == PAR_9_ACTIVE || selectButton == PAR_10_ACTIVE ) {
          //omer added
          //PAr_Value = 0;

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

          /* EEPROM'a data kayit edilir. */
          eepromWrite();


#if DEBUG
          Serial.print("SAVE GO TO PAR_ACTIVE :");
          Serial.println(selectButton);
#endif

          _PArCloseTime.counter = 0;
        }

        if ( selectButton == SETTINGS ) {
          if ( btnPowerTime.state ) {
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

            /* SOGUTUCU */
            freezeRoleOff.set = 1000;
            //LedMod(LED_A,LOW);
            //digitalWrite(ROLE_A,LOW);

            /* KARISTIRICI */
            LedMod(LED_B, LOW);
            digitalWrite(ROLE_B, LOW);

            /* KOMPRESSOR */
            digitalWrite(ROLE_C, LOW);

            confuseFasilaOn.set = 0;
            confuseFasilaOn.counter = 0;
            confuseFasilaOff.set = 0;
            confuseFasilaOff.counter = 0;

          }
        }

        btnPowerTime.set = 0;
      }
    }

    /* "Ayarlar" Butonuna basildiktan sonra Sicaklik Ayari Yapilip Tekrardan "Ayarlar" Butonuna basilip cikilmaz ise
      "setValueTime.set" calisir ve deger kaydedilip Ana Ekrana donulur.
    */
    if ( setValueTime.set != 0 ) {
      if ( ++setValueTime.count >= setValueTime.set ) {
        setValueTime.count = 0;
        setValueTime.state = !setValueTime.state;

        if ( ++setValueTime.counter >= 7) {

          stateTempHisterezis = false;
          stateTempShow = true;
          statePArShow = false;
          stateModeShow = false;

          selectButton = SETTINGS;
          btnSettingsTime.state = false;

          /* EEPROM Write Sicaklik Histerezis */
          //EEPROM.write(SET_VALUE,milkHisterezisSetValue*10);
          prefs.begin("settings", false);
          prefs.putFloat("milkHisterezisSetValue", milkHisterezisSetValue);
          prefs.end();


#if DEBUG
          Serial.print("SETTINGS :");
          Serial.println(selectButton);
#endif

          setValueTime.counter = 0;
          setValueTime.set = 0;
        }
      }
    }

    if ( confuseFasilaOn.set != 0 ) {
      if ( ++confuseFasilaOn.count >= confuseFasilaOn.set ) {
        confuseFasilaOn.count = 0;
        confuseFasilaOn.state = true;

        LedMod(LED_B, HIGH);
        digitalWrite(ROLE_B, HIGH);

        if ( ++confuseFasilaOn.counter >= (confuseOnTime * 10) * 60 ) {
          confuseFasilaOn.counter = 0;
          confuseFasilaOn.set = 0;
          confuseFasilaOff.set = 1000;
        }
      }
    }

    if ( confuseFasilaOff.set != 0 ) {
      if ( ++confuseFasilaOff.count >= confuseFasilaOff.set ) {
        confuseFasilaOff.count = 0;

        /* KARISTIRICI */
        LedMod(LED_B, LOW);
        digitalWrite(ROLE_B, LOW);

        /* SOGUTUCU */
        //digitalWrite(LED_A,LOW);
        digitalWrite(ROLE_A, LOW);

        /* KOMPRESSOR */
        //digitalWrite(LED_A,LOW);
        digitalWrite(ROLE_C, LOW);

        if ( ++confuseFasilaOff.counter >= (confuseOffTime * 10) * 60 ) {
          confuseFasilaOff.counter = 0;
          confuseFasilaOff.set = 0;
          confuseFasilaOn.set = 1000;
        }
      }
    }

    if ( freezeRoleOn.set != 0 ) {
      if ( ++freezeRoleOn.count >= freezeRoleOn.set ) {
        freezeRoleOn.count = 0;

        if ( ++freezeRoleOn.counter >= (roleOnValue * 10) ) {

          freezeRoleOn.state = true;
          freezeRoleOff.state = false;

          /* SOGUTUCU */
          LedMod(LED_A, HIGH);
          digitalWrite(ROLE_A, HIGH);

          freezeRoleOn.counter = 0;
          freezeRoleOn.set = 0;
        }
      }
    }

    if ( freezeRoleOff.set != 0 ) {
      if ( ++freezeRoleOff.count >= freezeRoleOff.set ) {
        freezeRoleOff.count = 0;

        if ( ++freezeRoleOff.counter >= (roleOffValue * 10) ) {

          freezeRoleOff.state = true;
          freezeRoleOn.state = false;

          /* SOGUTUCU */
          LedMod(LED_A, LOW);
          digitalWrite(ROLE_A, LOW);

          freezeRoleOff.counter = 0;
          freezeRoleOff.set = 0;

        }
      }
    }

    if ( systemAccelerationBlink.set != 0 ) {
      if ( ++systemAccelerationBlink.count >= systemAccelerationBlink.set ) {
        systemAccelerationBlink.count = 0;
        systemAccelerationBlink.state = !systemAccelerationBlink.state;

        /*Gyro check*/
        //Gyro_angle();
      }
    }

    if ( _PArValueTime.set != 0 ) {
      if ( ++_PArValueTime.count >= _PArValueTime.set ) {
        _PArValueTime.count = 0;
        _PArValueTime.state = true;

        stateTempHisterezis = false;
        stateTempShow = false;
        statePArShow = true;
        stateModeShow = false;

        selectButton = PAR;
        btnSettingsTime.state = false;
        //omer added to init the P0.0 every sigin
        PAr_Value = 0;

#if DEBUG
        Serial.print("PAR :");
        Serial.println(selectButton);
#endif

        _PArCloseTime.set = 1000;
        _PArValueTime.set = 0;

        setValueTime.counter = 0;
        setValueTime.set = 0;
      }
    }

    if ( _PArCloseTime.set != 0 ) {
      if ( ++_PArCloseTime.count >= _PArCloseTime.set ) {
        _PArCloseTime.count = 0;

        if ( ++_PArCloseTime.counter >= 30 ) {

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

#if DEBUG
          Serial.print("PAR_CLOSE :");
          Serial.println(selectButton);
#endif

          _PArCloseTime.set = 0;
          _PArCloseTime.counter = 0;
        }
      }
    }

    if ( _7segmentBlinkSendState.set != 0 ) {
      if ( ++_7segmentBlinkSendState.count >= _7segmentBlinkSendState.set ) {
        _7segmentBlinkSendState.count = 0;
        _7segmentBlinkSendState.state = !_7segmentBlinkSendState.state;

        if ( _7segmentBlinkSendState.state ) {
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

  bl_process();
}
