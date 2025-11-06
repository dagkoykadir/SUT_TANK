// SUT_TANK.ino - Complete Milk Tank Controller with BLE Integration
// Hardware: ESP32-based milk tank temperature controller
// Features: Temperature control, 7-segment display, BLE communication, accelerometer monitoring

// ============================================================================
// LIBRARY INCLUDES
// ============================================================================
#include <Wire.h>
#include <Preferences.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <ArduinoJson.h>

// ============================================================================
// PIN DEFINITIONS
// ============================================================================
// 7-Segment Display Pins
#define SEG_A     13
#define SEG_B     12
#define SEG_C     14
#define SEG_D     27
#define SEG_E     26
#define SEG_F     25
#define SEG_G     33
#define SEG_DP    32

// Digit Select Pins
#define DIGIT_1   23
#define DIGIT_2   22
#define DIGIT_3   21
#define DIGIT_4   19

// Analog Input Pins
#define NTC_PIN   34  // NTC thermistor analog input
#define BTN_PIN   35  // Button analog input

// Digital Output Pins
#define MIXER_PIN     5   // Mixer/Stirrer control
#define COOLER_PIN    18  // Cooler/Compressor control
#define POWER_PIN     17  // Power indicator/control
#define BUZZER_PIN    16  // Buzzer output

// I2C Pins (for ADXL345)
#define SDA_PIN   21
#define SCL_PIN   22

// ============================================================================
// CONSTANTS AND CONFIGURATION
// ============================================================================
#define PREFERENCES_NAMESPACE "sut_tank"
#define SERIAL_BAUD_RATE 115200

// Temperature calculation constants
#define NTC_SERIES_RESISTOR 10000.0  // 10K ohm series resistor
#define NTC_NOMINAL_RESISTANCE 10000.0  // 10K ohm at 25°C
#define NTC_NOMINAL_TEMPERATURE 25.0
#define NTC_B_COEFFICIENT 3950.0
#define ADC_MAX_VALUE 4095.0
#define KELVIN_OFFSET 273.15

// ADXL345 Registers
#define ADXL345_ADDRESS 0x53
#define ADXL345_POWER_CTL 0x2D
#define ADXL345_DATAX0 0x32

// Display refresh timing
#define DISPLAY_REFRESH_MS 5
#define DIGIT_ON_TIME_US 2000

// BLE Status transmission interval
#define BLE_STATUS_INTERVAL_MS 2000

// ============================================================================
// ENUMERATIONS
// ============================================================================
enum Buttons {
  BTN_NONE = 0,
  BTN_UP,
  BTN_DOWN,
  BTN_MENU,
  BTN_SET,
  BTN_POWER
};

enum DisplayMode {
  DISPLAY_TEMP = 0,
  DISPLAY_SETPOINT,
  DISPLAY_MENU,
  DISPLAY_ERROR
};

enum SystemState {
  STATE_OFF = 0,
  STATE_IDLE,
  STATE_COOLING,
  STATE_ERROR
};

// ============================================================================
// STRUCTURES
// ============================================================================
struct btnVariable {
  int lastValue;
  int currentValue;
  unsigned long lastDebounceTime;
  unsigned long debounceDelay;
  bool pressed;
  bool longPress;
  unsigned long pressStartTime;
};

struct tmrDelay {
  unsigned long startTime;
  unsigned long duration;
  bool active;
  bool triggered;
};

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================
// System state
SystemState systemState = STATE_OFF;
DisplayMode displayMode = DISPLAY_TEMP;
bool systemPowerOn = false;

// Temperature variables
float currentTemperature = 0.0;
float displayTemperature = 0.0;
int tempInteger = 0;
int tempDecimal = 0;

// Parameter storage (EEPROM/Preferences)
Preferences preferences;
float minFreezeSet = 4.0;      // Minimum cooling setpoint (°C)
float maxFreezeSet = 10.0;     // Maximum cooling setpoint (°C)
float tempSetpoint = 6.0;      // Target temperature (°C)
float tempHisterezis = 1.0;    // Temperature hysteresis (°C)
int mixerOnTime = 15;          // Mixer ON time (minutes)
int mixerOffTime = 45;         // Mixer OFF time (minutes)
int coolerMinOnTime = 5;       // Minimum cooler ON time (minutes)
int coolerMinOffTime = 5;      // Minimum cooler OFF time (minutes)
bool buzzerEnable = true;

// Control outputs
bool mixerActive = false;
bool coolerActive = false;

// Button handling
btnVariable btnControl;
Buttons lastButton = BTN_NONE;
Buttons currentButton = BTN_NONE;

// Timers
tmrDelay mixerTimer;
tmrDelay coolerTimer;
tmrDelay displayTimer;
unsigned long lastDisplayUpdate = 0;
unsigned long lastTempRead = 0;
unsigned long lastBLEStatusSend = 0;

// Display buffer
char displayBuffer[5] = "    ";
int currentDigit = 0;

// 7-Segment character encoding (Common Cathode)
const byte SEGMENT_CHARS[16] = {
  0b00111111, // 0
  0b00000110, // 1
  0b01011011, // 2
  0b01001111, // 3
  0b01100110, // 4
  0b01101101, // 5
  0b01111101, // 6
  0b00000111, // 7
  0b01111111, // 8
  0b01101111, // 9
  0b01110111, // A
  0b01111100, // b
  0b00111001, // C
  0b01011110, // d
  0b01111001, // E
  0b01110001  // F
};

const byte SEGMENT_MINUS = 0b01000000;
const byte SEGMENT_DEGREE = 0b01100011;
const byte SEGMENT_OFF = 0b00000000;

// ADXL345 data
int16_t accelX = 0;
int16_t accelY = 0;
int16_t accelZ = 0;
bool accelError = false;

// ============================================================================
// BLE VARIABLES AND DEFINITIONS
// ============================================================================
// Nordic UART Service UUIDs
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

static BLEServer *pServer = NULL;
static BLECharacteristic *pTxCharacteristic = NULL;
static BLECharacteristic *pRxCharacteristic = NULL;
static bool deviceConnected = false;
static bool oldDeviceConnected = false;

// ============================================================================
// BLE CALLBACK CLASSES
// ============================================================================
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("BLE Client Connected");
    }

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("BLE Client Disconnected");
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();

      if (rxValue.length() > 0) {
        Serial.print("Received BLE data: ");
        Serial.println(rxValue.c_str());
        
        // Process JSON command
        processBlECommand(rxValue.c_str());
      }
    }
};

// ============================================================================
// FUNCTION PROTOTYPES
// ============================================================================
// Temperature functions
float readNTCTemperature();
void updateTemperature();

// Display functions
void initDisplay();
void displayNumber(float value, bool showDecimal);
void displayText(const char* text);
void refreshDisplay();
void setSegment(byte segments);
void selectDigit(int digit);

// Button functions
void initButtons();
Buttons readButton();
void processButtons();
void handleButtonPress(Buttons btn);
void handleButtonLongPress(Buttons btn);

// EEPROM/Preferences functions
void loadPreferences();
void savePreferences();
void resetToDefaults();

// ADXL345 functions
void initADXL345();
void readADXL345();
bool checkADXL345();

// Control functions
void updateMixerControl();
void updateCoolerControl();
void updateOutputs();

// Timer functions
void initTimer(tmrDelay* timer, unsigned long durationMs);
void startTimer(tmrDelay* timer);
void stopTimer(tmrDelay* timer);
bool checkTimer(tmrDelay* timer);
void updateTimer(tmrDelay* timer);

// State machine
void updateStateMachine();

// BLE functions
void initBLE();
void processBlECommand(const char* jsonString);
void sendBLEStatus();
void sendBLESettings();

// ============================================================================
// SETUP FUNCTION
// ============================================================================
void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  Serial.println("SUT_TANK Starting...");

  // Initialize display pins
  pinMode(SEG_A, OUTPUT);
  pinMode(SEG_B, OUTPUT);
  pinMode(SEG_C, OUTPUT);
  pinMode(SEG_D, OUTPUT);
  pinMode(SEG_E, OUTPUT);
  pinMode(SEG_F, OUTPUT);
  pinMode(SEG_G, OUTPUT);
  pinMode(SEG_DP, OUTPUT);
  
  pinMode(DIGIT_1, OUTPUT);
  pinMode(DIGIT_2, OUTPUT);
  pinMode(DIGIT_3, OUTPUT);
  pinMode(DIGIT_4, OUTPUT);

  // Initialize control output pins
  pinMode(MIXER_PIN, OUTPUT);
  pinMode(COOLER_PIN, OUTPUT);
  pinMode(POWER_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  
  // Set initial output states
  digitalWrite(MIXER_PIN, LOW);
  digitalWrite(COOLER_PIN, LOW);
  digitalWrite(POWER_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  // Initialize analog input pins
  pinMode(NTC_PIN, INPUT);
  pinMode(BTN_PIN, INPUT);

  // Initialize I2C for ADXL345
  Wire.begin(SDA_PIN, SCL_PIN);
  
  // Initialize display
  initDisplay();
  displayText("INIT");

  // Load saved preferences
  loadPreferences();

  // Initialize ADXL345 accelerometer
  initADXL345();

  // Initialize buttons
  initButtons();

  // Initialize timers
  initTimer(&mixerTimer, mixerOnTime * 60000UL);
  initTimer(&coolerTimer, coolerMinOnTime * 60000UL);

  // Initialize BLE
  initBLE();

  // Initial temperature reading
  updateTemperature();

  // System ready
  systemState = STATE_IDLE;
  Serial.println("SUT_TANK Ready!");
  delay(1000);
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
  unsigned long currentMillis = millis();

  // Handle BLE connection state changes
  if (!deviceConnected && oldDeviceConnected) {
    delay(500); // Give the bluetooth stack time to get ready
    pServer->startAdvertising();
    Serial.println("Start advertising");
    oldDeviceConnected = deviceConnected;
  }
  
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
  }

  // Refresh display (high frequency)
  if (currentMillis - lastDisplayUpdate >= DISPLAY_REFRESH_MS) {
    lastDisplayUpdate = currentMillis;
    refreshDisplay();
  }

  // Read temperature (every 500ms)
  if (currentMillis - lastTempRead >= 500) {
    lastTempRead = currentMillis;
    updateTemperature();
  }

  // Process button inputs
  processButtons();

  // Update control state machine
  updateStateMachine();

  // Update mixer control
  updateMixerControl();

  // Update cooler control
  updateCoolerControl();

  // Apply outputs
  updateOutputs();

  // Read accelerometer
  readADXL345();

  // Send BLE status (every 2 seconds)
  if (deviceConnected && (currentMillis - lastBLEStatusSend >= BLE_STATUS_INTERVAL_MS)) {
    lastBLEStatusSend = currentMillis;
    sendBLEStatus();
  }
}

// ============================================================================
// TEMPERATURE FUNCTIONS
// ============================================================================
float readNTCTemperature() {
  // Read ADC value
  int adcValue = analogRead(NTC_PIN);
  
  // Avoid division by zero
  if (adcValue == 0 || adcValue >= ADC_MAX_VALUE) {
    return -999.0; // Error value
  }

  // Calculate NTC resistance using voltage divider
  float resistance = NTC_SERIES_RESISTOR * (ADC_MAX_VALUE / (float)adcValue - 1.0);

  // Steinhart-Hart equation (simplified Beta parameter equation)
  float steinhart;
  steinhart = resistance / NTC_NOMINAL_RESISTANCE;     // (R/Ro)
  steinhart = log(steinhart);                          // ln(R/Ro)
  steinhart /= NTC_B_COEFFICIENT;                      // 1/B * ln(R/Ro)
  steinhart += 1.0 / (NTC_NOMINAL_TEMPERATURE + KELVIN_OFFSET); // + (1/To)
  steinhart = 1.0 / steinhart;                         // Invert
  steinhart -= KELVIN_OFFSET;                          // Convert to Celsius

  return steinhart;
}

void updateTemperature() {
  float rawTemp = readNTCTemperature();
  
  // Simple filtering
  if (rawTemp > -50 && rawTemp < 100) {
    currentTemperature = currentTemperature * 0.9 + rawTemp * 0.1;
  }
  
  // Update display temperature
  displayTemperature = currentTemperature;
  tempInteger = (int)displayTemperature;
  tempDecimal = (int)((displayTemperature - tempInteger) * 10);
}

// ============================================================================
// DISPLAY FUNCTIONS
// ============================================================================
void initDisplay() {
  // Turn off all segments
  setSegment(SEGMENT_OFF);
  
  // Turn off all digits
  digitalWrite(DIGIT_1, LOW);
  digitalWrite(DIGIT_2, LOW);
  digitalWrite(DIGIT_3, LOW);
  digitalWrite(DIGIT_4, LOW);
}

void displayNumber(float value, bool showDecimal) {
  // Handle negative values
  bool isNegative = false;
  if (value < 0) {
    isNegative = true;
    value = -value;
  }
  
  // Limit range
  if (value > 999.9) {
    value = 999.9;
  }
  
  // Convert to display format
  int wholeNumber = (int)value;
  int decimal = (int)((value - wholeNumber) * 10);
  
  // Build display buffer
  if (isNegative) {
    displayBuffer[0] = '-';
    displayBuffer[1] = '0' + (wholeNumber / 10);
    displayBuffer[2] = '0' + (wholeNumber % 10);
    displayBuffer[3] = '0' + decimal;
  } else {
    if (wholeNumber >= 100) {
      displayBuffer[0] = '0' + (wholeNumber / 100);
      displayBuffer[1] = '0' + ((wholeNumber / 10) % 10);
      displayBuffer[2] = '0' + (wholeNumber % 10);
      displayBuffer[3] = '0' + decimal;
    } else if (wholeNumber >= 10) {
      displayBuffer[0] = ' ';
      displayBuffer[1] = '0' + (wholeNumber / 10);
      displayBuffer[2] = '0' + (wholeNumber % 10);
      displayBuffer[3] = '0' + decimal;
    } else {
      displayBuffer[0] = ' ';
      displayBuffer[1] = ' ';
      displayBuffer[2] = '0' + wholeNumber;
      displayBuffer[3] = '0' + decimal;
    }
  }
}

void displayText(const char* text) {
  for (int i = 0; i < 4; i++) {
    if (text[i] != '\0') {
      displayBuffer[i] = text[i];
    } else {
      displayBuffer[i] = ' ';
    }
  }
}

void refreshDisplay() {
  // Turn off all digits first
  digitalWrite(DIGIT_1, LOW);
  digitalWrite(DIGIT_2, LOW);
  digitalWrite(DIGIT_3, LOW);
  digitalWrite(DIGIT_4, LOW);
  
  // Update current display based on mode
  switch (displayMode) {
    case DISPLAY_TEMP:
      displayNumber(displayTemperature, true);
      break;
    case DISPLAY_SETPOINT:
      displayNumber(tempSetpoint, true);
      break;
    case DISPLAY_ERROR:
      displayText("ERR ");
      break;
    default:
      displayText("    ");
      break;
  }
  
  // Multiplex display - show current digit
  char ch = displayBuffer[currentDigit];
  byte segments = SEGMENT_OFF;
  
  if (ch >= '0' && ch <= '9') {
    segments = SEGMENT_CHARS[ch - '0'];
  } else if (ch >= 'A' && ch <= 'F') {
    segments = SEGMENT_CHARS[10 + (ch - 'A')];
  } else if (ch >= 'a' && ch <= 'f') {
    segments = SEGMENT_CHARS[10 + (ch - 'a')];
  } else if (ch == '-') {
    segments = SEGMENT_MINUS;
  } else if (ch == ' ') {
    segments = SEGMENT_OFF;
  }
  
  // Add decimal point for temperature display (after digit 2)
  if (currentDigit == 2 && displayMode == DISPLAY_TEMP) {
    segments |= 0b10000000; // DP bit
  }
  
  setSegment(segments);
  selectDigit(currentDigit);
  
  // Move to next digit
  currentDigit = (currentDigit + 1) % 4;
}

void setSegment(byte segments) {
  digitalWrite(SEG_A, segments & 0b00000001 ? HIGH : LOW);
  digitalWrite(SEG_B, segments & 0b00000010 ? HIGH : LOW);
  digitalWrite(SEG_C, segments & 0b00000100 ? HIGH : LOW);
  digitalWrite(SEG_D, segments & 0b00001000 ? HIGH : LOW);
  digitalWrite(SEG_E, segments & 0b00010000 ? HIGH : LOW);
  digitalWrite(SEG_F, segments & 0b00100000 ? HIGH : LOW);
  digitalWrite(SEG_G, segments & 0b01000000 ? HIGH : LOW);
  digitalWrite(SEG_DP, segments & 0b10000000 ? HIGH : LOW);
}

void selectDigit(int digit) {
  digitalWrite(DIGIT_1, digit == 0 ? HIGH : LOW);
  digitalWrite(DIGIT_2, digit == 1 ? HIGH : LOW);
  digitalWrite(DIGIT_3, digit == 2 ? HIGH : LOW);
  digitalWrite(DIGIT_4, digit == 3 ? HIGH : LOW);
}

// ============================================================================
// BUTTON FUNCTIONS
// ============================================================================
void initButtons() {
  btnControl.lastValue = 0;
  btnControl.currentValue = 0;
  btnControl.lastDebounceTime = 0;
  btnControl.debounceDelay = 50;
  btnControl.pressed = false;
  btnControl.longPress = false;
  btnControl.pressStartTime = 0;
}

Buttons readButton() {
  int adcValue = analogRead(BTN_PIN);
  
  // Button voltage divider thresholds
  // Adjust these values based on your hardware
  if (adcValue < 100) {
    return BTN_NONE;
  } else if (adcValue < 800) {
    return BTN_UP;
  } else if (adcValue < 1500) {
    return BTN_DOWN;
  } else if (adcValue < 2200) {
    return BTN_MENU;
  } else if (adcValue < 3000) {
    return BTN_SET;
  } else if (adcValue < 3800) {
    return BTN_POWER;
  }
  
  return BTN_NONE;
}

void processButtons() {
  unsigned long currentMillis = millis();
  Buttons btn = readButton();
  
  // Debounce logic
  if (btn != btnControl.lastValue) {
    btnControl.lastDebounceTime = currentMillis;
  }
  
  if ((currentMillis - btnControl.lastDebounceTime) > btnControl.debounceDelay) {
    if (btn != btnControl.currentValue) {
      btnControl.currentValue = btn;
      
      // Button pressed
      if (btn != BTN_NONE && !btnControl.pressed) {
        btnControl.pressed = true;
        btnControl.pressStartTime = currentMillis;
        currentButton = btn;
        
        // Handle button press
        handleButtonPress(btn);
      }
      
      // Button released
      if (btn == BTN_NONE && btnControl.pressed) {
        btnControl.pressed = false;
        btnControl.longPress = false;
      }
    }
    
    // Check for long press
    if (btnControl.pressed && !btnControl.longPress) {
      if ((currentMillis - btnControl.pressStartTime) > 2000) {
        btnControl.longPress = true;
        handleButtonLongPress(currentButton);
      }
    }
  }
  
  btnControl.lastValue = btn;
}

void handleButtonPress(Buttons btn) {
  switch (btn) {
    case BTN_POWER:
      systemPowerOn = !systemPowerOn;
      if (systemPowerOn) {
        systemState = STATE_IDLE;
        digitalWrite(POWER_PIN, HIGH);
        Serial.println("System ON");
      } else {
        systemState = STATE_OFF;
        digitalWrite(POWER_PIN, LOW);
        mixerActive = false;
        coolerActive = false;
        Serial.println("System OFF");
      }
      break;
      
    case BTN_UP:
      if (displayMode == DISPLAY_SETPOINT) {
        tempSetpoint += 0.5;
        if (tempSetpoint > maxFreezeSet) {
          tempSetpoint = maxFreezeSet;
        }
        savePreferences();
        Serial.print("Setpoint: ");
        Serial.println(tempSetpoint);
      }
      break;
      
    case BTN_DOWN:
      if (displayMode == DISPLAY_SETPOINT) {
        tempSetpoint -= 0.5;
        if (tempSetpoint < minFreezeSet) {
          tempSetpoint = minFreezeSet;
        }
        savePreferences();
        Serial.print("Setpoint: ");
        Serial.println(tempSetpoint);
      }
      break;
      
    case BTN_MENU:
      // Toggle display mode
      if (displayMode == DISPLAY_TEMP) {
        displayMode = DISPLAY_SETPOINT;
      } else {
        displayMode = DISPLAY_TEMP;
      }
      break;
      
    case BTN_SET:
      // Reserved for future menu navigation
      break;
      
    default:
      break;
  }
}

void handleButtonLongPress(Buttons btn) {
  if (btn == BTN_SET) {
    // Long press SET to reset to defaults
    resetToDefaults();
    displayText("RST ");
    delay(1000);
  }
}

// ============================================================================
// EEPROM/PREFERENCES FUNCTIONS
// ============================================================================
void loadPreferences() {
  preferences.begin(PREFERENCES_NAMESPACE, false);
  
  tempSetpoint = preferences.getFloat("tempSet", 6.0);
  tempHisterezis = preferences.getFloat("tempHyst", 1.0);
  minFreezeSet = preferences.getFloat("minFreeze", 4.0);
  maxFreezeSet = preferences.getFloat("maxFreeze", 10.0);
  mixerOnTime = preferences.getInt("mixerOn", 15);
  mixerOffTime = preferences.getInt("mixerOff", 45);
  coolerMinOnTime = preferences.getInt("coolMinOn", 5);
  coolerMinOffTime = preferences.getInt("coolMinOff", 5);
  buzzerEnable = preferences.getBool("buzzer", true);
  
  preferences.end();
  
  Serial.println("Preferences loaded");
}

void savePreferences() {
  preferences.begin(PREFERENCES_NAMESPACE, false);
  
  preferences.putFloat("tempSet", tempSetpoint);
  preferences.putFloat("tempHyst", tempHisterezis);
  preferences.putFloat("minFreeze", minFreezeSet);
  preferences.putFloat("maxFreeze", maxFreezeSet);
  preferences.putInt("mixerOn", mixerOnTime);
  preferences.putInt("mixerOff", mixerOffTime);
  preferences.putInt("coolMinOn", coolerMinOnTime);
  preferences.putInt("coolMinOff", coolerMinOffTime);
  preferences.putBool("buzzer", buzzerEnable);
  
  preferences.end();
  
  Serial.println("Preferences saved");
}

void resetToDefaults() {
  tempSetpoint = 6.0;
  tempHisterezis = 1.0;
  minFreezeSet = 4.0;
  maxFreezeSet = 10.0;
  mixerOnTime = 15;
  mixerOffTime = 45;
  coolerMinOnTime = 5;
  coolerMinOffTime = 5;
  buzzerEnable = true;
  
  savePreferences();
  Serial.println("Reset to defaults");
}

// ============================================================================
// ADXL345 ACCELEROMETER FUNCTIONS
// ============================================================================
void initADXL345() {
  Wire.beginTransmission(ADXL345_ADDRESS);
  Wire.write(ADXL345_POWER_CTL);
  Wire.write(0x08); // Measurement mode
  byte error = Wire.endTransmission();
  
  if (error == 0) {
    accelError = false;
    Serial.println("ADXL345 initialized");
  } else {
    accelError = true;
    Serial.println("ADXL345 initialization failed");
  }
}

void readADXL345() {
  if (accelError) {
    return;
  }
  
  Wire.beginTransmission(ADXL345_ADDRESS);
  Wire.write(ADXL345_DATAX0);
  byte error = Wire.endTransmission(false);
  
  if (error != 0) {
    accelError = true;
    return;
  }
  
  Wire.requestFrom(ADXL345_ADDRESS, 6, true);
  
  if (Wire.available() >= 6) {
    accelX = Wire.read() | (Wire.read() << 8);
    accelY = Wire.read() | (Wire.read() << 8);
    accelZ = Wire.read() | (Wire.read() << 8);
  }
}

bool checkADXL345() {
  Wire.beginTransmission(ADXL345_ADDRESS);
  byte error = Wire.endTransmission();
  return (error == 0);
}

// ============================================================================
// CONTROL FUNCTIONS
// ============================================================================
void updateMixerControl() {
  if (!systemPowerOn) {
    mixerActive = false;
    stopTimer(&mixerTimer);
    return;
  }
  
  updateTimer(&mixerTimer);
  
  if (!mixerTimer.active) {
    // Start new cycle
    if (!mixerActive) {
      // Start ON period
      mixerTimer.duration = mixerOnTime * 60000UL;
      startTimer(&mixerTimer);
      mixerActive = true;
      Serial.println("Mixer ON");
    } else {
      // Start OFF period
      mixerTimer.duration = mixerOffTime * 60000UL;
      startTimer(&mixerTimer);
      mixerActive = false;
      Serial.println("Mixer OFF");
    }
  }
}

void updateCoolerControl() {
  if (!systemPowerOn || systemState == STATE_OFF) {
    coolerActive = false;
    stopTimer(&coolerTimer);
    return;
  }
  
  updateTimer(&coolerTimer);
  
  // Temperature-based control with hysteresis
  float upperLimit = tempSetpoint + tempHisterezis;
  float lowerLimit = tempSetpoint - tempHisterezis;
  
  // Check if minimum time constraints are met
  bool canChangState = !coolerTimer.active;
  
  if (currentTemperature > upperLimit && !coolerActive && canChangState) {
    // Start cooling
    coolerActive = true;
    coolerTimer.duration = coolerMinOnTime * 60000UL;
    startTimer(&coolerTimer);
    Serial.println("Cooler ON");
  } else if (currentTemperature < lowerLimit && coolerActive && canChangState) {
    // Stop cooling
    coolerActive = false;
    coolerTimer.duration = coolerMinOffTime * 60000UL;
    startTimer(&coolerTimer);
    Serial.println("Cooler OFF");
  }
}

void updateOutputs() {
  digitalWrite(MIXER_PIN, mixerActive ? HIGH : LOW);
  digitalWrite(COOLER_PIN, coolerActive ? HIGH : LOW);
  digitalWrite(POWER_PIN, systemPowerOn ? HIGH : LOW);
}

// ============================================================================
// TIMER FUNCTIONS
// ============================================================================
void initTimer(tmrDelay* timer, unsigned long durationMs) {
  timer->startTime = 0;
  timer->duration = durationMs;
  timer->active = false;
  timer->triggered = false;
}

void startTimer(tmrDelay* timer) {
  timer->startTime = millis();
  timer->active = true;
  timer->triggered = false;
}

void stopTimer(tmrDelay* timer) {
  timer->active = false;
  timer->triggered = false;
}

bool checkTimer(tmrDelay* timer) {
  if (!timer->active) {
    return false;
  }
  
  if (millis() - timer->startTime >= timer->duration) {
    return true;
  }
  
  return false;
}

void updateTimer(tmrDelay* timer) {
  if (!timer->active) {
    return;
  }
  
  if (checkTimer(timer)) {
    timer->triggered = true;
    timer->active = false;
  }
}

// ============================================================================
// STATE MACHINE
// ============================================================================
void updateStateMachine() {
  if (!systemPowerOn) {
    systemState = STATE_OFF;
    return;
  }
  
  // Check for errors
  if (currentTemperature < -20 || currentTemperature > 50) {
    systemState = STATE_ERROR;
    displayMode = DISPLAY_ERROR;
    coolerActive = false;
    
    if (buzzerEnable) {
      digitalWrite(BUZZER_PIN, HIGH);
      delay(100);
      digitalWrite(BUZZER_PIN, LOW);
    }
    return;
  }
  
  // Normal operation
  if (systemState == STATE_ERROR) {
    systemState = STATE_IDLE;
    displayMode = DISPLAY_TEMP;
  }
  
  if (coolerActive) {
    systemState = STATE_COOLING;
  } else {
    systemState = STATE_IDLE;
  }
}

// ============================================================================
// BLE FUNCTIONS
// ============================================================================
void initBLE() {
  Serial.println("Initializing BLE...");
  
  // Create BLE Device
  BLEDevice::init("PEYMAK_BLE");
  
  // Create BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  
  // Create BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);
  
  // Create TX Characteristic
  pTxCharacteristic = pService->createCharacteristic(
                        CHARACTERISTIC_UUID_TX,
                        BLECharacteristic::PROPERTY_NOTIFY
                      );
  pTxCharacteristic->addDescriptor(new BLE2902());
  
  // Create RX Characteristic
  pRxCharacteristic = pService->createCharacteristic(
                        CHARACTERISTIC_UUID_RX,
                        BLECharacteristic::PROPERTY_WRITE
                      );
  pRxCharacteristic->setCallbacks(new MyCallbacks());
  
  // Start the service
  pService->start();
  
  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  
  Serial.println("BLE initialized, waiting for connections...");
}

void processBlECommand(const char* jsonString) {
  // Parse JSON
  StaticJsonDocument<512> doc;
  DeserializationError error = deserializeJson(doc, jsonString);
  
  if (error) {
    Serial.print("JSON parse failed: ");
    Serial.println(error.c_str());
    return;
  }
  
  // Check command type
  const char* cmd = doc["cmd"];
  if (cmd == nullptr) {
    Serial.println("No command in JSON");
    return;
  }
  
  Serial.print("Processing command: ");
  Serial.println(cmd);
  
  if (strcmp(cmd, "GET_SETTINGS") == 0) {
    // Send current settings
    sendBLESettings();
  }
  else if (strcmp(cmd, "SET_TEMP") == 0) {
    // Update temperature setpoint
    if (doc.containsKey("value")) {
      float newSetpoint = doc["value"];
      if (newSetpoint >= minFreezeSet && newSetpoint <= maxFreezeSet) {
        tempSetpoint = newSetpoint;
        savePreferences();
        Serial.print("Temperature setpoint updated: ");
        Serial.println(tempSetpoint);
        sendBLESettings();
      }
    }
  }
  else if (strcmp(cmd, "SET_HYST") == 0) {
    // Update hysteresis
    if (doc.containsKey("value")) {
      float newHyst = doc["value"];
      if (newHyst >= 0.5 && newHyst <= 5.0) {
        tempHisterezis = newHyst;
        savePreferences();
        Serial.print("Hysteresis updated: ");
        Serial.println(tempHisterezis);
        sendBLESettings();
      }
    }
  }
  else if (strcmp(cmd, "SET_MIXER_ON") == 0) {
    // Update mixer ON time
    if (doc.containsKey("value")) {
      int newTime = doc["value"];
      if (newTime >= 1 && newTime <= 60) {
        mixerOnTime = newTime;
        savePreferences();
        Serial.print("Mixer ON time updated: ");
        Serial.println(mixerOnTime);
        sendBLESettings();
      }
    }
  }
  else if (strcmp(cmd, "SET_MIXER_OFF") == 0) {
    // Update mixer OFF time
    if (doc.containsKey("value")) {
      int newTime = doc["value"];
      if (newTime >= 1 && newTime <= 120) {
        mixerOffTime = newTime;
        savePreferences();
        Serial.print("Mixer OFF time updated: ");
        Serial.println(mixerOffTime);
        sendBLESettings();
      }
    }
  }
  else if (strcmp(cmd, "CTRL_POWER") == 0) {
    // Control power
    if (doc.containsKey("value")) {
      bool power = doc["value"];
      systemPowerOn = power;
      if (systemPowerOn) {
        systemState = STATE_IDLE;
      } else {
        systemState = STATE_OFF;
        mixerActive = false;
        coolerActive = false;
      }
      Serial.print("Power: ");
      Serial.println(systemPowerOn ? "ON" : "OFF");
    }
  }
  else if (strcmp(cmd, "CTRL_MIXER") == 0) {
    // Manual mixer control
    if (doc.containsKey("value") && systemPowerOn) {
      mixerActive = doc["value"];
      Serial.print("Mixer manual: ");
      Serial.println(mixerActive ? "ON" : "OFF");
    }
  }
  else if (strcmp(cmd, "CTRL_COOLER") == 0) {
    // Manual cooler control
    if (doc.containsKey("value") && systemPowerOn) {
      coolerActive = doc["value"];
      Serial.print("Cooler manual: ");
      Serial.println(coolerActive ? "ON" : "OFF");
    }
  }
  else if (strcmp(cmd, "RESET") == 0) {
    // Reset to defaults
    resetToDefaults();
    sendBLESettings();
    Serial.println("Settings reset to defaults");
  }
}

void sendBLEStatus() {
  if (!deviceConnected) {
    return;
  }
  
  // Create JSON status
  StaticJsonDocument<512> doc;
  
  doc["temp"] = currentTemperature;
  doc["setpoint"] = tempSetpoint;
  doc["power"] = systemPowerOn;
  doc["mixer"] = mixerActive;
  doc["cooler"] = coolerActive;
  doc["state"] = (int)systemState;
  
  // Accelerometer data
  if (!accelError) {
    doc["accel_x"] = accelX;
    doc["accel_y"] = accelY;
    doc["accel_z"] = accelZ;
  }
  
  // Serialize and send
  char jsonBuffer[512];
  serializeJson(doc, jsonBuffer);
  
  pTxCharacteristic->setValue(jsonBuffer);
  pTxCharacteristic->notify();
}

void sendBLESettings() {
  if (!deviceConnected) {
    return;
  }
  
  // Create JSON settings
  StaticJsonDocument<512> doc;
  
  doc["cmd"] = "SETTINGS";
  doc["tempSetpoint"] = tempSetpoint;
  doc["tempHysteresis"] = tempHisterezis;
  doc["minFreeze"] = minFreezeSet;
  doc["maxFreeze"] = maxFreezeSet;
  doc["mixerOnTime"] = mixerOnTime;
  doc["mixerOffTime"] = mixerOffTime;
  doc["coolerMinOnTime"] = coolerMinOnTime;
  doc["coolerMinOffTime"] = coolerMinOffTime;
  doc["buzzerEnable"] = buzzerEnable;
  
  // Serialize and send
  char jsonBuffer[512];
  serializeJson(doc, jsonBuffer);
  
  pTxCharacteristic->setValue(jsonBuffer);
  pTxCharacteristic->notify();
  
  Serial.println("Settings sent via BLE");
}
