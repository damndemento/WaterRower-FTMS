#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <U8g2lib.h>
#include <RunningAverage.h>

// ------------------- SETTINGS -------------------
#define MAGNET_COUNT 2         // number of magnets mounted on the wheel
#define DEBOUNCE_TIME 15000    // micros debounce per reed sensor
const float Ratio = 0.79;      // calibration factor for your machine

// ------------------- DISPLAY -------------------
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// ------------------- VARIABLES -------------------
volatile unsigned long lastMicros1 = 0;
volatile unsigned long lastMicros2 = 0;
volatile int lastSensor = 0;
volatile int directionChanges = 0;
volatile unsigned long clicks = 0;
volatile unsigned long strokes = 0;

unsigned long clicks_old = 0;
unsigned long meters = 0;
float Ms = 0;
float old_Ms = 0;

unsigned long last_calc = 0;
unsigned long stm = 0;
unsigned long stmra = 0;
unsigned long spm = 0;
unsigned long old_spm = 0;

RunningAverage mps_RA(5);
RunningAverage spm_RA(5);

// ------------------- BLE -------------------
BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

typedef struct {
  int strokeRate;
  int strokeCount;
  int averageStokeRate;
  int totalDistance;
  int instantaneousPace;
  int averagePace;
  int instantaneousPower;
  int averagePower;
  int elapsedTime;
} RowingData;

RowingData rdKpi;

// ------------------- INTERRUPTS -------------------
void IRAM_ATTR reed1ISR() {
  unsigned long now = micros();
  if (now - lastMicros1 > DEBOUNCE_TIME) {
    clicks++;
    if (lastSensor == 1) {
      directionChanges++;
      if (directionChanges >= 2) {
        strokes++;
        directionChanges = 0;
      }
    }
    lastSensor = 1;
    lastMicros1 = now;
  }
}

void IRAM_ATTR reed2ISR() {
  unsigned long now = micros();
  if (now - lastMicros2 > DEBOUNCE_TIME) {
    clicks++;
    if (lastSensor == 2) {
      directionChanges++;
      if (directionChanges >= 2) {
        strokes++;
        directionChanges = 0;
      }
    }
    lastSensor = 2;
    lastMicros2 = now;
  }
}

// ------------------- FUNCTIONS -------------------
void calcMetersMin() {
  meters = clicks / (Ratio * MAGNET_COUNT);
  mps_RA.addValue((clicks - clicks_old) / (Ratio * MAGNET_COUNT));
  Ms = mps_RA.getAverage();
  clicks_old = clicks;
}

void calcStrokeRate() {
  unsigned long now = millis();
  if (strokes > 0) {
    stm = 60000UL / (now - last_calc);
    spm_RA.addValue(stm);
    spm = spm_RA.getAverage();
    last_calc = now;
  } else {
    spm = 0;
  }
  old_spm = spm;
}

void updateDisplay() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x13_mf);

  u8g2.setCursor(0, 12);
  u8g2.print("SPM:");
  u8g2.setCursor(40, 12);
  u8g2.print(spm);

  u8g2.setCursor(0, 28);
  u8g2.print("Meters:");
  u8g2.setCursor(60, 28);
  u8g2.print(meters);

  u8g2.setCursor(0, 44);
  u8g2.print("Strokes:");
  u8g2.setCursor(60, 44);
  u8g2.print(strokes);

  u8g2.setCursor(0, 60);
  u8g2.print("Speed:");
  u8g2.setCursor(60, 60);
  u8g2.print(Ms, 1);

  u8g2.sendBuffer();
}

void sendBLEdata() {
  rdKpi.strokeRate = (int)round(spm);
  rdKpi.strokeCount = strokes;
  rdKpi.averageStokeRate = (int)round(spm_RA.getAverage());
  rdKpi.totalDistance = meters;

  if (Ms > 0) {
    rdKpi.instantaneousPace = (int)round(500 / Ms);
  } else {
    rdKpi.instantaneousPace = 0;
  }

  rdKpi.averagePace = rdKpi.instantaneousPace; // simple placeholder
  rdKpi.instantaneousPower = (int)(Ms * 3);    // simple placeholder
  rdKpi.averagePower = rdKpi.instantaneousPower;
  rdKpi.elapsedTime = millis() / 1000;

  pCharacteristic->setValue((uint8_t *)&rdKpi, sizeof(rdKpi));
  pCharacteristic->notify();
}

// ------------------- BLE CALLBACK -------------------
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    deviceConnected = true;
  };

  void onDisconnect(BLEServer *pServer) {
    deviceConnected = false;
  }
};

// ------------------- SETUP -------------------
void setup() {
  Serial.begin(115200);
  u8g2.begin();

  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(4), reed1ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(5), reed2ISR, FALLING);

  BLEDevice::init("RowerBLE");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(BLEUUID((uint16_t)0x1818));
  pCharacteristic = pService->createCharacteristic(
      BLEUUID((uint16_t)0x2AD1),
      BLECharacteristic::PROPERTY_NOTIFY
  );
  pCharacteristic->addDescriptor(new BLE2902());

  pService->start();
  pServer->getAdvertising()->start();
}

// ------------------- LOOP -------------------
void loop() {
  calcMetersMin();
  calcStrokeRate();
  updateDisplay();

  if (deviceConnected) {
    sendBLEdata();
  }

  if (!deviceConnected && oldDeviceConnected) {
    delay(500);
    pServer->startAdvertising();
    oldDeviceConnected = deviceConnected;
  }

  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
  }

  delay(200);
}
