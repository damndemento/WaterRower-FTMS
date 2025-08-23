/************************************************/
/* Project: Vendomnia Rower BLE for ESP32lolin  */
/*   many thanks to zpukr's ArduRower Project   */
/*    at https://github.com/zpukr/ArduRower     */
/*                                              */
/************************************************/

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLEAdvertising.h>
#include <BLE2902.h>
#include "RunningAverage.h" // https://github.com/RobTillaart/Arduino/tree/master/libraries/RunningAverage
#include "SSD1306.h"
#include "font.h" // Created by http://oleddisplay.squix.ch/

// Oled Display
#define SDA 5
#define SCL 4
#define RST_LED 16 // RST must be set by software
// SSD1306  display(0x3c, SDA, SCL, RST_LED);
SSD1306 display(0x3c, 5, 4, GEOMETRY_128_64); // ADDRESS, SDA, SCL, GEOMETRY_128_32 (or 128_64)

// Global Define
#define _VERSION 1.0
#define BLE_SERVICE_NAME "Vendomnia" // name of the Bluetooth Service

#define SerialDebug Serial // Usb is used by S4, additionnal port on SAMD21

#define FitnessMachineService 0x1826
#define FitnessMachineControlPoint 0x2AD9 // Beta Implementation
#define FitnessMachineFeature 0x2ACC      // CX Not implemented yet
#define FitnessMachineStatus 0x2ADA       // Beta Implementation
#define FitnessMachineRowerData 0x2AD1    // CX Main cx implemented
#define DEVICE_INFORMATION 0x180A

BLEServer *pServer = NULL;
BLECharacteristic *pCtrCharacteristic;
BLECharacteristic *pDtCharacteristic;
BLECharacteristic *pFmfCharacteristic;
BLECharacteristic *pStCharacteristic;
BLECharacteristic *pBatCharacteristic;
BLEAdvertising *pAdvertising;

BLECharacteristic *pCharacteristic24;
BLECharacteristic *pCharacteristic25;
BLECharacteristic *pCharacteristic26;
BLECharacteristic *pCharacteristic27;
BLECharacteristic *pCharacteristic28;
BLECharacteristic *pCharacteristic29;

bool deviceConnected = false;
bool oldDeviceConnected = false;
//// Service
int32_t fitnessMachineServiceId;
uint16_t rowerDataFlagsP1 = 0b0000011111110;
uint16_t rowerDataFlagsP2 = 0b1111100000001;

struct rowerDataKpi
{
  int bpm; // Start of Part 1
  int strokeCount;
  // int tmpstrokeRate;
  int strokeRate;
  int averageStokeRate;
  int totalDistance;
  int instantaneousPace;
  int tmpinstantaneousPace;
  int averagePace;
  int instantaneousPower;
  int averagePower;
  int resistanceLevel;
  int totalEnergy; // Start of Part 2
  int energyPerHour;
  int energyPerMinute;
  int heartRate;
  int metabolicEquivalent;
  int elapsedTime;
  int elapsedTimeSec;
  int elapsedTimeMin;
  int elapsedTimeHour;
  int remainingTime;
};

struct rowerDataKpi rdKpi;

bool bleInitFlag = false;
bool bleConnectionStatus = false;

unsigned long currentTime = 0;
unsigned long previousTime = 0;

/*-----------------SETUP--PINS-----------------------------*/
const int ROWERINPUT = 2; // the input pin where the waterrower sensor is connected
const int ROWERINPUT2 = 3; // ADD: second reed sensor pin

/*-----------------CONSTANTS-------------------------------*/
// const float Ratio = 4.8; // from old script 4.8; meters per rpm = circumference of rotor (D=34cm) -> 1,068m -> Ratio = 0.936 ; WaterRower 7,3 m/St. -> Ratio: 3.156
const float Ratio = 0.79; // one magnet
const int MAGNET_COUNT = 1; // ADD: change this for multiple magnets (2, 4, 6, etc.)
// const float Ratio = 3.156;

RunningAverage stm_RA(21); // size of array for strokes/min
RunningAverage mps_RA(7);  // size of array for meters/second -> emulates momentum of boat

/*-----------------VARIABLES-----------------------------*/
long start;
float rounds;
volatile unsigned long click_time = 0;
volatile unsigned long last_click_time = 0;
volatile unsigned long old_split = 0;
volatile unsigned long split_time = 0;
volatile unsigned long start_split;
long debouncing_time = 15; // Debouncing Time in Milliseconds
volatile unsigned long last_micros;
volatile int clicks = 0;
volatile int clicks_old = 0;
volatile float data_output = 0;
volatile long rpm = 0;
volatile long old_rpm = 0;
volatile long stm = 0;
volatile int old_strokes = 0;
volatile float spm = 0;
int old_spm = 0;
volatile long stmra = 0;
int accel = 0;
int puffer = 0;
volatile float Ms = 0;
volatile int meters = 0;
volatile int meters_old = 0;
// volatile int trigger = 0;
unsigned long timer1 = 0;
unsigned long timer2 = 0;
int stage = 0; // sets case for what parameter to be displayed
int rotation = 0;
int strokes = 0;
int trend = 0;
int resetTimer = 0;
int stoprowing = 0; // since the esp32lolin has no usable buttons this is used for the rowing while loop

// ADD: New variables for dual sensor stroke detection
volatile int lastSensor = 0;           // Track which sensor triggered last (1 or 2)  
volatile int directionChanges = 0;     // Count direction changes

class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    deviceConnected = true;
    SerialDebug.println("CONNECTED");
    resetTimer = 0;
  };
  void onDisconnect(BLEServer *pServer)
  {
    deviceConnected = false;
    SerialDebug.println("DISCONNECTED");
  }
};

class MyCallbacks : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pCharacteristic)
  {
    std::string rxValue = pCharacteristic->getValue();
    if (rxValue.length() > 0)
    {
      SerialDebug.println("*********");
      SerialDebug.print("Received Value: ");
      if (rxValue.empty())
        SerialDebug.print("-Null-");
      else
      {
        for (int i = 0; i < rxValue.length(); i++)
        {
          char chr = rxValue[i];
          SerialDebug.print(chr + "Int:" + (int)chr);
        }
      }
      SerialDebug.println();
      SerialDebug.println("*********");
    }
  }
};

void initBLE()
{
  // Initialise the module
  SerialDebug.print(F("Init BLE:"));

  // Create the BLE Device
  BLEDevice::init(BLE_SERVICE_NAME);

  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Add the Fitness Machine Service definition
  // Service ID should be 1
  SerialDebug.println(F("Adding the Fitness Machine Service definition (UUID = 0x1826): "));

  // Create the BLE Service
  BLEService *pService = pServer->createService(BLEUUID((uint16_t)FitnessMachineService));

  // Add the Fitness Machine Rower Data characteristic
  // Chars ID for Measurement should be 1
  pDtCharacteristic = pService->createCharacteristic(
      BLEUUID((uint16_t)FitnessMachineRowerData),
      BLECharacteristic::PROPERTY_NOTIFY);
  pDtCharacteristic->addDescriptor(new BLE2902());
  pDtCharacteristic->setCallbacks(new MyCallbacks());

  pFmfCharacteristic = pService->createCharacteristic(
      BLEUUID((uint16_t)FitnessMachineFeature),
      BLECharacteristic::PROPERTY_READ);
  pFmfCharacteristic->addDescriptor(new BLE2902());

  // Add the Fitness Machine Control Point characteristic
  pCtrCharacteristic = pService->createCharacteristic(
      BLEUUID((uint16_t)FitnessMachineControlPoint),
      BLECharacteristic::PROPERTY_WRITE);
  pCtrCharacteristic->addDescriptor(new BLE2902());
  pCtrCharacteristic->setCallbacks(new MyCallbacks());

  // Add the Fitness Machine Status characteristic
  pStCharacteristic = pService->createCharacteristic(
      BLEUUID((uint16_t)FitnessMachineStatus),
      BLECharacteristic::PROPERTY_NOTIFY);
  pStCharacteristic->addDescriptor(new BLE2902());

  // Start the service
  pService->start();

  // -------------------------------------------------------------------------------------

  BLEService *pService2 = pServer->createService(BLEUUID((uint16_t)DEVICE_INFORMATION));
  pCharacteristic24 = pService2->createCharacteristic(
      BLEUUID((uint16_t)0x2A24),
      BLECharacteristic::PROPERTY_READ);
  pCharacteristic25 = pService2->createCharacteristic(
      BLEUUID((uint16_t)0x2A25),
      BLECharacteristic::PROPERTY_READ);
  pCharacteristic26 = pService2->createCharacteristic(
      BLEUUID((uint16_t)0x2A26),
      BLECharacteristic::PROPERTY_READ);
  pCharacteristic27 = pService2->createCharacteristic(
      BLEUUID((uint16_t)0x2A27),
      BLECharacteristic::PROPERTY_READ);
  pCharacteristic28 = pService2->createCharacteristic(
      BLEUUID((uint16_t)0x2A28),
      BLECharacteristic::PROPERTY_READ);
  pCharacteristic29 = pService2->createCharacteristic(
      BLEUUID((uint16_t)0x2A29),
      BLECharacteristic::PROPERTY_READ);
  pService2->start();

  pCharacteristic24->setValue("4");
  pCharacteristic25->setValue("0000");
  pCharacteristic26->setValue("0.30");
  pCharacteristic27->setValue("2.2BLE");
  pCharacteristic28->setValue("4.3");
  pCharacteristic29->setValue("Waterrower");

  char cRower[8];
  cRower[0] = 0x26;
  cRower[1] = 0x56;
  cRower[2] = 0x00;
  cRower[3] = 0x00;
  cRower[4] = 0x00;
  cRower[5] = 0x00;
  cRower[6] = 0x00;
  cRower[7] = 0x00;
  pFmfCharacteristic->setValue((uint8_t *)cRower, 8);

  // --------------------------------------------------------------------------------------------------------

  // Start advertising
  SerialDebug.println(F("Adding Fitness Machine Service UUID to the advertising payload "));
  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  BLEAdvertisementData oAdvertisementData;
  oAdvertisementData.addData(std::string{0x02, 0x01, 0x06, 0x05, 0x02, 0x26, 0x18, 0x0a, 0x18});
  pAdvertising->setScanResponse(true);
  pAdvertising->setAdvertisementData(oAdvertisementData);
  pAdvertising->addServiceUUID(BLEUUID((uint16_t)FitnessMachineService));
  BLEDevice::startAdvertising();

  bleInitFlag = true;
  SerialDebug.println();
}

void initBleData()
{
  rdKpi.bpm = 0;
  rdKpi.strokeCount = 0;
  // rdKpi.tmpstrokeRate=0;
  rdKpi.strokeRate = 0;
  rdKpi.averageStokeRate = 0;
  rdKpi.totalDistance = 0;
  rdKpi.tmpinstantaneousPace = 0;
  rdKpi.instantaneousPace = 0;
  rdKpi.averagePace = 0;
  rdKpi.instantaneousPower = 0;
  rdKpi.averagePower = 0;
  rdKpi.resistanceLevel = 0;
  rdKpi.totalEnergy = 0;
  rdKpi.energyPerHour = 0;
  rdKpi.energyPerMinute = 0;
  rdKpi.heartRate = 0;
  rdKpi.metabolicEquivalent = 0;
  rdKpi.elapsedTime = 0;
  rdKpi.elapsedTimeSec = 0;
  rdKpi.elapsedTimeMin = 0;
  rdKpi.elapsedTimeHour = 0;
  rdKpi.remainingTime = 0;
}

void setCxLightRowerData()
{
#ifdef DEEPTRACE
  SerialDebug.printf("sendBleLightData() start");
#endif
  // This function is a subset of field to be sent in one piece
  uint16_t rowerDataFlags = 0b0000001111110; // 0x7E

  // 0000000000001 - 1   - 0x001 + More Data 0 <!> WARNINNG <!> This Bit is working the opposite way, 0 means field is present, 1 means not present
  // 0000000000010 - 2   - 0x002 + Average Stroke present
  // 0000000000100 - 4   - 0x004 + Total Distance Present
  // 0000000001000 - 8   - 0x008 + Instantaneous Pace present
  // 0000000010000 - 16  - 0x010 + Average Pace Present
  // 0000000100000 - 32  - 0x020 + Instantaneous Power present
  // 0000001000000 - 64  - 0x040 + Average Power present
  // 0000010000000 - 128 - 0x080 - Resistance Level present
  // 0000100000000 - 256 - 0x080 + Expended Energy present
  // 0001000000000 - 512 - 0x080 - Heart Rate present
  // 0010000000000 - 1024- 0x080 - Metabolic Equivalent present
  // 0100000000000 - 2048- 0x080 - Elapsed Time present
  // 1000000000000 - 4096- 0x080 - Remaining Time present

  //  C1  Stroke Rate             uint8     Position    2  + (After the Flag 2bytes)
  //  C1  Stroke Count            uint16    Position    3  +
  //  C2  Average Stroke Rate     uint8     Position    5  +
  //  C3  Total Distance          uint24    Position    6  +
  //  C4  Instantaneous Pace      uint16    Position    9  +
  //  C5  Average Pace            uint16    Position    11 +
  //  C6  Instantaneous Power     sint16    Position    13 +
  //  C7  Average Power           sint16    Position    15 +
  //  C8  Resistance Level        sint16    Position    17 -
  //  C9  Total Energy            uint16    Position    19 +
  //  C9  Energy Per Hour         uint16    Position    21 +
  //  C9  Energy Per Minute       uint8     Position    23 +
  //  C10 Heart Rate              uint8     Position    24 -
  //  C11 Metabolic Equivalent    uint8     Position    25 -
  //  C12 Elapsed Time            uint16    Position    26 -
  //  C13 Remaining Time          uint16    Position    28 -

  char cRower[17];

  cRower[0] = rowerDataFlags & 0x000000FF;
  cRower[1] = (rowerDataFlags & 0x0000FF00) >> 8;

  // rdKpi.strokeRate=rdKpi.tmpstrokeRate*2;
  cRower[2] = rdKpi.strokeRate & 0x000000FF;

  cRower[3] = rdKpi.strokeCount & 0x000000FF;
  cRower[4] = (rdKpi.strokeCount & 0x0000FF00) >> 8;

  cRower[5] = rdKpi.averageStokeRate & 0x000000FF;

  cRower[6] = rdKpi.totalDistance & 0x000000FF;
  cRower[7] = (rdKpi.totalDistance & 0x0000FF00) >> 8;
  cRower[8] = (rdKpi.totalDistance & 0x00FF0000) >> 16;

  if (rdKpi.tmpinstantaneousPace > 0) // Avoid Divide by Zero
    rdKpi.instantaneousPace = (100000 / rdKpi.tmpinstantaneousPace) / 2;
  cRower[9] = rdKpi.instantaneousPace & 0x000000FF;
  cRower[10] = (rdKpi.instantaneousPace & 0x0000FF00) >> 8;

  cRower[11] = rdKpi.averagePace & 0x000000FF;
  cRower[12] = (rdKpi.averagePace & 0x0000FF00) >> 8;

  cRower[13] = rdKpi.instantaneousPower & 0x000000FF;
  cRower[14] = (rdKpi.instantaneousPower & 0x0000FF00) >> 8;

  cRower[15] = rdKpi.averagePower & 0x000000FF;
  cRower[16] = (rdKpi.averagePower & 0x0000FF00) >> 8;

  pDtCharacteristic->setValue((uint8_t *)cRower, 17);
  pDtCharacteristic->notify();

#ifdef DEEPTRACE
  SerialDebug.printf("sendBleLightData() end");
#endif
}

void setCxRowerData()
{
  // Due the size limitation of the message in the BLE Stack of the NRF
  // the message will be split in 2 parts with the according Bitfield (read the spec :) )
  // rowerDataFlagsP1=0b0000011111110 = 0xFE
  // rowerDataFlagsP2=0b1111100000001 = 0x1F01

  char cRower[19]; // P1 is the biggest part whereas P2 is 13

  // Send the P1 part of the Message
  cRower[0] = rowerDataFlagsP1 & 0x000000FF;
  cRower[1] = (rowerDataFlagsP1 & 0x0000FF00) >> 8;

  cRower[2] = rdKpi.strokeRate & 0x000000FF;

  cRower[3] = rdKpi.strokeCount & 0x000000FF;
  cRower[4] = (rdKpi.strokeCount & 0x0000FF00) >> 8;

  cRower[5] = rdKpi.averageStokeRate & 0x000000FF;

  cRower[6] = rdKpi.totalDistance & 0x000000FF;
  cRower[7] = (rdKpi.totalDistance & 0x0000FF00) >> 8;
  cRower[8] = (rdKpi.totalDistance & 0x00FF0000) >> 16;

  cRower[9] = rdKpi.instantaneousPace & 0x000000FF;
  cRower[10] = (rdKpi.instantaneousPace & 0x0000FF00) >> 8;

  cRower[11] = rdKpi.averagePace & 0x000000FF;
  cRower[12] = (rdKpi.averagePace & 0x0000FF00) >> 8;

  cRower[13] = rdKpi.instantaneousPower & 0x000000FF;
  cRower[14] = (rdKpi.instantaneousPower & 0x0000FF00) >> 8;

  cRower[15] = rdKpi.averagePower & 0x000000FF;
  cRower[16] = (rdKpi.averagePower & 0x0000FF00) >> 8;

  cRower[17] = rdKpi.resistanceLevel & 0x000000FF;
  cRower[18] = (rdKpi.resistanceLevel & 0x0000FF00) >> 8;

  pDtCharacteristic->setValue((uint8_t *)cRower, 19);
  pDtCharacteristic->notify();

  // Send the P2 part of the Message
  cRower[0] = rowerDataFlagsP2 & 0x000000FF;
  cRower[1] = (rowerDataFlagsP2 & 0x0000FF00) >> 8;

  cRower[2] = rdKpi.totalEnergy & 0x000000FF;
  cRower[3] = (rdKpi.totalEnergy & 0x0000FF00) >> 8;

  cRower[4] = rdKpi.energyPerHour & 0x000000FF;
  cRower[5] = (rdKpi.energyPerHour & 0x0000FF00) >> 8;

  cRower[6] = rdKpi.energyPerMinute & 0x000000FF;

  cRower[7] = rdKpi.bpm & 0x000000FF;

  cRower[8] = rdKpi.metabolicEquivalent & 0x000000FF;

  cRower[9] = rdKpi.elapsedTime & 0x000000FF;
  cRower[10] = (rdKpi.elapsedTime & 0x0000FF00) >> 8;

  cRower[11] = rdKpi.remainingTime & 0x000000FF;
  cRower[12] = (rdKpi.remainingTime & 0x0000FF00) >> 8;

  pDtCharacteristic->setValue((uint8_t *)cRower, 13);
  pDtCharacteristic->notify();
}

void row_start()
{
  display.clear();
  display.setColor(WHITE);
  display.setFont(ArialMT_Plain_24);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(64, 10, "Waiting");
  display.drawString(64, 34, "for start");
  display.display();
  delay(100);
  while (clicks == 0)
  {
  };

  reset();
  rowing();
}

void variableParameter()
{
  // lcd.setCursor(9, 0);
  switch (stage)
  {
  case 1:
    // lcd.print("Rot");
    data_output = clicks;
    break;
  case 2:
    // lcd.print("Rpm");
    data_output = rpm;
    break;
  case 3:
    // lcd.print("Trend ");
    if (trend == 0)
    {
      // lcd.print("c");
    }
    if (trend == 1)
    {
      // lcd.print("-");
    }
    if (trend == 2)
    {
      // lcd.print("+");
    }
    data_output = 1;
    break;
  case 4:
    // lcd.print("St ");
    data_output = strokes;
    break;
  case 5:
    // lcd.print("StA");
    data_output = stm;
    break;
  case 6:
    // lcd.print("StM");
    data_output = stmra;
    break;
  default:
    // lcd.print("Sp5");
    data_output = split_time;
    break;
  }
  if (data_output >= 10)
  {
    if (data_output >= 100)
    {
      // lcd.print(" ");
    }
    else
    {
      // lcd.print("  ");
    }
  }
  else
  {
    // lcd.print("   ");
  }
  // lcd.print(data_output);
}

/*------------------INTERRUPT-----------------------------------*/
void IRAM_ATTR calcrpm()
{
  click_time = millis();
  rpm = 60000 / (click_time - last_click_time);
  last_click_time = click_time;
  accel = rpm - old_rpm;
  
  // REMOVED: Old acceleration-based stroke detection no longer needed
  /*
  if ((accel > 3) && (puffer == 0))
  { // > 20 to eliminate micro "acceleration" with 4 magnets due to splashing water ( > 3 for one magnet)
    strokes++;
    puffer = 3; // prevents counting two strokes due to still accelerating rotor
  }
  if (puffer > 0)
  {
    puffer--;
  }
  */
  
  old_rpm = rpm;
}

// MODIFIED: Add sensor parameter to existing function
void IRAM_ATTR rowerinterrupt(int sensorNum)
{
  /*  if (trigger == 0) {
      reset();
      trigger = 1;
    }*/
  if (rotation == 0)
  { // prevents double-count due to switch activating twice on the magnet passing once
    clicks++;
    rotation = 1;
    calcrpm();
    
    // NEW: Add double detection logic for stroke counting
    if (lastSensor == sensorNum) {
      // Same sensor triggered twice = direction change!
      directionChanges++;
      SerialDebug.println("Direction change detected! Count: " + String(directionChanges));
      
      // Every 2 direction changes = 1 stroke (replaces old acceleration logic)
      if (directionChanges >= 2) {
        strokes++;
        directionChanges = 0;
        SerialDebug.println("NEW STROKE! Total: " + String(strokes));
      }
    }
    lastSensor = sensorNum;  // Update last sensor
  }
  else
  {
    rotation = 0;
  }
}

// EXISTING: Keep original function, add sensor 1 parameter
void IRAM_ATTR rowerdebounceinterrupt()
{
  noInterrupts();
  if ((long)(micros() - last_micros) >= debouncing_time * 1000)
  {
    rowerinterrupt(1);  // Call with sensor 1
    last_micros = micros();
  }
  interrupts();
}

// NEW: Add second debounce interrupt function for sensor 2
void IRAM_ATTR rowerdebounceinterrupt2()
{
  noInterrupts();
  if ((long)(micros() - last_micros) >= debouncing_time * 1000)
  {
    rowerinterrupt(2);  // Call with sensor 2  
    last_micros = micros();
  }
  interrupts();
}
/*------------------INTERRUPT END-----------------------------------*/

// MODIFIED: Account for multiple magnets in distance calculation
float calcmetersmin()
{
  // OLD: meters = clicks / Ratio;
  // NEW: Account for multiple magnets
  meters = clicks / (Ratio * MAGNET_COUNT);
  
  stm = (strokes * 60000) / (millis() - start);
  
  // OLD: mps_RA.addValue((clicks - clicks_old) / Ratio);
  // NEW: Account for multiple magnets  
  mps_RA.addValue((clicks - clicks_old) / (Ratio * MAGNET_COUNT));
  
  float x = mps_RA.getAverage(); // use of floating average emulates the momentum of a boat
  clicks_old = clicks;
  return x;
}

// calculate strokes/minute as rolling average
void calcstmra()
{
  stm_RA.addValue(strokes - old_strokes);
  old_strokes = strokes;
  stmra = stm_RA.getAverage() * 60;
  if (stmra > stm)
  {
    trend = 2;
  }
  if (stmra < stm)
  {
    trend = 1;
  }
  if (stmra == stm)
  {
    trend = 0;
  }
}

// calculate 500m split time (in seconds)
void split()
{
  start_split = millis();
  split_time = (start_split - old_split) / 1000;
  old_split = millis();
}

// MODIFIED: Add new dual sensor variables to reset
void reset()
{
  clicks = 0;
  clicks_old = 0;
  meters = 0;
  meters_old = 0;
  click_time = 0;
  old_split = 0;
  split_time = 0;
  Ms = 0;
  timer1 = 0;
  timer2 = 0;
  start = millis();
  old_split = millis();
  strokes = 0;
  old_strokes = 0;
  stmra = 0;
  stm = 0;
  rpm = 0;
  stm_RA.clear();
  mps_RA.clear();
  trend = 0;
  resetTimer = 0;
  
  // NEW: Reset dual sensor variables
  lastSensor = 0;
  directionChanges = 0;
  // lcd.clear();
}

void setup()
{
  // Start the OLED Display
  display.init();
  display.setFont(ArialMT_Plain_16);
  display.flipScreenVertically(); // this is to flip the screen 180 degrees

  display.clear();
  display.setColor(WHITE);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(64, 4, "Vendomnia");
  display.display();

  SerialDebug.begin(115200);
  SerialDebug.println("/************************************");
  SerialDebug.println(" * Vendomnia Rower ");
  SerialDebug.print(" * Version ");
  SerialDebug.println(_VERSION);
  SerialDebug.println(" ***********************************/");

  initBLE();
  currentTime = millis();
  previousTime = millis();
  initBleData();

  stm_RA.clear(); // explicitly start clean
  mps_RA.clear(); // explicitly start clean
  pinMode(ROWERINPUT, INPUT_PULLUP);
  pinMode(ROWERINPUT2, INPUT_PULLUP);  // NEW: Add second sensor pin

  // digitalWrite(ROWERINPUT, HIGH);
  delay(500);
  
  // MODIFIED: Attach both interrupts
  attachInterrupt(ROWERINPUT, rowerdebounceinterrupt, CHANGE);
  attachInterrupt(ROWERINPUT2, rowerdebounceinterrupt2, CHANGE);  // NEW
  
  timer1 = millis();
  timer2 = millis();
  reset();
}

void rowing()
{
  while (stoprowing == 0)  // FIXED: was using assignment instead of comparison!
  {
    // calculate split times every 500m
    if ((meters % 500) == 0 && meters > 0)
    {
      timer2 = millis();
      if ((millis() - timer2) >= 1000)
      {
        split();
      }
    }

    if ((millis() - timer1) >= 1000)
    {
      timer1 = millis();

      // calculate meters/min
      Ms = calcmetersmin();

      // calculate moving average of strokes/min
      old_spm = (int)spm;
      calcstmra();
      spm = stm_RA.getAverage() * 60;

      // connecting
      if (deviceConnected && !oldDeviceConnected)
      {
        // do stuff here on connecting
        oldDeviceConnected = deviceConnected;
      }

      if (deviceConnected)
      { //** Send a value to protopie. The value is in txValue **//

        long sec = 1000 * (millis() - start);
        rdKpi.strokeRate = (int)round(spm + old_spm);
        rdKpi.strokeCount = strokes;
        rdKpi.averageStokeRate = (int)(strokes * 60 * 2 / sec);
        rdKpi.totalDistance = meters;
        rdKpi.instantaneousPace = (int)round(500 / Ms); // pace for 500m
        float avrMs = meters / sec;
        rdKpi.averagePace = (int)round(500 / avrMs);
        rdKpi.instantaneousPower = (int)round(2.8 * Ms * Ms * Ms); // https://www.concept2.com/indoor-rowers/training/calculators/watts-calculator
        rdKpi.averagePower = (int)round(2.8 * avrMs * avrMs * avrMs);
        rdKpi.elapsedTime = sec;

        setCxRowerData();
        // setCxLightRowerData();
        // delay(500); // bluetooth stack will go into congestion, if too many packets are sent
        SerialDebug.println("Send data " + String(rdKpi.strokeCount));

        delay(10);

        // Why coxswain need this notify for start???
        char cRower[3];
        cRower[0] = 0x01;
        cRower[1] = 0x00;
        cRower[2] = 0x00;
        pDtCharacteristic->setValue((uint8_t *)cRower, 3);
        pDtCharacteristic->notify();
      }

      // disconnecting
      if (!deviceConnected && oldDeviceConnected)
      {
        pServer->startAdvertising(); // restart advertising
        SerialDebug.println("start advertising");
        oldDeviceConnected = deviceConnected;
        resetTimer = 21;
      }

      if (resetTimer > 0)
      {
        if (resetTimer == 1)
        {
          resetTimer = 0;
          clicks = 0;
          break;
        }
        resetTimer = resetTimer - 1;
      }
    }

    display.clear();
    display.setFont(ArialMT_Plain_10);
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.drawString(5, 23, "meters");
    // display.drawString(24, 53, "time");
    display.drawString(100, 23, "m/s");
    display.drawString(102, 53, "st/min");

    // set time display
    display.setFont(Roboto_Slab_Bold_27);
    display.setTextAlignment(TEXT_ALIGN_RIGHT);
    display.drawString(128, 26, String(round((spm + old_spm) / 2), 0));
    display.drawString(128, -3, String(Ms, 2)); // "m/s"
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.drawString(0, -3, String(meters, DEC)); //"m"

    /*-------SIMULATE ROWING-------------*/
    // if (random(0, 100) < 30)
    //{
    //   rowerinterrupt();
    // }
    /*-------SIMULATE ROWING-------------*/

    display.display();
  }
}

void loop()
{
  row_start();
}
