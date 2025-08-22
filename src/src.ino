/************************************************/
/* Project: Vendomnia Rower BLE for ESP32lolin  */
/*   many thanks to zpukr's ArduRower Project   */
/*    at https://github.com/zpukr/ArduRower     */
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
SSD1306 display(0x3c, SDA, SCL, GEOMETRY_128_64); // ADDRESS, SDA, SCL, GEOMETRY_128_32 (or 128_64)

// Global Define
#define _VERSION 1.0
#define BLE_SERVICE_NAME "Vendomnia"

#define SerialDebug Serial

#define FitnessMachineService 0x1826
#define FitnessMachineControlPoint 0x2AD9
#define FitnessMachineFeature 0x2ACC
#define FitnessMachineStatus 0x2ADA
#define FitnessMachineRowerData 0x2AD1
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

int32_t fitnessMachineServiceId;
uint16_t rowerDataFlagsP1 = 0b0000011111110;
uint16_t rowerDataFlagsP2 = 0b1111100000001;

struct rowerDataKpi
{
    int bpm;
    int strokeCount;
    int strokeRate;
    int averageStokeRate;
    int totalDistance;
    int instantaneousPace;
    int tmpinstantaneousPace;
    int averagePace;
    int instantaneousPower;
    int averagePower;
    int resistanceLevel;
    int totalEnergy;
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

// Pins
const int ROWERINPUT = 2;

// Constants
const float Ratio = 0.79; // meters per magnet

RunningAverage stm_RA(21);
RunningAverage mps_RA(7);

// Variables
long start;
float rounds;
volatile unsigned long click_time = 0;
volatile unsigned long last_click_time = 0;
volatile unsigned long old_split = 0;
volatile unsigned long split_time = 0;
volatile unsigned long start_split;
long debouncing_time = 15; 
volatile unsigned long last_micros;
volatile int clicks = 0;
volatile int clicks_old = 0;
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
unsigned long timer1 = 0;
unsigned long timer2 = 0;
int stage = 0;
int rotation = 0;
int strokes = 0;
int trend = 0;
int resetTimer = 0;
int stoprowing = 0;

// BLE callbacks
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
            for (int i = 0; i < rxValue.length(); i++)
            {
                char chr = rxValue[i];
                SerialDebug.print(chr + " Int:" + (int)chr);
            }
            SerialDebug.println();
            SerialDebug.println("*********");
        }
    }
};
void initBLE()
{
    SerialDebug.print(F("Init BLE:"));

    BLEDevice::init(BLE_SERVICE_NAME);

    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    SerialDebug.println(F("Adding the Fitness Machine Service definition (UUID = 0x1826): "));

    BLEService *pService = pServer->createService(BLEUUID((uint16_t)FitnessMachineService));

    pDtCharacteristic = pService->createCharacteristic(
        BLEUUID((uint16_t)FitnessMachineRowerData),
        BLECharacteristic::PROPERTY_NOTIFY);
    pDtCharacteristic->addDescriptor(new BLE2902());
    pDtCharacteristic->setCallbacks(new MyCallbacks());

    pFmfCharacteristic = pService->createCharacteristic(
        BLEUUID((uint16_t)FitnessMachineFeature),
        BLECharacteristic::PROPERTY_READ);
    pFmfCharacteristic->addDescriptor(new BLE2902());

    pCtrCharacteristic = pService->createCharacteristic(
        BLEUUID((uint16_t)FitnessMachineControlPoint),
        BLECharacteristic::PROPERTY_WRITE);
    pCtrCharacteristic->addDescriptor(new BLE2902());
    pCtrCharacteristic->setCallbacks(new MyCallbacks());

    pStCharacteristic = pService->createCharacteristic(
        BLEUUID((uint16_t)FitnessMachineStatus),
        BLECharacteristic::PROPERTY_NOTIFY);
    pStCharacteristic->addDescriptor(new BLE2902());

    pService->start();

    BLEService *pService2 = pServer->createService(BLEUUID((uint16_t)DEVICE_INFORMATION));
    pCharacteristic24 = pService2->createCharacteristic(BLEUUID((uint16_t)0x2A24), BLECharacteristic::PROPERTY_READ);
    pCharacteristic25 = pService2->createCharacteristic(BLEUUID((uint16_t)0x2A25), BLECharacteristic::PROPERTY_READ);
    pCharacteristic26 = pService2->createCharacteristic(BLEUUID((uint16_t)0x2A26), BLECharacteristic::PROPERTY_READ);
    pCharacteristic27 = pService2->createCharacteristic(BLEUUID((uint16_t)0x2A27), BLECharacteristic::PROPERTY_READ);
    pCharacteristic28 = pService2->createCharacteristic(BLEUUID((uint16_t)0x2A28), BLECharacteristic::PROPERTY_READ);
    pCharacteristic29 = pService2->createCharacteristic(BLEUUID((uint16_t)0x2A29), BLECharacteristic::PROPERTY_READ);
    pService2->start();

    pCharacteristic24->setValue("4");
    pCharacteristic25->setValue("0000");
    pCharacteristic26->setValue("0.30");
    pCharacteristic27->setValue("2.2BLE");
    pCharacteristic28->setValue("4.3");
    pCharacteristic29->setValue("Waterrower");

    char cRower[8] = {0x26, 0x56, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    pFmfCharacteristic->setValue((uint8_t *)cRower, 8);

    SerialDebug.println(F("Adding Fitness Machine Service UUID to the advertising payload "));
    pAdvertising = pServer->getAdvertising();
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
    uint16_t rowerDataFlags = 0b0000001111110;
    char cRower[17];

    cRower[0] = rowerDataFlags & 0xFF;
    cRower[1] = (rowerDataFlags >> 8) & 0xFF;

    cRower[2] = rdKpi.strokeRate & 0xFF;

    cRower[3] = rdKpi.strokeCount & 0xFF;
    cRower[4] = (rdKpi.strokeCount >> 8) & 0xFF;

    cRower[5] = rdKpi.averageStokeRate & 0xFF;

    cRower[6] = rdKpi.totalDistance & 0xFF;
    cRower[7] = (rdKpi.totalDistance >> 8) & 0xFF;
    cRower[8] = (rdKpi.totalDistance >> 16) & 0xFF;

    if (rdKpi.tmpinstantaneousPace > 0)
        rdKpi.instantaneousPace = (100000 / rdKpi.tmpinstantaneousPace) / 2;

    cRower[9] = rdKpi.instantaneousPace & 0xFF;
    cRower[10] = (rdKpi.instantaneousPace >> 8) & 0xFF;

    cRower[11] = rdKpi.averagePace & 0xFF;
    cRower[12] = (rdKpi.averagePace >> 8) & 0xFF;

    cRower[13] = rdKpi.instantaneousPower & 0xFF;
    cRower[14] = (rdKpi.instantaneousPower >> 8) & 0xFF;

    cRower[15] = rdKpi.averagePower & 0xFF;
    cRower[16] = (rdKpi.averagePower >> 8) & 0xFF;

    pDtCharacteristic->setValue((uint8_t *)cRower, 17);
    pDtCharacteristic->notify();
}

void setCxRowerData()
{
    char cRower[19];

    // P1
    cRower[0] = rowerDataFlagsP1 & 0xFF;
    cRower[1] = (rowerDataFlagsP1 >> 8) & 0xFF;

    cRower[2] = rdKpi.strokeRate & 0xFF;
    cRower[3] = rdKpi.strokeCount & 0xFF;
    cRower[4] = (rdKpi.strokeCount >> 8) & 0xFF;
    cRower[5] = rdKpi.averageStokeRate & 0xFF;
    cRower[6] = rdKpi.totalDistance & 0xFF;
    cRower[7] = (rdKpi.totalDistance >> 8) & 0xFF;
    cRower[8] = (rdKpi.totalDistance >> 16) & 0xFF;
    cRower[9] = rdKpi.instantaneousPace & 0xFF;
    cRower[10] = (rdKpi.instantaneousPace >> 8) & 0xFF;
    cRower[11] = rdKpi.averagePace & 0xFF;
    cRower[12] = (rdKpi.averagePace >> 8) & 0xFF;
    cRower[13] = rdKpi.instantaneousPower & 0xFF;
    cRower[14] = (rdKpi.instantaneousPower >> 8) & 0xFF;
    cRower[15] = rdKpi.averagePower & 0xFF;
    cRower[16] = (rdKpi.averagePower >> 8) & 0xFF;
    cRower[17] = rdKpi.resistanceLevel & 0xFF;
    cRower[18] = (rdKpi.resistanceLevel >> 8) & 0xFF;

    pDtCharacteristic->setValue((uint8_t *)cRower, 19);
    pDtCharacteristic->notify();

    // P2
    cRower[0] = rowerDataFlagsP2 & 0xFF;
    cRower[1] = (rowerDataFlagsP2 >> 8) & 0xFF;
    cRower[2] = rdKpi.totalEnergy & 0xFF;
    cRower[3] = (rdKpi.totalEnergy >> 8) & 0xFF;
    cRower[4] = rdKpi.energyPerHour & 0xFF;
    cRower[5] = (rdKpi.energyPerHour >> 8) & 0xFF;
    cRower[6] = rdKpi.energyPerMinute & 0xFF;
    cRower[7] = rdKpi.bpm & 0xFF;
    cRower[8] = rdKpi.metabolicEquivalent & 0xFF;
    cRower[9] = rdKpi.elapsedTime & 0xFF;
    cRower[10] = (rdKpi.elapsedTime >> 8) & 0xFF;
    cRower[11] = rdKpi.remainingTime & 0xFF;
    cRower[12] = (rdKpi.remainingTime >> 8) & 0xFF;

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
        // wait for first magnet pass
    }

    reset();
    rowing();
}

void variableParameter()
{
    switch (stage)
    {
        case 1: data_output = clicks; break;
        case 2: data_output = rpm; break;
        case 3:
            if (trend == 0) data_output = 1;
            else if (trend == 1) data_output = 1;
            else if (trend == 2) data_output = 1;
            break;
        case 4: data_output = strokes; break;
        case 5: data_output = stm; break;
        case 6: data_output = stmra; break;
        default: data_output = split_time; break;
    }
}

void IRAM_ATTR calcrpm()
{
    click_time = millis();
    rpm = 60000 / (click_time - last_click_time);
    last_click_time = click_time;
    accel = rpm - old_rpm;

    if ((accel > 3) && (puffer == 0))
    {
        strokes++;
        puffer = 3; // prevent double-count due to acceleration
    }

    if (puffer > 0) puffer--;

    old_rpm = rpm;
}

void IRAM_ATTR rowerinterrupt()
{
    if (rotation == 0)
    {
        clicks++;
        rotation = 1;
        calcrpm();
    }
    else
    {
        rotation = 0;
    }
}

void IRAM_ATTR rowerdebounceinterrupt()
{
    noInterrupts();
    if ((long)(micros() - last_micros) >= debouncing_time * 1000)
    {
        rowerinterrupt();
        last_micros = micros();
    }
    interrupts();
}
// calculate and return meters per second
float calcmetersmin()
{
    meters = clicks / Ratio; // total meters
    stm = (strokes * 60000) / (millis() - start); // strokes/min

    mps_RA.addValue((clicks - clicks_old) / Ratio); // rolling average to emulate boat momentum
    float x = mps_RA.getAverage();
    clicks_old = clicks;

    return x; // meters per second
}

// calculate strokes/minute as rolling average
void calcstmra()
{
    stm_RA.addValue(strokes - old_strokes);
    old_strokes = strokes;
    stmra = stm_RA.getAverage() * 60; // convert to strokes/min

    if (stmra > stm) trend = 2;    // positive trend
    else if (stmra < stm) trend = 1; // negative trend
    else trend = 0;                  // steady
}

// calculate 500m split time (in seconds)
void split()
{
    start_split = millis();
    split_time = (start_split - old_split) / 1000;
    old_split = millis();
}

// reset function
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
    strokes = 0;
    old_strokes = 0;
    stmra = 0;
    stm = 0;
    rpm = 0;
    trend = 0;
    resetTimer = 0;
    stm_RA.clear();
    mps_RA.clear();
}

void setup()
{
    // Start the OLED Display
    display.init();
    display.setFont(ArialMT_Plain_16);
    display.flipScreenVertically(); // flip screen 180°

    display.clear();
    display.setColor(WHITE);
    display.setTextAlignment(TEXT_ALIGN_CENTER);
    display.drawString(64, 4, "Vendomnia");
    display.display();

    // Serial
    SerialDebug.begin(115200);
    SerialDebug.println("/************************************");
    SerialDebug.println(" * Vendomnia Rower ");
    SerialDebug.print(" * Version ");
    SerialDebug.println(_VERSION);
    SerialDebug.println(" ***********************************/");

    // BLE
    initBLE();
    bleInitFlag = true;

    // Initialize timing
    currentTime = millis();
    previousTime = millis();

    // Initialize BLE data structure
    initBleData();

    // Clear rolling averages
    stm_RA.clear();
    mps_RA.clear();

    // Configure rower input
    pinMode(ROWERINPUT, INPUT_PULLUP);
    delay(500);

    // Attach interrupt
    attachInterrupt(ROWERINPUT, rowerdebounceinterrupt, CHANGE);

    // Initialize timers
    timer1 = millis();
    timer2 = millis();

    // Reset counters and parameters
    reset();
}

void rowing()
{
    while (stoprowing == 0)
    {
        // Calculate split times every 500 meters
        if ((meters % 500) == 0 && meters > 0)
        {
            timer2 = millis();
            if ((millis() - timer2) >= 1000)
            {
                split();
            }
        }

        // Update every second
        if ((millis() - timer1) >= 1000)
        {
            timer1 = millis();

            // Calculate meters/sec
            Ms = calcmetersmin();

            // Calculate moving average of strokes/min
            old_spm = (int)spm;
            calcstmra();
            spm = stm_RA.getAverage() * 60;

            // BLE: device connected
            if (deviceConnected && !oldDeviceConnected)
            {
                oldDeviceConnected = deviceConnected;
            }

            if (deviceConnected)
            {
                long sec = 1000 * (millis() - start);

                rdKpi.strokeRate = (int)round((spm + old_spm) / 2);
                rdKpi.strokeCount = strokes;
                rdKpi.averageStokeRate = (int)(strokes * 60 * 2 / sec);
                rdKpi.totalDistance = meters;
                rdKpi.instantaneousPace = (int)round(500 / Ms);
                float avrMs = (float)meters / sec;
                rdKpi.averagePace = (int)round(500 / avrMs);
                rdKpi.instantaneousPower = (int)round(2.8 * Ms * Ms * Ms);
                rdKpi.averagePower = (int)round(2.8 * avrMs * avrMs * avrMs);
                rdKpi.elapsedTime = sec;

                // Send BLE data
                setCxRowerData();

                SerialDebug.println("Send data " + String(rdKpi.strokeCount));

                delay(10);

                // Notify start for coxswain
                char cRower[3] = {0x01, 0x00, 0x00};
                pDtCharacteristic->setValue((uint8_t *)cRower, 3);
                pDtCharacteristic->notify();
            }

            // BLE: device disconnected
            if (!deviceConnected && oldDeviceConnected)
            {
                pServer->startAdvertising();
                SerialDebug.println("start advertising");
                oldDeviceConnected = deviceConnected;
                resetTimer = 21;
            }

            // Handle reset timer
            if (resetTimer > 0)
            {
                if (resetTimer == 1)
                {
                    resetTimer = 0;
                    clicks = 0;
                    break;
                }
                resetTimer--;
            }
        }

        // Update OLED display
        display.clear();
        display.setFont(ArialMT_Plain_10);
        display.setTextAlignment(TEXT_ALIGN_LEFT);
        display.drawString(5, 23, "meters");
        display.drawString(100, 23, "m/s");
        display.drawString(102, 53, "st/min");

        display.setFont(Roboto_Slab_Bold_27);
        display.setTextAlignment(TEXT_ALIGN_RIGHT);
        display.drawString(128, 26, String(round((spm + old_spm) / 2), 0));
        display.drawString(128, -3, String(Ms, 2));
        display.setTextAlignment(TEXT_ALIGN_LEFT);
        display.drawString(0, -3, String(meters, DEC));

        display.display();
    }
}

void loop()
{
    // Start rowing when the device is ready
    if (clicks == 0)
    {
        row_start();
    }

    // Update BLE connection status
    if (bleInitFlag)
    {
        if (deviceConnected)
        {
            bleConnectionStatus = true;
        }
        else
        {
            bleConnectionStatus = false;
        }
    }

    // Periodically refresh BLE data even if strokes are zero
    if (bleConnectionStatus)
    {
        setCxRowerData();
        delay(500);
    }
}
