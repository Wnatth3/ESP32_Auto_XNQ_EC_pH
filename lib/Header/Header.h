#ifndef Header_h
#define Header_h

#include <FS.h>  //this needs to be first, or it all crashes and burns...
//
#define _TASK_STATUS_REQUEST
#include <TaskScheduler.h>
// 
#include <Arduino.h>
//
#include <Preferences.h>
//
#include <menu.h>
#include <menuIO/chainStream.h>
#include <menuIO/serialIn.h>
#include <menuIO/serialOut.h>
// Display SSD1306 128x64 OLED
#include <Wire.h>
#include <menuIO/u8g2Out.h>
// // Click Encoder
#include <ClickEncoder.h>  // Using this library: https://github.com/soligen2010/encoder.git
#include <menuIO/clickEncoderIn.h>
#include <menuIO/keyIn.h>
#include <menuIO/serialIO.h>
// WifiManager
#include <WiFiManager.h>  //https://github.com/tzapu/WiFiManager
//
#include <SPIFFS.h>
//
#include <ArduinoJson.h>  //https://github.com/bblanchon/ArduinoJson
// PubSubClient
#include <PubSubClient.h>
#include <ezLED.h>  // ezLED library
// OTA
#include <ESPmDNS.h>
#include <Update.h>
#include <WebServer.h>
#include <WiFiClient.h>
// Modbus RS485 Device
#include <ModbusMaster.h>
// DHT22
// #include <Adafruit_Sensor.h>
// #include <DHT.h>
//
#include <gpio_viewer.h>

//******************************** Defines **********************************//
#define DebugMode
#ifdef DebugMode
#define de(x)   Serial.print(x)
#define deln(x) Serial.println(x)
#else
#define de(x)
#define deln(x)
#endif

// #define simulation  // Comment this line to use real sensor

#ifdef simulation
#define turnOn  HIGH  // Active high
#define turnOff LOW
#else
#define turnOn  LOW  // Active low
#define turnOff HIGH
#endif

//******************************** Variables ********************************//

//----------------- WiFi Manager --------------//
#define resetPin 0
// default custom static IP
char static_ip[16]  = "192.168.0.40";
char static_gw[16]  = "192.168.0.1";
char static_sn[16]  = "255.255.255.0";
char static_dns[16] = "8.8.8.8";
// MQTT
char mqtt_server[16] = "192.168.0.10";
char mqtt_port[6]    = "1883";
char mqtt_user[15]   = "pi";
char mqtt_pass[15]   = "Berry1911";

bool shouldSaveConfig = false;  // flag for saving data

//----------------- Modbus --------------------//
// https://www.bizkit.ru/en/2019/02/21/12563/
// ESP32          MAX485         SHT30-RS485        Power supply
// 4         <---> RE             V+           <---> V+ (5V)
// 4         <---> DE   B-  <---> B- (Blue)
// 17 (TX)   <---> DI   A+  <---> A+ (Yellow)
// 16 (RX)   <---> RO             V-           <---> V- (GND)
// VIN (5V)  <---> VCC
// GND (GND) <---> GND

// Slave ID 1 (Defalt: 1)
#define slaveId1 1
#define slaveId2 2
#define slaveId3 3
#define reDePin  4     // RE & DE (driver enable) – enable transmitter to work & (receiver enable) – enable receiver to work
#define rxPin    16    // RO (receiver output) – receiver digital output
#define txPin    17    // DI (driver input) – transmitter digital input
#define baud     9600  // Default: 4800 (1), 9600 (2), Address: 2001

enum Para { EC,
            PH,
            WATERTEMP,
            WATERLEVEL };

ModbusMaster modbus1;  // EC Sensor
ModbusMaster modbus2;  // Water Temp and pH Sensor
ModbusMaster modbus3;  // Water Level Sensor

//----------------- Mixer ---------------------//

// Water filling
// #define waterPumpPin 13

// Stirring pump
#define stirringPumpPin 23
#define oxygenPumpPin   5

// EC
// Fert A Pump - Green
#define ccwFertA_PumpPin    33
#define cwFertA_PumpPin     25
#define enableFertA_PumpPin 32
// Fert B Pump -  Yellow
#define ccwFertB_PumpPin    26
#define cwFertB_PumpPin     27
#define enableFertB_PumpPin 14

// Setting PWM properties
#define pwmChannel0 0
#define pwmChannel1 1
#define frequency   30000
#define resolution  8
// #define maxDutyCycle (int)pow(2, resolution) - 1

// pH
#ifdef simulation
#define ecPin 39
// #define phPin 36
#else
// #define ecPin 0
// #define phPin 1
#endif

#define phUpPumpPin   19
#define phDownPumpPin 18
// DHT22
// #define dhtPin  13
// #define dhtType DHT22  // DHT 22  (AM2302), AM2321

// States
uint8_t state            = 0;
bool    onlyAdjustEc     = false;
bool    onlyAdjustPh     = false;
bool    calibration      = false;
bool    fertAPumpCalib   = false;
bool    fertBPumpCalib   = false;
bool    deviceTest       = false;
bool    ecTest           = false;
bool    fertA_PumpTest   = false;
bool    fertB_PumpTest   = false;
bool    phUpPumpTest     = false;
bool    phDownPumpTest   = false;
bool    phUpPumpTest20   = false;
bool    phDownPumpTest20 = false;
bool    stirringPumpTest = false;
bool    oxygenPumpTest   = false;
bool    phWtempTest      = false;
bool    waterLvTest      = false;
// uint8_t fetchDataState = 0;
bool isMixing;

// EC
uint16_t ecMax = 1500, ecMin = 800;
uint8_t  fertAPumpPct = 100;
uint8_t  fertBPumpPct = 100;
uint16_t fertOn = 1, fertOff = 5;  // ON and OFF interval of fertilizer pumps.

// pH
float    phMax = 6.5, phMin = 5.5;
uint16_t phOn = 1, phOff = 3;  // pH up & down pumps must have the same interval between on and off periods.
// float    medianMvPh;           // Store the median milivolt of the pH sensor.

// Water Temperature
// float waterTMax, waterTMin;  // Water Teperature

// Water Level
uint16_t waterLvMax, waterLvMin;

// Previous Values
uint8_t  prevState;
uint16_t prevWaterLvMax, prevWaterLvMin;
uint16_t prevEcMax, prevEcMin;
uint8_t  prevFertAPumpPct, prevFertBPumpPct;
uint16_t prevFertOn, prevFertOff;
float    prevPhMax, prevPhMin;
uint16_t prevPhOn, prevPhOff;
// float    prevWaterTMax, prevWaterTMin;  // Water Teperature
// float prevAirTMax, prevAirTMin;

// Monitoring Parameters
// float mAirTemp, mHumi;
uint16_t mEc;
float    mPh;
float    mWaterTemp;
uint16_t mWaterLv;

//------------------------------ DHT22 Tempe & Humi------------//
// DHT dht(dhtPin, dhtType);

//----------------- Preference ----------------//
Preferences pf;

//----------------- MQTT ----------------------//
#define ledPin LED_BUILTIN

// Publishcation Topics
// Monitor
#define pubMonitorWaterLv "esp32/monitor/mWaterLv"
// #define pubMonitorAirTemp   "esp32/monitor/mAirTemp"
// #define pubMonitorHumi      "esp32/monitor/mHumi"

#define pubMonitorEc        "esp32/monitor/mEc"
#define pubMonitorPh        "esp32/monitor/mPh"
#define pubMonitorWaterTemp "esp32/monitor/mWaterTemp"

// Setting
#define pubSetState "esp32/setting/state"
// #define pubSetCalib "esp32/setting/calib"
// #define pubFetchDataState  "esp32/setting/fetchDataState"
#define pubSetWaterLvMax   "esp32/setting/waterLvMax"
#define pubSetWaterLvMin   "esp32/setting/waterLvMin"
#define pubSetEcMax        "esp32/setting/ecMax"
#define pubSetEcMin        "esp32/setting/ecMin"
#define pubSetFertAPumpPct "esp32/setting/fertAPumpPct"
#define pubSetFertBPumpPct "esp32/setting/fertBPumpPct"
#define pubSetFertOn       "esp32/setting/fertOn"
#define pubSetFertOff      "esp32/setting/fertOff"
#define pubSetPhMax        "esp32/setting/phMax"
#define pubSetPhMin        "esp32/setting/phMin"
#define pubSetPhOn         "esp32/setting/phOn"
#define pubSetPhOff        "esp32/setting/phOff"
// #define pubSetWaterTMax    "esp32/setting/waterTMax"
// #define pubSetWaterTMin    "esp32/setting/waterTMin"

// Subscriptions Topics
// Input
#define subInputState   "server/input/state"
#define subInputRestart "server/input/restart"
// #define subFetchDataState    "server/input/fetchDataState"
#define subInputWaterLvMax   "server/input/waterLvMax"
#define subInputWaterLvMin   "server/input/waterLvMin"
#define subInputEcMax        "server/input/ecMax"
#define subInputEcMin        "server/input/ecMin"
#define subInputFertAPumpPct "server/input/fertAPumpPct"
#define subInputFertBPumpPct "server/input/fertBPumpPct"
#define subInputFertOn       "server/input/fertOn"
#define subInputFertOff      "server/input/fertOff"
#define subInputPhMax        "server/input/phMax"
#define subInputPhMin        "server/input/phMin"
#define subInputPhOn         "server/input/phOn"
#define subInputPhOff        "server/input/phOff"
// #define subInputWaterTMax    "server/input/waterTMax"
// #define subInputWaterTMin    "server/input/waterTMin"

WiFiClient   client;
PubSubClient mqtt(client);

ezLED mqttConnectedLed(ledPin);

GPIOViewer gpio_viewer;

//----------------- Tasdk Scheduler -----------//
// Scheduler menuTask;
Scheduler menMqt;  // Menu & MQTT tasks
Scheduler calib;   // Menu & MQTT tasks
Scheduler test;    // Test tasks
Scheduler mix;     // Mixing tasks
// Scheduler fetch;   // Fetching tasks

#endif