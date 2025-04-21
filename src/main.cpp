/*
1.7 - Add delay adjustable before read  EC and pH sensor
1.6 - Update ArduinoJson 7.0. libraries and others
1.5 - Add calibration scheduler
    - Change ledcSetup and ledcAttachPin to ledcAttachChannel
1.4 - Remove RunningMedian library
    - Add GPIOviewwe library
    - Add OnlyAdjust the EC and pH menus
    - Add AsyncTCP library https://github.com/dvarrel/AsyncTCP.git
    - Add AsyncWebServer library https://github.com/me-no-dev/ESPAsyncWebServer.git
    - Improve tDetWarpper
1.3 - Improved  devices testing
1.2 - Remove simulation and unwanted lines
    - Add water level sensor
1.1 - Update U8G2 library to v2.35.17
1.0 - Base on ESP32_Auto_XNQ_EC_1.0
    - Add pH and water temperature sensor
*/

#include <FS.h>  //this needs to be first, or it all crashes and burns...
//
#include <Arduino.h>
//
// #define _TASK_SLEEP_ON_IDLE_RUN
#define _TASK_STATUS_REQUEST

#include <TaskScheduler.h>
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

//******************************** Canfiguration ****************************//
#define DebugMode  // Comment this line to disable debug mode
// #define simulation  // Uncomment this line to enable simulation made

#ifdef DebugMode
#define de(x)   Serial.print(x)
#define deln(x) Serial.println(x)
#else
#define de(x)
#define deln(x)
#endif

#ifdef simulation
#define turnOn  HIGH  // Active high
#define turnOff LOW
#else
#define turnOn  LOW  // Active low
#define turnOff HIGH
#endif

//******************************** Variables ********************************//
#define deviceName "FertMixer"
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

bool        shouldSaveConfig = false;  // flag for saving data
WiFiManager wifiManager;

//----------------- OTA Web Update ------------//
/* Style */
String style =
    "<style>#file-input,input{width:100%;height:44px;border-radius:999px;margin:10px auto;font-size:15px}"
    "input{background:#202020;border: 1px solid #777;padding:0 15px;text-align:center;color:white}body{background:#202020;font-family:sans-serif;font-size:14px;color:white}"
    "#file-input{background:#202020;padding:0;border:1px solid #ddd;line-height:44px;text-align:center;display:block;cursor:pointer}"
    "#bar,#prgbar{background-color:#f1f1f1;border-radius:10px}#bar{background-color:#4C4CEA;width:0%;height:10px}"
    "form{background:#181818;max-width:258px;margin:75px auto;padding:30px;border-radius:5px;box-shadow: 0px 0px 20px 5px #777;text-align:center}"
    ".btn{background:#4C4CEA;border-radius:999px;color:white;cursor:pointer}</style>";

/* Login page */
String loginIndex =
    "<form name=loginForm>"
    "<h1>ESP32 Login</h1>"
    "<input name=userid placeholder='Username'> "
    "<input name=pwd placeholder=Password type=Password> "
    "<input type=submit onclick=check(this.form) class=btn value=Login></form>"
    "<script>"
    "function check(form) {"
    "if(form.userid.value=='admin' && form.pwd.value=='admin')"
    "{window.open('/ud')}"
    "else"
    "{alert('Error Password or Username')}"
    "}"
    "</script>" +
    style;

/* Server Index Page */
String ud =
    "<script src='https://ajax.googleapis.com/ajax/libs/jquery/3.2.1/jquery.min.js'></script>"
    "<form method='POST' action='#' enctype='multipart/form-data' id='upload_form'>"
    "<input type='file' name='update' id='file' onchange='sub(this)' style=display:none>"
    "<label id='file-input' for='file'>   Choose file...</label>"
    "<input type='submit' class=btn value='Update'>"
    "<br><br>"
    "<div id='prg'></div>"
    "<br><div id='prgbar'><div id='bar'></div></div><br></form>"
    "<script>"
    "function sub(obj){"
    "var fileName = obj.value.split('\\\\');"
    "document.getElementById('file-input').innerHTML = '   '+ fileName[fileName.length-1];"
    "};"
    "$('form').submit(function(e){"
    "e.preventDefault();"
    "var form = $('#upload_form')[0];"
    "var data = new FormData(form);"
    "$.ajax({"
    "url: '/update',"
    "type: 'POST',"
    "data: data,"
    "contentType: false,"
    "processData:false,"
    "xhr: function() {"
    "var xhr = new window.XMLHttpRequest();"
    "xhr.upload.addEventListener('progress', function(evt) {"
    "if (evt.lengthComputable) {"
    "var per = evt.loaded / evt.total;"
    "$('#prg').html('progress: ' + Math.round(per*100) + '%');"
    "$('#bar').css('width',Math.round(per*100) + '%');"
    "}"
    "}, false);"
    "return xhr;"
    "},"
    "success:function(d, s) {"
    "console.log('success!') "
    "},"
    "error: function (a, b, c) {"
    "}"
    "});"
    "});"
    "</script>" +
    style;

WebServer server(80);

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
uint8_t  prevEcDelay;
uint8_t  prevFertAPumpPct, prevFertBPumpPct;
uint16_t prevFertOn, prevFertOff;
uint8_t  prevPhWtempDelay;
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

// Delay time before reading data from EC and pH sensors
uint8_t ecDelay      = 3;
uint8_t phWtempDelay = 4;

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
#define pubSetEcDelay      "esp32/setting/ecDelay"
#define pubSetFertOn       "esp32/setting/fertOn"
#define pubSetFertOff      "esp32/setting/fertOff"
#define pubSetPhMax        "esp32/setting/phMax"
#define pubSetPhMin        "esp32/setting/phMin"
#define pubSetPhWtempDelay "esp32/setting/phWtempDelay"
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
#define subInputEcDelay      "server/input/ecDelay"
#define subInputFertAPumpPct "server/input/fertAPumpPct"
#define subInputFertBPumpPct "server/input/fertBPumpPct"
#define subInputFertOn       "server/input/fertOn"
#define subInputFertOff      "server/input/fertOff"
#define subInputPhMax        "server/input/phMax"
#define subInputPhMin        "server/input/phMin"
#define subInputPhWtempDelay "server/input/phWtempDelay"
#define subInputPhOn         "server/input/phOn"
#define subInputPhOff        "server/input/phOff"
// #define subInputWaterTMax    "server/input/waterTMax"
// #define subInputWaterTMin    "server/input/waterTMin"

WiFiClient   client;
PubSubClient mqtt(client);

ezLED mqttConnectedLed(ledPin);

GPIOViewer gpio_viewer;

//----------------- Menu ----------------------//
using namespace Menu;

#define MAX_DEPTH 3

// Define the display------------------
#define fontName  u8g2_font_6x10_mf
#define fontX     6
#define fontY     12
#define offsetX   0
#define offsetY   1
#define U8_Width  128
#define U8_Height 64

// U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);  // ,22, 21); // page buffer, size = 128 bytes
#ifdef simulation
U8G2_ST7567_ENH_DG128064I_1_HW_I2C u8g2(U8G2_R2, U8X8_PIN_NONE);
#else
U8G2_ST7567_ENH_DG128064I_1_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
#endif

// define menu colors -----------------
const colorDef<uint8_t> colors[6] MEMMODE = {
    {{0, 0}, {0, 1, 1}}, // bgColor
    {{1, 1}, {1, 0, 0}}, // fgColor
    {{1, 1}, {1, 0, 0}}, // valColor
    {{1, 1}, {1, 0, 0}}, // unitColor
    {{0, 1}, {0, 0, 1}}, // cursorColor
    {{1, 1}, {1, 0, 0}}, // titleColor
};

// Click Encoder
#define encA     34  // 16  // CLK pin of click encoder
#define encB     35  // 17  // DT pin of click encoder
#define encBtn   39  // 4   // SW pin of click encoder
#define encSteps 2

ClickEncoder       clickEncoder = ClickEncoder(encA, encB, encBtn, encSteps);
ClickEncoderStream encStream(clickEncoder, 1);

void IRAM_ATTR onTimer() { clickEncoder.service(); }  // ESP32 timer, Start the timer to read the clickEncoder every 1 ms

//-----Custom floatField---------------

void savUChar(const char *kName, uint8_t val, const char *pubTopic) {
    pf.putUChar(kName, val);
    mqtt.publish(pubTopic, String(val).c_str());
}
void savUShort(const char *kName, uint16_t val, const char *pubTopic) {
    pf.putUShort(kName, val);
    mqtt.publish(pubTopic, String(val).c_str());
}
void savFloat(const char *kName, float val, const char *pubTopic) {
    pf.putFloat(kName, val);
    mqtt.publish(pubTopic, String(val).c_str());
}

void savState() { savUChar("kState", state, pubSetState); }
void savWaterLvMax() { savUShort("kWaterLvMax", waterLvMax, pubSetWaterLvMax); }
void savWaterLvMin() { savUShort("kWaterLvMin", waterLvMin, pubSetWaterLvMin); }
void savEcMax() { savUShort("kEcMax", ecMax, pubSetEcMax); }
void savEcMin() { savUShort("kEcMin", ecMin, pubSetEcMin); }
void savEcDelay() { savUChar("kEcDelay", ecDelay, pubSetEcDelay); }
void savFertAPumpPct() { savUChar("kFertAPumpPct", fertAPumpPct, pubSetFertAPumpPct); }
void savFertBPumpPct() { savUChar("kFertBPumpPct", fertBPumpPct, pubSetFertBPumpPct); }
void savFertOn() { savUShort("kFertOn", fertOn, pubSetFertOn); }
void savFertOff() { savUShort("kFertOff", fertOff, pubSetFertOff); }
void savPhMax() { savFloat("kPhMax", phMax, pubSetPhMax); }
void savPhMin() { savFloat("kPhMin", phMin, pubSetPhMin); }
void savPhWtempDelay() { savUChar("kPhWtempDelay", phWtempDelay, pubSetPhWtempDelay); }
void savPhOn() { savUShort("kPhOn", phOn, pubSetPhOn); }
void savPhOff() { savUShort("kPhOff", phOff, pubSetPhOff); }
// void savWaterTMax() {
//     pf.putFloat("kWaterTMax", waterTMax);
//     mqtt.publish(pubSetWaterTMax, String(waterTMax).c_str());
// }
// void savWaterTMin() {
//     pf.putFloat("kWaterTMin", waterTMin);
//     mqtt.publish(pubSetWaterTMin, String(waterTMin).c_str());
// // }

void espRestart() { ESP.restart(); }

//-----Custom floatField----------------
#define DECIMALSFLIED_DEFAULT 1  // number of decimals
template <typename T>
class decimalslField
    : public menuField<T> {  // https://github.com/neu-rah/ArduinoMenu/blob/master/examples/customField/customField/customField.ino
   private:
    idx_t decimals;

   public:
    decimalslField(constMEM menuFieldShadow<T> &shadow) : menuField<T>(shadow) { decimals = DECIMALSFLIED_DEFAULT; }
    decimalslField(
        T         &value,
        constText *text,
        constText *units,
        T          low,
        T          high,
        T          step,
        T          tune,
        action     a = doNothing,
        eventMask  e = noEvent,
        styles     s = noStyle) : decimalslField(*new menuFieldShadow<T>(value, text, units, low, high, step, tune, a, e, s)) {}

    Used printTo(navRoot &root, bool sel, menuOut &out, idx_t idx, idx_t len, idx_t panelNr = 0) override {  // https://github.com/neu-rah/ArduinoMenu/issues/94#issuecomment-290936646
        // menuFieldShadow<T>& s=*(menuFieldShadow<T>*)shadow;
        menuField<T>::reflex = menuField<T>::target();
        idx_t l              = prompt::printTo(root, sel, out, idx, len);
        // bool ed = this == root.navFocus;
        // bool sel=nav.sel==i;
        if (l < len) {
            out.print((root.navFocus == this && sel) ? (menuField<T>::tunning ? '>' : ':') : ' ');
            l++;
            if (l < len) {
                l += out.print(menuField<T>::reflex, decimals);  // NOTE: this can exceed the limits!
                if (l < len) {
                    l += print_P(out, fieldBase::units(), len);
                }
            }
        }
        return l;
    }

    void  setDecimals(idx_t d) { decimals = d; }
    idx_t getDecimals(void) { return (decimals); }
};
//-----Custom floatField----------------END
// Dashboard----------------------------------
// FIELD(var.name, title, units, min., max., step size,fine step size, action, events mask, styles)
// MENU(menuDashboard, "Dashboard", doNothing, noEvent, noStyle,
//  altFIELD(decimalslField, mWaterLv, "WaterLv:", "%", 0.0, 100.0, 0.0, 0.0, doNothing, noEvent, noStyle),
//  altFIELD(decimalslField, mEc, "EC:", "mS/cm", 0.0, 5.0, 0.0, 0.0, doNothing, noEvent, noStyle),
//  altFIELD(decimalslField, mPh, "pH:", "", 0.0, 14.0, 0.0, 0.0, doNothing, noEvent, noStyle),
//  altFIELD(decimalslField, mWaterTemp, "WaterTemp:", "C", 0.0, 50.0, 0.0, 0.0, doNothing, noEvent, noStyle),
//  altFIELD(decimalslField, mAirTemp, "AirTemp:", "C", 0.0, 50.0, 0.0, 0.0, doNothing, noEvent, noStyle),
//  altFIELD(decimalslField, mHumi, "Humidity:", "%", 0.0, 100.0, 0.0, 0.0, doNothing, noEvent, noStyle)
// EXIT("<Back"));

// Satatus----------------------------------
// TOGGLE(var.name, id, title, action, event mask, styles, value, value [,value ...])
TOGGLE(state, menuStatus, "Status: ", savState, enterEvent, noStyle,  //,doExit,enterEvent,noStyle
       VALUE("ON", 1, doNothing, noEvent),
       VALUE("OFF", 0, doNothing, noEvent));

TOGGLE(onlyAdjustEc, onlyAdjustEcOnOff, "Only Adjust EC: ", doNothing, enterEvent, noStyle,  //,doExit,enterEvent,noStyle
       VALUE("ON", 1, doNothing, noEvent),
       VALUE("OFF", 0, doNothing, noEvent));

TOGGLE(onlyAdjustPh, onlyAdjustPhOnOff, "Only Adjust pH: ", doNothing, enterEvent, noStyle,  //,doExit,enterEvent,noStyle
       VALUE("ON", 1, doNothing, noEvent),
       VALUE("OFF", 0, doNothing, noEvent));

// TOGGLE(fetchDataState, menuFetchData, "FetchData: ", doNothing, noEvent, noStyle,  //,doExit,enterEvent,noStyle
//        VALUE("ON", 1, doNothing, noEvent),
//        VALUE("OFF", 0, doNothing, noEvent));

// SELECT(var.name, id, title, action, events mask, styles, value, value [, value ...])
// SELECT(mode, modes, "Mode:", saveValues, blurEvent, noStyle // The value is seaved after leave the active
// menu.
//   ,VALUE("Manual",0,doNothing,noEvent)
//   ,VALUE("Auto",1,doNothing,noEvent)
// );

// TOGGLE(mode, modes, "Mode: ", saveValues, enterEvent, noStyle,  //,doExit,enterEvent,noStyle
//        VALUE("Manual", 0, doNothing, noEvent),
//        VALUE("Auto", 1, doNothing, noEvent));

// MENU(menuWaterLv, "Water Level", doNothing, noEvent, noStyle,
//      altFIELD(decimalslField, fullTankDst, "FullTankDst:", "cm", 30.0, 90.0, 10.0, 1.0, savFullTankDst, exitEvent, noStyle),
//      altFIELD(decimalslField, emptyTankDst, "EmptyTankDst:", "cm", 40.0, 100.0, 10.0, 1.0, savEmptyTankDst, exitEvent, noStyle),
//      altFIELD(decimalslField, fullTankGap, "fullTankGap:", "cm", 5.0, 30.0, 10.0, 1.0, savFullTankGap, exitEvent, noStyle),
//      altFIELD(decimalslField, waterLvMax, "Max:", "%", 10.0, 100.0, 10.0, 1.0, savWaterLvMax, exitEvent, noStyle),
//      altFIELD(decimalslField, waterLvMin, "Min:", "%", 5.0, 90.0, 10.0, 1.0, savWaterLvMin, exitEvent, noStyle),
//      EXIT("<Back"));

// MENU(ecSettingMenu, "EC Setting", doNothing, noEvent, noStyle,
//      FIELD(ecMax, "Max:", " uS/cm", 600, 4000, 100, 10, savEcMax, exitEvent, noStyle),
//      FIELD(ecMin, "Min:", " uS/cm", 500, 3000, 100, 10, savEcMin, exitEvent, noStyle),
//      EXIT("<Back"));

MENU(paraSettingMenu, "Parameter Setting", doNothing, noEvent, noStyle,
     FIELD(ecMax, "Max EC:", " uS/cm", 0, 4400, 100, 10, savEcMax, exitEvent, noStyle),
     FIELD(ecMin, "Min EC:", " uS/cm", 0, 4400, 100, 10, savEcMin, exitEvent, noStyle),
     FIELD(ecDelay, "EC Delay:", "s", 1, 100, 10, 1, savEcDelay, exitEvent, noStyle),
     FIELD(fertOn, "Fert On:", "s", 1, 100, 10, 1, savFertOn, exitEvent, noStyle),
     FIELD(fertOff, "Fert Off:", "s", 1, 100, 10, 1, savFertOff, exitEvent, noStyle),
     altFIELD(decimalslField, phMax, "Max pH: ", "", 0.0, 14.0, 1.0, 0.1, savPhMax, exitEvent, noStyle),
     altFIELD(decimalslField, phMin, "Min pH: ", "", 0.0, 14.0, 1.0, 0.1, savPhMin, exitEvent, noStyle),
     FIELD(phWtempDelay, "pH Wtemp Delay:", "s", 1, 100, 10, 1, savPhWtempDelay, exitEvent, noStyle),
     FIELD(phOn, "Ph On:", " s", 1, 100, 10, 1, savPhOn, exitEvent, noStyle),
     FIELD(phOff, "Ph Off:", " s", 1, 100, 10, 1, savPhOff, exitEvent, noStyle),
     FIELD(waterLvMax, "Max WaterLv:", " mm", 300, 10000, 10, 1, savWaterLvMax, exitEvent, noStyle),
     FIELD(waterLvMin, "Min WaterLv:", " mm", 20, 200, 10, 1, savWaterLvMin, exitEvent, noStyle),
     EXIT("<Back"));

// MENU(menuPh, "pH Setting", doNothing, noEvent, noStyle,
//      altFIELD(decimalslField, phMax, "Max: ", "", 0.6, 7.0, 1.0, 0.1, savPhMax, exitEvent, noStyle),
//      altFIELD(decimalslField, phMin, "Min: ", "", 0.5, 6.9, 1.0, 0.1, savPhMin, exitEvent, noStyle),
//      FIELD(phOn, "Ph On:", "s", 1, 100, 10, 1, savPhOn, exitEvent, noStyle),
//      FIELD(phOff, "Ph Off:", "s", 1, 100, 10, 1, savPhOff, exitEvent, noStyle),
//      EXIT("<Back"));

// MENU(menuWaterTemp, "WaterTemp", doNothing, noEvent, noStyle,
//      altFIELD(decimalslField, waterTMax, "Max: ", "C", 21.0, 30.0, 1.0, 0.1, savWaterTMax, exitEvent, noStyle),
//      altFIELD(decimalslField, waterTMin, "Min: ", "C", 20.0, 29.0, 1.0, 0.1, savWaterTMin, exitEvent, noStyle),
//      EXIT("<Back"));

// MENU(airTemp, "AirTemp", doNothing, noEvent, noStyle,
//      altFIELD(decimalslField, airTMax, "Max: ", "C", 21.0, 35.0, 1.0, 0.1, saveValues, exitEvent, noStyle),
//      altFIELD(decimalslField, airTMin, "Min: ", "C", 20.0, 34.0, 1.0, 0.1, saveValues, exitEvent, noStyle),
//      EXIT("<Back"));

TOGGLE(calibration, calibrationOnOff, "Calibration: ", doNothing, enterEvent, noStyle,  //,doExit,enterEvent,noStyle
       VALUE("ON", 1, doNothing, noEvent),
       VALUE("OFF", 0, doNothing, noEvent));

TOGGLE(fertAPumpCalib, fertAPumpOnOff, "Fert A Pump: ", doNothing, enterEvent, noStyle,  //,doExit,enterEvent,noStyle
       VALUE("ON", 1, doNothing, noEvent),
       VALUE("OFF", 0, doNothing, noEvent));

TOGGLE(fertBPumpCalib, fertBPumpOnOff, "Fert B Pump: ", doNothing, enterEvent, noStyle,  //,doExit,enterEvent,noStyle
       VALUE("ON", 1, doNothing, noEvent),
       VALUE("OFF", 0, doNothing, noEvent));

TOGGLE(phUpPumpTest20, phUpPump20OnOff, "pH Up Pump 20s: ", doNothing, enterEvent, noStyle,  //,doExit,enterEvent,noStyle
       VALUE("ON", 1, doNothing, noEvent),
       VALUE("OFF", 0, doNothing, noEvent));

TOGGLE(phDownPumpTest20, phDownPump20OnOff, "pH Down Pump 20s: ", doNothing, enterEvent, noStyle,  //,doExit,enterEvent,noStyle
       VALUE("ON", 1, doNothing, noEvent),
       VALUE("OFF", 0, doNothing, noEvent));

MENU(PumpCalibMenu, "Pump Calibration", doNothing, noEvent, noStyle,
     SUBMENU(calibrationOnOff),
     SUBMENU(fertAPumpOnOff),
     SUBMENU(fertBPumpOnOff),
     SUBMENU(phUpPump20OnOff),
     SUBMENU(phDownPump20OnOff),
     FIELD(fertAPumpPct, "A Pump Pct:", "%", 1, 100, 10, 1, savFertAPumpPct, exitEvent, noStyle),
     FIELD(fertBPumpPct, "B Pump Pct:", "%", 1, 100, 10, 1, savFertBPumpPct, exitEvent, noStyle),
     EXIT("<Back"));

TOGGLE(deviceTest, deviceTestOnOff, "Device Test: ", doNothing, enterEvent, noStyle,  //,doExit,enterEvent,noStyle
       VALUE("ON", 1, doNothing, noEvent),
       VALUE("OFF", 0, doNothing, noEvent));

TOGGLE(fertA_PumpTest, fertA_PumpOnOff, "FertA Pump: ", doNothing, enterEvent, noStyle,  //,doExit,enterEvent,noStyle
       VALUE("ON", 1, doNothing, noEvent),
       VALUE("OFF", 0, doNothing, noEvent));
TOGGLE(fertB_PumpTest, fertB_PumpOnOff, "FertB Pump: ", doNothing, enterEvent, noStyle,  //,doExit,enterEvent,noStyle
       VALUE("ON", 1, doNothing, noEvent),
       VALUE("OFF", 0, doNothing, noEvent));

TOGGLE(phUpPumpTest, phUpPumpOnOff, "pH Up Pump: ", doNothing, enterEvent, noStyle,  //,doExit,enterEvent,noStyle
       VALUE("ON", 1, doNothing, noEvent),
       VALUE("OFF", 0, doNothing, noEvent));

TOGGLE(phDownPumpTest, phDownPumpOnOff, "pH Down Pump: ", doNothing, enterEvent, noStyle,
       VALUE("ON", 1, doNothing, noEvent),
       VALUE("OFF", 0, doNothing, noEvent));

TOGGLE(stirringPumpTest, stirringPumpOnOff, "Stirring Pump: ", doNothing, enterEvent, noStyle,
       VALUE("ON", 1, doNothing, noEvent),
       VALUE("OFF", 0, doNothing, noEvent));

TOGGLE(oxygenPumpTest, oxygenPumpOnOff, "Oxygen Pump: ", doNothing, enterEvent, noStyle,
       VALUE("ON", 1, doNothing, noEvent),
       VALUE("OFF", 0, doNothing, noEvent));

TOGGLE(ecTest, ecTestOnOff, "EC Test: ", doNothing, enterEvent, noStyle,  //,doExit,enterEvent,noStyle
       VALUE("ON", 1, doNothing, noEvent),
       VALUE("OFF", 0, doNothing, noEvent));

TOGGLE(phWtempTest, phWtempTestOnOff, "pH&WaterTempTest: ", doNothing, enterEvent, noStyle,  //,doExit,enterEvent,noStyle
       VALUE("ON", 1, doNothing, noEvent),
       VALUE("OFF", 0, doNothing, noEvent));

TOGGLE(waterLvTest, waterLvTestOnOff, "Waterlv Test: ", doNothing, enterEvent, noStyle,  //,doExit,enterEvent,noStyle
       VALUE("ON", 1, doNothing, noEvent),
       VALUE("OFF", 0, doNothing, noEvent));

MENU(deviceTestMenu, "Device Test", doNothing, noEvent, noStyle,
     SUBMENU(deviceTestOnOff),
     SUBMENU(fertA_PumpOnOff),
     SUBMENU(fertB_PumpOnOff),
     SUBMENU(phUpPumpOnOff),
     SUBMENU(phDownPumpOnOff),
     SUBMENU(stirringPumpOnOff),
     SUBMENU(oxygenPumpOnOff),
     FIELD(mEc, "EC:", "  uS/cm", 0, 45000, 0, 0, doNothing, noEvent, noStyle),
     SUBMENU(ecTestOnOff),
     altFIELD(decimalslField, mPh, "pH:", "", 0.0, 14.0, 0.0, 0.0, doNothing, noEvent, noStyle),
     altFIELD(decimalslField, mWaterTemp, "WaterTemp:", " C", 0.0, 50.0, 0.0, 0.0, doNothing, noEvent, noStyle),
     SUBMENU(phWtempTestOnOff),
     FIELD(mWaterLv, "WaterLv:", " mm", 0, 10000, 0, 0, doNothing, noEvent, noStyle),
     SUBMENU(waterLvTestOnOff),
     EXIT("<Back"));

// FIELD(var.name, title, units, min., max., step size,fine step size, action, events mask, styles)
// MENU(menuPhCalib, "pH Calibration", doNothing, noEvent, noStyle,
//      altFIELD(decimalslField, medianPhAnl, "pH Anl:", "", 0.0, 30000.0, 0, 0, doNothing, noEvent, noStyle),
//      altFIELD(decimalslField, phVolt, "Voltage:", "V", 0.0, 5.0, 0, 0, doNothing, noEvent, noStyle),
//      SUBMENU(menuCalib),
//      altFIELD(decimalslField, phStd_1, "1 Std: ", "", 1.0, 14.0, 1.0, 0.1, savPhStd_1, exitEvent, noStyle),
//      altFIELD(decimalslField, phMeas_1, "1 Meas:", "ADC", 0.0, 30000.0, 100.0, 10.0, savPhMeas_1, exitEvent, noStyle),
//      altFIELD(decimalslField, phStd_2, "2 Std: ", "", 1.0, 14.0, 1.0, 0.1, savPhStd_2, exitEvent, noStyle),
//      altFIELD(decimalslField, phMeas_2, "2 Meas:", "ADC", 0.0, 30000.0, 100.0, 10.0, savPhMeas_2, exitEvent, noStyle),
//      EXIT("<Back"));

MENU(mainMenu, "Main menu", doNothing, noEvent, noStyle,
     //  SUBMENU(menuDashboard),
     FIELD(mWaterLv, "WaterLv:", " mm", 20, 10000, 0, 0, doNothing, noEvent, noStyle),
     FIELD(mEc, "EC:", " uS/cm", 0, 4000, 0, 0, doNothing, noEvent, noStyle),
     altFIELD(decimalslField, mPh, "pH:", "", 0.0, 14.0, 0.0, 0.0, doNothing, noEvent, noStyle),
     altFIELD(decimalslField, mWaterTemp, "WaterTemp:", " C", 0.0, 50.0, 0.0, 0.0, doNothing, noEvent, noStyle),
     SUBMENU(menuStatus),
     SUBMENU(onlyAdjustEcOnOff),
     SUBMENU(onlyAdjustPhOnOff),
     //  SUBMENU(menuCalib),
     //  SUBMENU(menuFetchData),
     //  SUBMENU(menuWaterLv),
     //  SUBMENU(ecSettingMenu),
     SUBMENU(paraSettingMenu),
     //  SUBMENU(menuPh),
     //  SUBMENU(menuWaterTemp),
     //  SUBMENU(airTemp),
     SUBMENU(PumpCalibMenu),
     //  SUBMENU(ecTestMenu),
     SUBMENU(deviceTestMenu),
     //  SUBMENU(menuPhCalib),
     OP(">>Restart", espRestart, enterEvent),
     EXIT("<Exit"));

// serialIn serial(Serial);
MENU_INPUTS(in, &encStream /*, &serial*/);

MENU_OUTPUTS(out, MAX_DEPTH,
             U8G2_OUT(u8g2, colors, fontX, fontY, offsetX, offsetY, {0, 0, U8_Width / fontX, U8_Height / fontY}),
             // SERIAL_OUT(Serial)
             NONE  // least 2 outputs
);

NAVROOT(nav, mainMenu, MAX_DEPTH, in, out);

// ESP32 timer thanks to: http://www.iotsharing.com/2017/06/how-to-use-interrupt-timer-in-arduino-esp32.html
// and: https://techtutorialsx.com/2017/10/07/esp32-arduino-timer-interrupts/
hw_timer_t  *timer    = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

//----------------- Tasdk Scheduler -----------//
// Scheduler menuTask;
Scheduler menMqt;  // Menu & MQTT tasks
Scheduler calib;   // Menu & MQTT tasks
Scheduler test;    // Test tasks
Scheduler mix;     // Mixing tasks
// Scheduler fetch;   // Fetching tasks

//******************************** Tasks ************************************//
//----------------- Menu ----------------------//
void menuLoop();
// Task tMenuLoop(0, TASK_FOREVER, &menuLoop, &menuTask, true);
Task tMenuLoop(0, TASK_FOREVER, &menuLoop, &menMqt, false);

//----------------- WiFi Manager --------------//
// void wifiManagerSetup();
void wifiResetting();
Task tWifiResetting(0, TASK_FOREVER, &wifiResetting, &menMqt, false);

//----------------- OTA Updater ---------------//
void otaWebUpdateSetup();

//----------------- MQTT ----------------------//
// int   savIntInput(int prev, const char *kPrev, String msg, const char *kVal, const char *pubTopic);
// float savFloatInput(float prev, const char *kPrev, String msg, const char *kVal, const char *pubTopic);
// void  mqttCallback(char *topic, byte *message, unsigned int length);
// void  publishSetting();
// void  subscribeInput();
void connectMqttLoop();
void reconnectMqtt();
Task tConnectMqttLoop(0, TASK_FOREVER, &connectMqttLoop, &menMqt, false);
Task tReconnectMqtt(3000, TASK_FOREVER, &reconnectMqtt, &menMqt, false);

//----------------- Fill Water ----------------//
// void waterPumpOn();
// void  waterPumpOff();
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);
// void  measureWater();
// void  waterState();
// void  fillWaterInit();
// void  fillWater();
// Task  tMeasureWater(100, TASK_FOREVER, &measureWater, &mix, false);
// Task  tWaterState(0, TASK_ONCE, &waterState, &mix, false);
// Task  tFillWaterInit(0, TASK_ONCE, &fillWaterInit, &mix, false);
// Task  tFillWater(100, TASK_FOREVER, &fillWater, &mix, false);
// Task  tWaterPumpOff(0, TASK_ONCE, &waterPumpOff, &mix, false);

//----------------- EC ------------------------//
// float calibrate(float ind, float x1, float y1, float x2, float y2);
// float calibrate(float y);

void stirringPumpOff();
void measureEcInit();
void measureEc();
void ecState();
void fillFertInit();
void fertA_On();
void fertA_Off();
void fertB_On();
void fertB_Off();
void fertB_OffDis();
Task tMeasureEcInit(0, TASK_ONCE, &measureEcInit, &mix, false);
Task tMeasureEc(500, TASK_FOREVER, &measureEc, &mix, false);
Task tEcState(0, TASK_ONCE, &ecState, &mix, false);
Task tFillFertInit(0, TASK_ONCE, &fillFertInit, &mix, false);
Task tFertA_On(fertOff * 1000, TASK_ONCE, &fertA_On, &mix, false);
Task tFertA_Off(fertOn * 1000, TASK_ONCE, &fertA_Off, &mix, false);
Task tFertB_On(fertOff * 1000, TASK_ONCE, &fertB_On, &mix, false);
Task tFertB_Off(fertOn * 1000, TASK_ONCE, &fertB_Off, &mix, false, NULL, &fertB_OffDis);
Task tStirringPumpOff(3000, TASK_ONCE, &stirringPumpOff, &mix, false);

//----------------- pH ------------------------//
void measurePhInit();
void measurePh();
void phState();
void phDownPumpOnCb();
void phDownOn();
void phDownOff();
void phDownOffDis();
void phUpPumpOnCb();
void phUpOn();
void phUpOff();
void phUpOffDis();
Task tMeasurePhInit(0, TASK_ONCE, &measurePhInit, &mix, false);
Task tMeasurePh(500, TASK_FOREVER, &measurePh, &mix, false);
Task tPhState(0, TASK_ONCE, &phState, &mix, false);
Task tPhDownPumpOn(0, TASK_ONCE, &phDownPumpOnCb, &mix, false);
Task tPhDownOn(phOff * 1000, TASK_ONCE, &phDownOn, &mix, false);
Task tPhDownOff(phOn * 1000, TASK_ONCE, &phDownOff, &mix, false, NULL, &phDownOffDis);
Task tPhUpPumpOn(0, TASK_ONCE, &phUpPumpOnCb, &mix, false);
Task tPhUpOn(phOff * 1000, TASK_ONCE, &phUpOn, &mix, false);
Task tPhUpOff(phOn * 1000, TASK_ONCE, &phUpOff, &mix, false, NULL, &phUpOffDis);

//----------------- Detection -----------------//
void detWrapper();
void getEc();
void getPhWtemp();
void decideMixing();
Task tDetWrapper(10 * TASK_SECOND, TASK_FOREVER, &detWrapper, &menMqt, false);
Task tGetEc(0, TASK_ONCE, &getEc, &menMqt, false);
Task tGetPhWtemp(0, TASK_ONCE, &getPhWtemp, &menMqt, false);
Task tDecideMixing(0, TASK_ONCE, &decideMixing, &menMqt, false);

//----------------- Only adjust the pH --------//
void onlyAdjustPhOn();
void onlyAdjustPhOff();
Task tOnlyAdjustPh(0, TASK_FOREVER, &onlyAdjustPhOn, &menMqt, false);

//----------------- Only adjust the EC ----------//

void onlyAdjustEcOn();
void onlyAdjustEcOff();
Task tOnlyAdjustEc(0, TASK_FOREVER, &onlyAdjustEcOn, &menMqt, false);

//----------------- Send Data -----------------//
void sendData();
Task tSendData(5000, TASK_FOREVER, &sendData, &menMqt, false);

//----------------- States --------------------//
void stateOff();
void stateOn();
Task tState(0, TASK_FOREVER, &stateOn, &menMqt, false);

//----------------- Calibration Mode -----------//
void calibrationOn();
void calibrationOff();
Task tCalibration(0, TASK_FOREVER, &calibrationOn, &menMqt, false);

//----------------- Fertilizer Pump Calibration Mode ----------//
void fertAPumpCalibMode();
void fertBPumpCalibMode();
void fertAPumpCalibCb();
void fertBPumpCalibCb();
Task tFertAPumpCalibMode(0, TASK_FOREVER, &fertAPumpCalibMode, &calib, false);
Task tFertAPumpCalib(20 * TASK_SECOND, 2, &fertAPumpCalibCb, &calib, false);
Task tFertBPumpCalibMode(0, TASK_FOREVER, &fertBPumpCalibMode, &calib, false);
Task tFertBPumpCalib(20 * TASK_SECOND, 2, &fertBPumpCalibCb, &calib, false);

//----------------- pH Pump Test for 20s ------//
void phUpPumpTest20s();
void phUpPumpTest20Cb();
void phDownPumpTest20s();
void phDownPumpTest20Cb();
Task tPhUpPumpTest20(0, TASK_FOREVER, &phUpPumpTest20s, &calib, false);
Task tPhUpPumpTest20Cb(20 * TASK_SECOND, 2, &phUpPumpTest20Cb, &calib, false);
Task tPhDownPumpTest20(0, TASK_FOREVER, &phDownPumpTest20s, &calib, false);
Task tPhDownPumpTest20Cb(20 * TASK_SECOND, 2, &phDownPumpTest20Cb, &calib, false);

//----------------- Device Test Mode --------------//
void deviceTestOn();
void deviceTestOff();
Task tDeviceTest(0, TASK_FOREVER, &deviceTestOn, &menMqt, false);

//----------------- EC Test Mode --------------//
void ecTestModeOn();
void ecTestModeOff();
void measEcInTestMode();
Task tEcTestMode(0, TASK_FOREVER, &ecTestModeOn, &test, false);
Task tMeasEcInTestMode(500, TASK_FOREVER, &measEcInTestMode, &test, false);

//----------------- Fertilizer Pumps Test -----//
void fertA_PumpTestOn();
void fertA_PumpTestOff();
void fertB_PumpTestOn();
void fertB_PumpTestOff();
Task tFertA_PumpTest(0, TASK_FOREVER, &fertA_PumpTestOn, &test, false);
Task tFertB_PumpTest(0, TASK_FOREVER, &fertB_PumpTestOn, &test, false);

//----------------- pH Pump Test Mode ---------//
void phUpPumpTestOn();
void phUpPumpTestOff();
void phDownPumpTestOn();
void phDownPumpTestOff();
Task tPhUpPumpTest(0, TASK_FOREVER, &phUpPumpTestOn, &test, false);
Task tPhDownPumpTest(0, TASK_FOREVER, &phDownPumpTestOn, &test, false);

//----------------- Oxygen Pump Test Mode ---------//
// void stirringPumpTestMode();
// void stirringPumpTestCb();
// Task tStirringPumpTestMode(0, TASK_FOREVER, &stirringPumpTestMode, &test, false);
// Task tStirringPumpTest(5 * TASK_SECOND, 2, &stirringPumpTestCb, &test, false);
void stirringPumpTestOn();
void stirringPumpTestOff();
Task tStirringPumpTest(0, TASK_FOREVER, &stirringPumpTestOn, &test, false);

//----------------- Oxygen Pump Test Mode ---------//
// void oxygenPumpTestMode();
// void oxygenPumpTestCb();
// Task tOxygenPumpTestMode(0, TASK_FOREVER, &oxygenPumpTestMode, &test, false);
// Task tOxygenPumpTest(5 * TASK_SECOND, 2, &oxygenPumpTestCb, &test, false);
void oxygenPumpTestOn();
void oxygenPumpTestOff();
Task tOxygenPumpTest(0, TASK_FOREVER, &oxygenPumpTestOn, &test, false);

//----------------- pH Test Mode --------------//
void phWtempTestModeOn();
void phWtempTestModeOff();
void measPhWtempInTestMode();
Task tPhWtempTestMode(0, TASK_FOREVER, &phWtempTestModeOn, &test, false);
Task tMeasPhWtempInTestMode(500, TASK_FOREVER, &measPhWtempInTestMode, &test, false);

void waterLvTestModeOn();
void waterLvTestModeOff();
void measWaterLvInTestMode();
Task tWaterLvTestMode(0, TASK_FOREVER, &waterLvTestModeOn, &test, false);
Task tMeasWaterLvInTestMode(500, TASK_FOREVER, &measWaterLvInTestMode, &test, false);

//----------------- Fetch Data ----------------//
// void fetchDataStateCb();
// bool fetchDataEn();
// void fetchData();
// void stirringPumpOff();
// void stirringPumpOffDis();
// Task tFetchDataState(0, TASK_FOREVER, &fetchDataStateCb, &fetch, false);
// Task tFetchData(3000, TASK_ONCE, &fetchData, &fetch, false, &fetchDataEn);
// Task tFetchStirringPumpOff(3000, TASK_ONCE, &stirringPumpOff, &fetch, false, NULL, &stirringPumpOffDis);

//******************************** Functions ********************************//
//----------------- Preferences ---------------//
void getValues() {
    pf.begin("memory", false);
    state = pf.getUChar("kState", 0);
    // mode = pf.getInt("kMode", 0);
    waterLvMax   = pf.getUShort("kWaterLvMax", 0);
    waterLvMin   = pf.getUShort("kWaterLvMin", 0);
    ecMax        = pf.getUShort("kEcMax", 0);
    ecMin        = pf.getUShort("kEcMin", 0);
    ecDelay      = pf.getUChar("kEcDelay", 0);
    fertAPumpPct = pf.getUChar("kFertAPumpPct", 0);
    fertBPumpPct = pf.getUChar("kFertBPumpPct", 0);
    fertOn       = pf.getUShort("kFertOn", 0);
    fertOff      = pf.getUShort("kFertOff", 0);
    phMax        = pf.getFloat("kPhMax", 0);
    phMin        = pf.getFloat("kPhMin", 0);
    phWtempDelay = pf.getUChar("kPhWtempDelay", 0);
    phOn         = pf.getUShort("kPhOn", 0);
    phOff        = pf.getUShort("kPhOff", 0);
    // waterTMax    = pf.getFloat("kWaterTMax", 0);
    // waterTMin    = pf.getFloat("kWaterTMin", 0);
    // airTMax = pf.getFloat("kAirTMax", 0);
    // airTMin = pf.getFloat("kAirTMin", 0);
    Serial.print("phWtemp: " + String(phWtempDelay) + "s");
}
//----------------- Menu Loop -----------------//
void menuLoop() {
    nav.doInput();
    if (nav.changed(0)) {  // only draw if menu changed for gfx device
        // change checking leaves more time for other tasks
        u8g2.firstPage();
        do nav.doOutput();
        while (u8g2.nextPage());
    }
}

//----------------- WiFi Manager --------------/
// callback notifying us of the need to save config
void saveConfigCallback() {
    deln("Should save config");
    shouldSaveConfig = true;
}

void wifiManagerSetup() {
    // clean FS, for testing
    //  SPIFFS.format();

    // read configuration from FS json
    deln("mounting FS...");

    if (SPIFFS.begin()) {
        deln("mounted file system");
        if (SPIFFS.exists("/config.json")) {
            // file exists, reading and loading
            deln("reading config file");
            File configFile = SPIFFS.open("/config.json", "r");
            if (configFile) {
                deln("opened config file");
                size_t size = configFile.size();
                // Allocate a buffer to store contents of the file.
                std::unique_ptr<char[]> buf(new char[size]);

                configFile.readBytes(buf.get(), size);
                JsonDocument json;
                auto         deserializeError = deserializeJson(json, buf.get());
                serializeJson(json, Serial);
                if (!deserializeError) {
                    deln("\nparsed json");
                    strcpy(mqtt_server, json["mqtt_server"]);
                    strcpy(mqtt_port, json["mqtt_port"]);
                    strcpy(mqtt_user, json["mqtt_user"]);
                    strcpy(mqtt_pass, json["mqtt_pass"]);
                    if (json["ip"]) {
                        deln("setting custom ip from config");
                        strcpy(static_ip, json["ip"]);
                        strcpy(static_gw, json["gateway"]);
                        strcpy(static_sn, json["subnet"]);
                        strcpy(static_dns, json["dns"]);
                        deln(static_ip);
                    } else {
                        deln("no custom ip in config");
                    }
                } else {
                    deln("failed to load json config");
                }
            }
        }
    } else {
        deln("failed to mount FS");
    }

    WiFiManagerParameter custom_mqtt_server("server", "MQTT Server", mqtt_server, 16);
    WiFiManagerParameter custom_mqtt_port("port", "MQTT Port", mqtt_port, 6);
    WiFiManagerParameter custom_mqtt_user("user", "MQTT User", mqtt_user, 15);
    WiFiManagerParameter custom_mqtt_pass("pass", "MQTT Password", mqtt_pass, 15);

    wifiManager.setSaveConfigCallback(saveConfigCallback);  // set config save notify callback

    // set static ip
    IPAddress _ip, _gw, _sn, _dns;
    _ip.fromString(static_ip);
    _gw.fromString(static_gw);
    _sn.fromString(static_sn);
    _dns.fromString(static_dns);
    wifiManager.setSTAStaticIPConfig(_ip, _gw, _sn, _dns);

    // add all your parameters here
    wifiManager.addParameter(&custom_mqtt_server);
    wifiManager.addParameter(&custom_mqtt_port);
    wifiManager.addParameter(&custom_mqtt_user);
    wifiManager.addParameter(&custom_mqtt_pass);

    wifiManager.setClass("invert");  // set dark theme

    // wifiManager.resetSettings();  // reset settings - for testing

    wifiManager.setMinimumSignalQuality(20);  // defaults to 8% // set minimu quality of signal so it ignores AP's under that quality

    // wifiManager.setTimeout(120); // auto restart after 30 seconds
    wifiManager.setConfigPortalTimeout(30);  // auto close configportal after 30 seconds

    // fetches ssid and pass and tries to connect
    // if it does not connect it starts an access point with the specified name
    // here  "AutoConnectAP"
    // and goes into a blocking loop awaiting configuration
    if (!wifiManager.autoConnect(deviceName, "password")) {
        deln("failed to connect and hit timeout");
        // ESP.restart();
    } else {
        deln("connected...yeey :)");
    }

    // read updated parameters
    strcpy(mqtt_server, custom_mqtt_server.getValue());
    strcpy(mqtt_port, custom_mqtt_port.getValue());
    strcpy(mqtt_user, custom_mqtt_user.getValue());
    strcpy(mqtt_pass, custom_mqtt_pass.getValue());
    deln("The values in the file are: ");
    deln("\tmqtt_server : " + String(mqtt_server));
    deln("\tmqtt_port : " + String(mqtt_port));
    deln("\tmqtt_user : " + String(mqtt_user));
    deln("\tmqtt_pass : " + String(mqtt_pass));

    // save the custom parameters to FS
    if (shouldSaveConfig) {
        deln("saving config");
        JsonDocument json;
        json["mqtt_server"] = mqtt_server;
        json["mqtt_port"]   = mqtt_port;
        json["mqtt_user"]   = mqtt_user;
        json["mqtt_pass"]   = mqtt_pass;
        // Static IP
        json["ip"]      = WiFi.localIP().toString();
        json["gateway"] = WiFi.gatewayIP().toString();
        json["subnet"]  = WiFi.subnetMask().toString();
        json["dns"]     = WiFi.dnsIP().toString();

        File configFile = SPIFFS.open("/config.json", "w");
        if (!configFile) {
            deln("failed to open config file for writing");
        }

        serializeJson(json, Serial);
        serializeJson(json, configFile);
        configFile.close();
        // end save
    }

    deln("local ip: ");
    deln(WiFi.localIP());
    deln(WiFi.gatewayIP());
    deln(WiFi.subnetMask());
    deln(WiFi.dnsIP());

    // if (WiFi.status() == WL_CONNECTED) {
    //     tConnectMqttLoop.enable();
    // }
}

void wifiResetting() {
    if (digitalRead(resetPin) == LOW) {
        delay(50);  // poor mans debounce/press-hold, code not ideal for production
        if (digitalRead(resetPin) == LOW) {
            deln("Reset Button is pressed");
            delay(5000);  // reset delay hold for 3 second
            if (digitalRead(resetPin) == LOW) {
                digitalWrite(ledPin, HIGH);
                deln("Formating SPIFF and Resetting WiFi connection.");
                SPIFFS.format();
                wifiManager.resetSettings();
                // delay(3000);
                ESP.restart();
            }
        }
    }
}

void otaWebUpdateSetup() {
    const char *host = "ESP32";
    /*use mdns for host name resolution*/
    if (!MDNS.begin(host)) {  // http://esp32.local
        Serial.println("Error setting up MDNS responder!");
        while (1) {
            delay(1000);
        }
    }
    Serial.println("mDNS responder started");
    /*return index page which is stored in ud */
    server.on("/", HTTP_GET, []() {
        server.sendHeader("Connection", "close");
        server.send(200, "text/html", loginIndex);
    });
    server.on("/ud", HTTP_GET, []() {
        server.sendHeader("Connection", "close");
        server.send(200, "text/html", ud);
    });
    /*handling uploading firmware file */
    server.on(
        "/update", HTTP_POST, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
    ESP.restart(); }, []() {
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
      Serial.printf("Update: %s\n", upload.filename.c_str());
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { //start with max available size
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      /* flashing firmware to ESP*/
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(true)) { //true to set the size to the current progress
        Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
      } else {
        Update.printError(Serial);
      }
    } });
    server.begin();
    Serial.println("\tOTA Web updater setting is done.");
}

//----------------- MQTT ----------------------//
// Saves the int input and publishes it to an MQTT topic if it has changed.
int savIntInput(int prev, const char *kPrev, String msg, const char *kVal, const char *pubTopic) {
    prev = pf.getInt(kPrev);
    de("previousVal: ");
    deln(prev);

    int crrVal = msg.toInt();
    // de("Crrent Value: ");
    // deln(crrVal);

    if (crrVal != prev) {
        prev = crrVal;
        pf.putInt(kPrev, prev);

        pf.putInt(kVal, crrVal);
        // deln("Saved server input");

        // de("Saved Value: ");
        // deln(pf.getInt(kVal));

        mqtt.publish(pubTopic, String(crrVal).c_str());
        de("Published Value: ");
        deln(crrVal);

        // de("Previous Value: ");
        // deln(prev);
    } else {
        deln("Same value");
    }
    return pf.getInt(kVal);
}
uint16_t savUCharInput(uint8_t prev, const char *kPrev, String msg, const char *kVal, const char *pubTopic) {
    prev = pf.getUChar(kPrev);
    de("previousVal: ");
    deln(prev);

    uint8_t crrVal = atol(msg.c_str());
    // de("Crrent Value: ");
    // deln(crrVal);

    if (crrVal != prev) {
        prev = crrVal;
        pf.putUChar(kPrev, prev);
        pf.putUChar(kVal, crrVal);
        // deln("Saved server input");

        // de("Saved Value: ");
        // deln(pf.getUShort(kVal));

        mqtt.publish(pubTopic, String(crrVal).c_str());
        de("Published Value: ");
        deln(crrVal);

        // de("Previous Value: ");
        // deln(prev);
    } else {
        deln("Same value");
    }
    return pf.getUChar(kVal);
}
uint16_t savUShortInput(uint16_t prev, const char *kPrev, String msg, const char *kVal, const char *pubTopic) {
    prev = pf.getUShort(kPrev);
    de("previousVal: ");
    deln(prev);

    uint16_t crrVal = atol(msg.c_str());
    // de("Crrent Value: ");
    // deln(crrVal);

    if (crrVal != prev) {
        prev = crrVal;
        pf.putUShort(kPrev, prev);
        pf.putUShort(kVal, crrVal);
        // deln("Saved server input");

        // de("Saved Value: ");
        // deln(pf.getUShort(kVal));

        mqtt.publish(pubTopic, String(crrVal).c_str());
        de("Published Value: ");
        deln(crrVal);

        // de("Previous Value: ");
        // deln(prev);
    } else {
        deln("Same value");
    }
    return pf.getUShort(kVal);
}

// Saves the float input and publishes it to an MQTT topic if it has changed.
float savFloatInput(float prev, const char *kPrev, String msg, const char *kVal, const char *pubTopic) {
    prev = pf.getFloat(kPrev);
    de("previousVal: ");
    deln(prev);

    float crrVal = msg.toFloat();
    // de("Crrent Value: ");
    // deln(crrVal);

    if (crrVal != prev) {
        prev = crrVal;
        pf.putFloat(kPrev, prev);

        pf.putFloat(kVal, crrVal);
        // deln("Saved server input");

        // de("Saved Value: ");
        // deln(pf.getFloat(kVal));

        mqtt.publish(pubTopic, String(crrVal).c_str());
        de("Published Value: ");
        deln(crrVal);

        // de("Previous Value: ");
        // deln(prev);
    } else {
        deln("Same value");
    }
    return pf.getFloat(kVal);
}

void mqttCallback(char *topic, byte *message, unsigned int length) {
    de("Message arrived on topic: ");
    de(topic);
    de(". Message: ");
    String msg;

    for (int i = 0; i < length; i++) {
        de((char)message[i]);
        msg += (char)message[i];
    }
    deln();

    if (!strcmp(topic, subInputState)) {
        state = msg.toInt();
        pf.putUChar("kState", state);
        deln("State: " + String(state ? "ON" : "OFF"));
    } else if (!strcmp(topic, subInputRestart)) {
        espRestart();
        // } else if (!strcmp(topic, subFetchDataState)) {
        //     fetchDataState = msg.toInt();
        //     deln("Fetch Data State: " + String(fetchDataState ? "ON" : "OFF"));
    } else if (!strcmp(topic, subInputFertAPumpPct) || !strcmp(topic, subInputFertBPumpPct) ||
               !strcmp(topic, subInputEcDelay) || !strcmp(topic, subInputPhWtempDelay)) {
        if (!strcmp(topic, subInputFertAPumpPct)) {
            fertOn = savUCharInput(prevFertAPumpPct, "kPrevAPumpPct", msg, "kFertAPumpPct", pubSetFertAPumpPct);
        } else if (!strcmp(topic, subInputFertBPumpPct)) {
            fertOff = savUCharInput(prevFertBPumpPct, "kPrevBPumpPct", msg, "kFertBPumpPct", pubSetFertBPumpPct);
        } else if (!strcmp(topic, subInputEcDelay)) {
            ecDelay = savUCharInput(prevEcDelay, "kPrevEcDelay", msg, "kEcDelay", pubSetEcDelay);
        } else if (!strcmp(topic, subInputPhWtempDelay)) {
            phWtempDelay = savUCharInput(prevPhWtempDelay, "kPrevPhWtDelay", msg, "kPhWtempDelay", pubSetPhWtempDelay);
        }
    } else if (!strcmp(topic, subInputEcMax) || !strcmp(topic, subInputEcMin) ||
               !strcmp(topic, subInputFertOn) || !strcmp(topic, subInputFertOff) ||
               !strcmp(topic, subInputPhOn) || !strcmp(topic, subInputPhOff) ||
               !strcmp(topic, subInputWaterLvMax) || !strcmp(topic, subInputWaterLvMin)) {
        if (!strcmp(topic, subInputEcMax)) {
            ecMax = savUShortInput(prevEcMax, "kPrevEcMax", msg, "kEcMax", pubSetEcMax);
        } else if (!strcmp(topic, subInputEcMin)) {
            ecMin = savUShortInput(prevEcMin, "kPrevEcMin", msg, "kEcMin", pubSetEcMin);
        } else if (!strcmp(topic, subInputFertOn)) {
            fertOn = savUShortInput(prevFertOn, "kPrevFertOn", msg, "kFertOn", pubSetFertOn);
        } else if (!strcmp(topic, subInputFertOff)) {
            fertOff = savUShortInput(prevFertOff, "kPrevFertOff", msg, "kFertOff", pubSetFertOff);
        } else if (!strcmp(topic, subInputPhOn)) {
            phOn = savUShortInput(prevPhOn, "kPrevPhOn", msg, "kPhOn", pubSetPhOn);
        } else if (!strcmp(topic, subInputPhOff)) {
            phOff = savUShortInput(prevPhOff, "kPrevPhOff", msg, "kPhOff", pubSetPhOff);
        } else if (!strcmp(topic, subInputWaterLvMax)) {
            waterLvMax = savUShortInput(prevWaterLvMax, "kPrevWaterLvMax", msg, "kWaterLvMax", pubSetWaterLvMax);
        } else if (!strcmp(topic, subInputWaterLvMin)) {
            waterLvMin = savUShortInput(prevWaterLvMin, "kPrevWaterLvMin", msg, "kWaterLvMin", pubSetWaterLvMin);
        }

    } else {
        if (!strcmp(topic, subInputPhMax)) {
            phMax = savFloatInput(prevPhMax, "kPrevPhMax", msg, "kPhMax", pubSetPhMax);
        } else if (!strcmp(topic, subInputPhMin)) {
            phMin = savFloatInput(prevPhMin, "kPrevPhMin", msg, "kPhMin", pubSetPhMin);
        }
        // else if (!strcmp(topic, subInputWaterTMax)) {
        //     waterTMax = savFloatInput(prevWaterTMax, "kPrevWaterTMax", msg, "kWaterTMax", pubSetWaterTMax);
        // } else if (!strcmp(topic, subInputWaterTMin)) {
        //     waterTMin = savFloatInput(prevWaterTMin, "kPrevWaterTMin", msg, "kWaterTMin", pubSetWaterTMin);
        // }
    }
}

void publishSetting() {
    mqtt.publish(pubSetState, String(state).c_str());
    mqtt.publish(pubSetWaterLvMax, String(waterLvMax).c_str());
    mqtt.publish(pubSetWaterLvMin, String(waterLvMin).c_str());
    mqtt.publish(pubSetEcMax, String(ecMax).c_str());
    mqtt.publish(pubSetEcMin, String(ecMin).c_str());
    mqtt.publish(pubSetEcDelay, String(ecDelay).c_str());
    mqtt.publish(pubSetFertAPumpPct, String(fertAPumpPct).c_str());
    mqtt.publish(pubSetFertBPumpPct, String(fertBPumpPct).c_str());
    mqtt.publish(pubSetFertOn, String(fertOn).c_str());
    mqtt.publish(pubSetFertOff, String(fertOff).c_str());
    mqtt.publish(pubSetPhMax, String(phMax).c_str());
    mqtt.publish(pubSetPhMin, String(phMin).c_str());
    mqtt.publish(pubSetPhWtempDelay, String(phWtempDelay).c_str());
    mqtt.publish(pubSetPhOn, String(phOn).c_str());
    mqtt.publish(pubSetPhOff, String(phOff).c_str());
    // mqtt.publish(pubSetWaterTMax, String(waterTMax).c_str());
    // mqtt.publish(pubSetWaterTMin, String(waterTMin).c_str());
}
void subscribeInput() {
    mqtt.subscribe(subInputState);
    mqtt.subscribe(subInputRestart);
    // mqtt.subscribe(subFetchDataState);
    mqtt.subscribe(subInputWaterLvMax);
    mqtt.subscribe(subInputWaterLvMin);
    mqtt.subscribe(subInputEcMax);
    mqtt.subscribe(subInputEcMin);
    mqtt.subscribe(subInputEcDelay);
    mqtt.subscribe(subInputFertAPumpPct);
    mqtt.subscribe(subInputFertBPumpPct);
    mqtt.subscribe(subInputFertOn);
    mqtt.subscribe(subInputFertOff);
    mqtt.subscribe(subInputPhMax);
    mqtt.subscribe(subInputPhMin);
    mqtt.subscribe(subInputPhWtempDelay);
    mqtt.subscribe(subInputPhOn);
    mqtt.subscribe(subInputPhOff);
    // mqtt.subscribe(subInputWaterTMax);
    // mqtt.subscribe(subInputWaterTMin);
}

void reconnectMqtt() {
    if (WiFi.status() == WL_CONNECTED) {
        de("Connecting MQTT... ");
        if (mqtt.connect(deviceName, mqtt_user, mqtt_pass)) {
            tReconnectMqtt.disable();
            deln("connected");
            tConnectMqttLoop.setInterval(0);
            tConnectMqttLoop.enable();
            mqttConnectedLed.blinkNumberOfTimes(200, 200, 3);  // 250ms ON, 750ms OFF, repeat 10 times, blink immediately
            publishSetting();                                  // Once connected, publish the topics
            deln("Published setting");
            // Publish node in node red, the retain value must be "true".
            subscribeInput();  // Once connected, subscribe to the topics
            deln("Subscribed Input");
        } else {
            deln("failed state: " + String(mqtt.state()));
            if (tReconnectMqtt.getRunCounter() >= 3) {
                // ESP.restart();
                tReconnectMqtt.disable();
                tConnectMqttLoop.setInterval(1 * TASK_HOUR); // Try to reconnect after 1 hour.
                tConnectMqttLoop.enableDelayed(); 
            }
        }
    } else {
        if (tReconnectMqtt.isFirstIteration()) deln("WiFi is not connected");
    }
}

void connectMqttLoop() {
    mqttConnectedLed.loop();  // MUST call the led.loop() function in loop()

    if (!mqtt.connected()) {
        tConnectMqttLoop.disable();
        tReconnectMqtt.enable();
    } else {
        mqtt.loop();
    }
}

//----------------- Modbus --------------------//
void preTransmission() { digitalWrite(reDePin, HIGH); }
void postTransmission() { digitalWrite(reDePin, LOW); }

bool getResultMsg(ModbusMaster *node, uint8_t result) {
    String tmpstr2 = "\r\n";
    switch (result) {
        case node->ku8MBSuccess: return true; break;
        case node->ku8MBIllegalFunction: tmpstr2 += "Illegal Function"; break;
        case node->ku8MBIllegalDataAddress: tmpstr2 += "Illegal Data Address"; break;
        case node->ku8MBIllegalDataValue: tmpstr2 += "Illegal Data Value"; break;
        case node->ku8MBSlaveDeviceFailure: tmpstr2 += "Slave Device Failure"; break;
        case node->ku8MBInvalidSlaveID: tmpstr2 += "Invalid Slave ID"; break;
        case node->ku8MBInvalidFunction: tmpstr2 += "Invalid Function"; break;
        case node->ku8MBResponseTimedOut: tmpstr2 += "Response Timed Out"; break;
        case node->ku8MBInvalidCRC: tmpstr2 += "Invalid CRC"; break;
        default: tmpstr2 += "Unknown error: " + String(result); break;
    }
    Serial.println(tmpstr2);
    return false;
}

// https://github.com/4-20ma/ModbusMaster/issues/109
void getData(Para paras) {
    // Solved Compilation error: jump to case label [-fpermissive]. https://stackoverflow.com/questions/5685471/error-jump-to-case-label-in-switch-statement
    switch (paras) {
        case EC: {
            // uint8_t ecResult = modbus1.readInputRegisters(0, 2);
            uint8_t ecResult = modbus1.readHoldingRegisters(0, 2);
            if (getResultMsg(&modbus1, ecResult)) {
                mEc = modbus1.getResponseBuffer(1) / 10.f;
                // de("EC: " + String(mEc) + " uS/cm");
            }
        } break;
        case PH: {
            uint8_t phResult = modbus2.readHoldingRegisters(0, 2);
            if (getResultMsg(&modbus2, phResult)) {
                mPh = modbus2.getResponseBuffer(1) / 10.f;
                // de(" / pH: " + String(mPh, 1));
            }
        } break;
        case WATERTEMP: {
            uint8_t waterTempResult = modbus2.readHoldingRegisters(0, 2);
            if (getResultMsg(&modbus2, waterTempResult)) {
                mWaterTemp = modbus2.getResponseBuffer(0) / 10.f;
                // de(" / WaterTemp: " + String(mWaterTemp, 1) + " C");
            }
        } break;
        case WATERLEVEL: {
            uint8_t waterLevelResult = modbus3.readHoldingRegisters(0, 5);
            if (getResultMsg(&modbus3, waterLevelResult)) {
                mWaterLv = modbus3.getResponseBuffer(4) /* 10.f*/;
                // Serial.print(" / Water Lv: " + String(waterLv) + " mm");
            }
        } break;
    }
}

//----------------- Fill Water ----------------//
// void waterPumpOn() {
//     digitalWrite(waterPumpPin, turnOn);
//     deln("Water Pump: On");
// }
// void waterPumpOff() {
//     digitalWrite(waterPumpPin, turnOff);
//     deln("Water Pump: Off");
// }

// float soundToDistance() {
//     digitalWrite(trigPin, LOW);
//     delayMicroseconds(2);         // wait for 2 ms to avoid
//     digitalWrite(trigPin, HIGH);  // turn on the Trigger to generate pulse
//     delayMicroseconds(10);        // keep the trigger "ON" for 10 ms to generate
//     digitalWrite(trigPin, LOW);   // Turn off the pulse trigger to stop

//     float duration = pulseIn(echoPin, HIGH, 15000); // (echoPin, HIGH, timeout(us) = 1000)
//     storeDistance.add(duration * 0.0343 / 2.f);
//     return storeDistance.getMedian();
// }

// void measureWater() {
//     // int anlWaterLv = analogRead(ultrasonicPin);
//     // mWaterLv       = mapFloat(anlWaterLv, 0, 4095, 0.0, 100.0);
//     // storeDistance.add(soundToDistance());
//     // distance = storeDistance.getMedian();

//     distance = soundToDistance();
//     deln("Distance: " + String(distance) + " cm");
//     if (distance > (fullTankDst - fullTankGap) && distance < emptyTankDst) {
//         mWaterLv = map(distance, emptyTankDst, fullTankDst, 0.0, 100.0);
//         deln("\tWater Level  : " + String(mWaterLv));
//         tWaterState.restart();
//     } else {
//         deln("Distance is out of specified range.");
//         isMixing = true;
//     }
// }

// void waterState() {
//     deln("Water state started");
//     tWaterState.setCallback(NULL);  // Prevent repeating the tEcState.
//     if (mWaterLv < waterLvMin) {
//         tFillWaterInit.restart();
//     } else {
//         mix.disableAll();
//         tWaterState.setCallback(&waterState);
//         tMeasureEcInit.restart();
//     }
// }

// void fillWaterInit() {
//     tFillWaterInit.setCallback(NULL);
//     waterPumpOn();
//     tFillWater.enable();
//     isMixing = true;
// }

// void fillWater() {
//     if (tFillWater.isFirstIteration()) deln("Fill Water");
//     // if (distance < fullTankDst) {
//     if (mWaterLv > waterLvMax) {
//         waterPumpOff();
//         mix.disableAll();
//         tWaterState.setCallback(&waterState);
//         tFillWaterInit.setCallback(&fillWaterInit);
//         tMeasureEcInit.restart();
//         deln("Water filling is disabled");
//     }
// }

//----------------- EC ------------------------//
void IRAM_ATTR stirringPumpOn() {
    digitalWrite(stirringPumpPin, turnOn);
    deln("Stirring Pump: On");
}
void IRAM_ATTR stirringPumpOff() {
    digitalWrite(stirringPumpPin, turnOff);
    deln("Stirring Pump: Off");
}
void IRAM_ATTR fertA_PumpOn() {
    digitalWrite(cwFertA_PumpPin, LOW);
    digitalWrite(ccwFertA_PumpPin, HIGH);
    float fertA_DutyCycle = 225 / 100.f * fertAPumpPct;
    ledcWrite(pwmChannel0, fertA_DutyCycle);
    deln("Fert A Pump: On");
}
void IRAM_ATTR fertA_PumpOff() {
    digitalWrite(cwFertA_PumpPin, LOW);
    digitalWrite(ccwFertA_PumpPin, LOW);
    deln("Fert A Pump: Off");
}

void IRAM_ATTR fertB_PumpOn() {
    digitalWrite(cwFertB_PumpPin, LOW);
    digitalWrite(ccwFertB_PumpPin, HIGH);
    float fertB_DutyCycle = 225 / 100.f * fertBPumpPct;
    ledcWrite(pwmChannel1, fertB_DutyCycle);
    deln("Fert B Pump: On");
}
void IRAM_ATTR fertB_PumpOff() {
    digitalWrite(cwFertB_PumpPin, LOW);
    digitalWrite(ccwFertB_PumpPin, LOW);
    deln("Fert B Pump: Off");
}

void measureEcInit() {
    deln("EC measurement initialized");
    stirringPumpOn();
    tMeasureEc.enableDelayed(3000);
}

// Linear Equation: https://doi.org/10.1109/ICIC47613.2019.8985851
// float calibrate(float ind, float x1, float y1, float x2, float y2) {
//     return (((ind - x1) / (x2 - x1)) * (y2 - y1)) + y1;
// }

void measureEc() {
#ifdef simulation
    if (digitalRead(stirringPumpPin)) {
#else
    if (!digitalRead(stirringPumpPin)) {  // Stirring pump is ON at active low
#endif

#ifdef simulation
        storeEc.add(analogRead(ecPin));
        medianEcAnl = storeEc.getMedian();
        mEc         = mapFloat(medianEcAnl, 0, 4095, 0.0, 4000.0);
#else
        getData(EC);
#endif
        deln("\tEC Value: " + String(mEc) + " uS/cm");
        tEcState.restart();
    } else {
        deln("Stirring Pump: Unavailable");
    }
}

void ecState() {
    deln("EC state started");
    tEcState.setCallback(NULL);  // Prevent repeating the tEcState
    if (mEc < ecMin) {
        tFillFertInit.restart();
    } else {
        mix.disableAll();
        tMeasureEcInit.setCallback(&measureEcInit);
        tEcState.setCallback(&ecState);  // Return ecState to the tEcState
        // tDetWrapper.enable();
        tDetWrapper.enable();
        // tMeasurePhInit.restart();
        // isMixing = false;
    }
}

void fillFertInit() {
    tFillFertInit.setCallback(NULL);  // Prevent repeating the tFillFertInit
    tFertA_On.restart();
    deln("Fill Fertilizer Initalized");
}

void fertA_On() {
    deln();
    fertA_PumpOn();
    isMixing = true;
    tSendData.enable();
    tFertA_Off.waitForDelayed(tFertA_On.getInternalStatusRequest());
}

void fertA_Off() {
    fertA_PumpOff();
    tFertB_On.waitForDelayed(tFertA_Off.getInternalStatusRequest());
}
void fertB_On() {
    fertB_PumpOn();
    tFertB_Off.waitForDelayed(tFertB_On.getInternalStatusRequest());
}
void fertB_Off() {
    fertB_PumpOff();
    tFertA_On.waitForDelayed(tFertB_Off.getInternalStatusRequest());
}

void fertB_OffDis() {
    tFertA_Off.setInterval(fertOn * 1000);
    deln("Set A ON intv");
    tFertA_On.setInterval(fertOff * 1000);
    deln("Set A OFF intv");
    tFertB_Off.setInterval(fertOn * 1000);
    deln("Set B ON intv");
    tFertB_On.setInterval(fertOff * 1000);
    deln("Set B OFF intv");
    deln("FertB_OffDis is executed.");

    if (mEc > ecMax) {
        tFertA_On.disable();
        deln("\tA&B Pumps: OFF");
        mix.disableAll();
        tEcState.setCallback(&ecState);            // Return ecState to the tEcState
        tFillFertInit.setCallback(&fillFertInit);  // Return fillFertInit to the tFillFertInit
        // tMeasurePhInit.restart();
        deln("Fertilizer filling is disabled");
        // tDetWrapper.enableDelayed();
        onlyAdjustPh ? onlyAdjustPh = false : tDetWrapper.enableDelayed();
        isMixing = false;
    }
}

//----------------- pH ------------------------//
void IRAM_ATTR phDownPumpOn() {
    digitalWrite(phDownPumpPin, turnOn);
    deln("ph Down Pump: On");
}
void IRAM_ATTR phDownPumpOff() {
    digitalWrite(phDownPumpPin, turnOff);
    deln("ph Down Pump: Off");
}
void IRAM_ATTR phUpPumpOn() {
    digitalWrite(phUpPumpPin, turnOn);
    deln("ph Up Pump: On");
}
void IRAM_ATTR phUpPumpOff() {
    digitalWrite(phUpPumpPin, turnOff);
    deln("ph Up Pump: Off");
}
void measurePhInit() {
    deln("pH measure Initialized");
    tMeasurePhInit.setCallback(NULL);
    if (digitalRead(stirringPumpPin) == turnOff) stirringPumpOn();
    tMeasurePh.enableDelayed(3000);
}

void measurePh() {
#ifdef simulation
    if (digitalRead(stirringPumpPin)) {
#else
    // if (digitalRead(stirringPumpPin) == LOW) {  // Stirring pump is ON at active low
    if (!digitalRead(stirringPumpPin)) {  // Stirring pump is ON at active low
#endif

#ifdef simulation
        storePh.add(analogRead(phPin));
        medianPhAnl = storePh.getMedian();
        mPh         = mapFloat(medianPhAnl, 0, 4095, 4.5, 7.5);
#else
        getData(PH);
        getData(WATERTEMP);
#endif
        // deln("\tpH Value: " + String(mPh));
        deln("pH: " + String(mPh) + " / " + String(mWaterTemp) + "C");

    } else {
        deln("Stirring Pump: Unavailable");
    }
    tPhState.restart();
}

void phState() {
    tPhState.setCallback(NULL);  // Prevents repeated phState calls.

    if (mPh > phMax) {
        tPhDownPumpOn.restart();
        // deln("\tphDown started");
    } else if (mPh < phMin) {
        tPhUpPumpOn.restart();
        // deln("\tphUP started");
    } else {
        deln("\tThe optimal pH range");
        stirringPumpOff();
        mix.disableAll();
        tMeasurePhInit.setCallback(&measurePhInit);  // Return measurePhInit to the tMeasurePhInit
        tPhState.setCallback(&phState);              // Return phState to the tPhState
        // tDetWrapper.enableDelayed();
        onlyAdjustPh ? onlyAdjustPh = false : tDetWrapper.enableDelayed();
        isMixing = false;
    }
}
void phDownPumpOnCb() {
    deln("\tphDown started");
    tPhDownPumpOn.setCallback(NULL);  // Prevent repeated phDownPumpOnCb calls.
    tPhDownOn.restart();
}

void phDownOn() {
    phDownPumpOn();
    isMixing = true;
    tPhDownOff.waitForDelayed(tPhDownOn.getInternalStatusRequest());
}
void phDownOff() {
    phDownPumpOff();
    tPhDownOn.waitForDelayed(tPhDownOff.getInternalStatusRequest());
}
void phDownOffDis() {
    tPhDownOn.setInterval(phOff * 1000);
    tPhDownOff.setInterval(phOn * 1000);

    float avgPh = (phMax + phMin) / 2.f;
    deln("\t\tAdjusting less than " + String(avgPh));
    if (mPh < avgPh) {
        tPhDownOn.disable();
        deln("\tpH Dowm Pumps: OFF");
        mix.disableAll();                            // Make sure the all tasks is disabled.
        tMeasurePhInit.setCallback(&measurePhInit);  // Return measurePhInit to the tMeasurePhInit
        tPhState.setCallback(&phState);              // Return phState to the tPhState
        tPhDownPumpOn.setCallback(&phDownPumpOnCb);  // Return phDownPumpOnCb to the tPhDownPumpOn
        tStirringPumpOff.restartDelayed();
        // tDetWrapper.enableDelayed();
        onlyAdjustPh ? onlyAdjustPh = false : tDetWrapper.enableDelayed();
        isMixing = false;
    }
    // deln("phDownOffDis is executed.");
}

void phUpPumpOnCb() {
    deln("\tphUP started");
    tPhUpPumpOn.setCallback(NULL);
    tPhUpOn.restart();
}
void phUpOn() {
    phUpPumpOn();
    isMixing = true;
    tPhUpOff.waitForDelayed(tPhUpOn.getInternalStatusRequest());
}
void phUpOff() {
    phUpPumpOff();
    tPhUpOn.waitForDelayed(tPhUpOff.getInternalStatusRequest());
}
void phUpOffDis() {
    tPhUpOn.setInterval(phOff * 1000);
    tPhUpOff.setInterval(phOn * 1000);

    float avgPh = (phMax + phMin) / 2.f;
    deln("\t\tAdjusting more than " + String(avgPh));
    if (mPh > avgPh) {
        tPhUpOn.disable();
        deln("\tpH Up Pumps: OFF");
        mix.disableAll();                            // Make sure the all tasks is disabled.
        tMeasurePhInit.setCallback(&measurePhInit);  // Return measurePhInit to the tMeasurePhInit
        tPhState.setCallback(&phState);              // Return phState to the tPhState
        tPhUpPumpOn.setCallback(&phUpPumpOnCb);      // Return phUpPumpOnCb to the tPhUpPumpOn
        tStirringPumpOff.restartDelayed();
        // tDetWrapper.enableDelayed();
        tDetWrapper.enableDelayed();
        isMixing = false;
    }
    // deln("phUpOffDis is executed.");
}

void IRAM_ATTR oxygenPumpOn() {
    digitalWrite(oxygenPumpPin, turnOn);
    deln("Oxygen Pump: On");
}
void IRAM_ATTR oxygenPumpOff() {
    digitalWrite(oxygenPumpPin, turnOff);
    deln("Oxygen Pump: Off");
}

//----------------- Detection -----------------//
// void detectWrapper2();
// void getEc();
// void getPhWtemp();
// void decideMixing();
// Task tDetectWrapper2(5000, TASK_FOREVER, &detectWrapper2, &menMqt, false);
// Task tGetEc(0, TASK_ONCE, &getEc, &menMqt, false);
// Task tGetPhWtemp(0, TASK_ONCE, &getPhWtemp, &menMqt, false);
// Task tDecideMixing(0, TASK_ONCE, &decideMixing, &menMqt, false);

void getEc() {
    getData(EC);
    deln("EC: " + String(mEc) + " uS/cm");
}
// void getEc() {
//     getData(EC);
//     deln("EC: " + String(mEc) + " uS/cm");
//     sendData();
//     stirringPumpOff();
//     tDecideMixing.restart();
// }

void getPhWtemp() {
    getData(PH);
    getData(WATERTEMP);
    deln("pH: " + String(mPh) + " | WaterTemp: " + String(mWaterTemp) + " C");
    sendData();
    stirringPumpOff();
    tDecideMixing.restart();
}

// void decideMixing() {
//     deln("\nDecide Mixing");
//     if (mWaterLv > waterLvMin) {
//         if (mEc < ecMin) {
//             tDetWrapper.disable();
//             mix.disableAll();
//             deln("EC is lower than (Min) " + String(ecMin) + " uS/cm");
//             tMeasureEcInit.restart();
//         } else if (mPh < phMin || mPh > phMax) {
//             tDetWrapper.disable();
//             mix.disableAll();
//             deln("pH is lower than (Min) " + String(phMin) + " or higher than (Max) " + String(phMax));
//             tMeasurePhInit.restart();
//         } else {
//             deln("Everything is OK. The system will detect them next time.");
//         }
//     }
// }
void decideMixing() {
    deln("\nDecide Mixing");
    if (mEc < ecMin) {
        tDetWrapper.disable();
        mix.disableAll();
        deln("EC is lower than (Min) " + String(ecMin) + " uS/cm");
        tMeasureEcInit.restart();
    } else if (mPh < phMin || mPh > phMax) {
        tDetWrapper.disable();
        mix.disableAll();
        deln("pH is lower than (Min) " + String(phMin) + " or higher than (Max) " + String(phMax));
        tMeasurePhInit.restart();
    } else {
        deln("Everything is OK. The system will detect them next time.");
    }
}

void detWrapper() {
    deln("\nDetection Wrapper2");
    isMixing = false;
    getData(WATERLEVEL);
    mqtt.publish(pubMonitorWaterLv, String(mWaterLv).c_str());
    deln("Water Level: " + String(mWaterLv) + " mm");

    if (mWaterLv < waterLvMin) {
        deln("Water Level is " + String(mWaterLv) + " lower than (Min) " + String(waterLvMin));
    } else {
        stirringPumpOn();
        tGetEc.restartDelayed(ecDelay * 1000);
        tGetPhWtemp.restartDelayed(phWtempDelay * 1000);
    }
}

void onlyAdjustPhOn() {
    if (onlyAdjustPh) {
        tOnlyAdjustPh.setCallback(&onlyAdjustPhOff);
        state = 0;
        if (mWaterLv < waterLvMin) {
            deln("Water Level is " + String(mWaterLv) + " lower than (Min) " + String(waterLvMin));
        } else {
            deln("\n Only adjust the pH");
            tMeasurePhInit.restart();
        }
    }
}
void onlyAdjustPhOff() {
    if (!onlyAdjustPh) {
        tOnlyAdjustPh.setCallback(&onlyAdjustPhOn);
        phUpPumpOff();
        phDownPumpOff();
        stirringPumpOff();
    }
}

void onlyAdjustEcOn() {
    if (onlyAdjustEc) {
        tOnlyAdjustEc.setCallback(&onlyAdjustEcOff);
        state = 0;
        if (mWaterLv < waterLvMin) {
            deln("Water Level is " + String(mWaterLv) + " lower than (Min) " + String(waterLvMin));
        } else {
            deln("\n Only adjust the EC.");
            tMeasureEcInit.restart();
        }
    }
}
void onlyAdjustEcOff() {
    if (!onlyAdjustEc) {
        tOnlyAdjustEc.setCallback(&onlyAdjustEcOn);
        fertA_PumpOff();
        fertB_PumpOff();
        stirringPumpOff();
    }
}

//----------------- Read Data -----------------//
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// void readData() {
//     mAirTemp = dht.readTemperature();
//     mHumi    = dht.readHumidity();
//     DS18B20.requestTemperatures();
//     mWaterTemp = DS18B20.getTempCByIndex(0);

//     deln("Air Temp= " + String(mAirTemp) + " C");
//     deln("Humidity= " + String(mHumi) + " %");
//     deln("Water Temp= " + String(mWaterTemp) + " C");
// }

//----------------- Send Data -----------------//
void sendData() {
    if (mqtt.connected()) {
        isMixing ? tSendData.setInterval(5000) : tSendData.setInterval(15 * TASK_MINUTE);  // Send data intval base on isMixing.

        // readData();
        mqtt.publish(pubMonitorWaterLv, String(mWaterLv).c_str());
        mqtt.publish(pubMonitorEc, String(mEc).c_str());
        mqtt.publish(pubMonitorPh, String(mPh).c_str());
        mqtt.publish(pubMonitorWaterTemp, String(mWaterTemp).c_str());
        // mqtt.publish(pubMonitorAirTemp, String(mAirTemp).c_str());
        // mqtt.publish(pubMonitorHumi, String(mHumi).c_str());

        deln("\t\tData sending is done.");
    } else {
        deln("\t\tData sending is failed. Check MQTT connection.");
    }
}

void stateOn() {
    if (state) {
        tState.setCallback(&stateOff);
        // tDetWrapper.enable();
        tDetWrapper.enable();
        // isMixing = false;
        tSendData.enableDelayed();
    }
}
void stateOff() {
    if (!state) {
        tState.setCallback(&stateOn);
        mix.disableAll();
        tDetWrapper.disable();
        // waterPumpOff();
        stirringPumpOff();
        fertA_PumpOff();
        fertB_PumpOff();
        phDownPumpOff();
        phUpPumpOff();
        tSendData.disable();
        tEcState.setCallback(&ecState);
        // tFertA_On.setCallback(&fertA_On);
        // tFertB_On.setCallback(&fertB_On);
        tPhState.setCallback(&phState);
    }
}

//----------------- Calibration Mode ----------//
void calibrationOn() {
    if (calibration) {
        tCalibration.setCallback(&calibrationOff);
        fertAPumpCalib   = false;
        fertBPumpCalib   = false;
        phUpPumpTest20   = false;
        phDownPumpTest20 = false;
        tFertAPumpCalibMode.enable();
        tFertBPumpCalibMode.enable();
        tPhUpPumpTest20.enable();
        tPhDownPumpTest20.enable();
    }
}

void calibrationOff() {
    if (!calibration) {
        tCalibration.setCallback(&calibrationOn);
        fertAPumpCalib   = false;
        fertBPumpCalib   = false;
        phUpPumpTest20   = false;
        phDownPumpTest20 = false;
        calib.disableAll();
        fertA_PumpOff();
        fertB_PumpOff();
        phUpPumpOff();
        phDownPumpOff();
    }
}
// How do i change global boolean inside a function? https://stackoverflow.com/questions/63052027/how-do-i-change-global-boolean-inside-a-function
void pumpTest(Task *tMode, Task *tCb, void (*on)(), void (*off)(), bool *pumpState) {
    if (tCb->isFirstIteration()) {
        (*on)();
        // deln("fertB_PumpOn");
    }
    if (tCb->isLastIteration()) {
        (*off)();
        *pumpState = false;
        // Serial.println(fertBPumpCalib ? "true" : "false");
        tMode->enable();
        // deln("fertB_PumpOff");
    }
}

void fertAPumpCalibMode() {
    if (fertAPumpCalib) {
        tFertAPumpCalibMode.disable();
        state = 0;  // Turn off if Mixer is running.
        savState();
        tFertAPumpCalib.restart();
    }
}
void fertAPumpCalibCb() {
    // if (tFertAPumpCalib.isFirstIteration()) {
    //     fertA_PumpOn();
    //     // deln("fertA_PumpOn");
    // }
    // if (tFertAPumpCalib.isLastIteration()) {
    //     fertA_PumpOff();
    //     fertAPumpCalib = false;
    //     tFertAPumpCalibMode.enable();
    //     // deln("fertB_PumpOff");
    // }
    pumpTest(&tFertAPumpCalibMode, &tFertAPumpCalib, fertA_PumpOn, fertA_PumpOff, &fertAPumpCalib);
}

void fertBPumpCalibMode() {
    if (fertBPumpCalib) {
        tFertBPumpCalibMode.disable();
        state = 0;  // Turn off if Mixer is running.
        savState();
        tFertBPumpCalib.restart();
    }
}

void fertBPumpCalibCb() {
    // if (tFertBPumpCalib.isFirstIteration()) {
    //     fertB_PumpOn();
    //     // deln("fertB_PumpOn");
    // }
    // if (tFertBPumpCalib.isLastIteration()) {
    //     fertB_PumpOff();
    //     fertBPumpCalib = false;
    //     tFertBPumpCalibMode.enable();
    //     // deln("fertB_PumpOff");
    // }
    pumpTest(&tFertBPumpCalibMode, &tFertBPumpCalib, fertB_PumpOn, fertB_PumpOff, &fertBPumpCalib);
}

void deviceTestOn() {
    if (deviceTest) {
        tDeviceTest.setCallback(&deviceTestOff);
        state = 0;
        savState();
        ecTest           = false;
        fertA_PumpTest   = false;
        fertB_PumpTest   = false;
        phUpPumpTest     = false;
        phDownPumpTest   = false;
        stirringPumpTest = false;
        oxygenPumpTest   = false;
        phWtempTest      = false;
        waterLvTest      = false;
        tFertA_PumpTest.enable();
        tFertB_PumpTest.enable();
        tPhUpPumpTest.enable();
        tPhDownPumpTest.enable();
        tStirringPumpTest.enable();
        tOxygenPumpTest.enable();
        tEcTestMode.enable();
        tPhWtempTestMode.enable();
        tWaterLvTestMode.enable();
        deln("Test Mode: ON");
    }
}
void deviceTestOff() {
    if (!deviceTest) {
        tDeviceTest.setCallback(&deviceTestOn);
        ecTest           = false;
        fertA_PumpTest   = false;
        fertB_PumpTest   = false;
        phUpPumpTest     = false;
        phDownPumpTest   = false;
        stirringPumpTest = false;
        oxygenPumpTest   = false;
        phWtempTest      = false;
        waterLvTest      = false;
        test.disableAll();
        fertA_PumpOff();
        fertB_PumpOff();
        phUpPumpOff();
        phDownPumpOff();
        stirringPumpOff();
        oxygenPumpOff();
        deln("Test Mode: OFF");
    }
}

void ecTestModeOn() {
    if (ecTest) {
        tEcTestMode.setCallback(&ecTestModeOff);
        if (phWtempTest) phWtempTest = false;  // Prevent overlaping work between phWtempTestModeOn and ecTestModeOn
        tMeasEcInTestMode.enable();
    }
}
void measEcInTestMode() {
#ifdef simulation
    storeEc.add(analogRead(ecPin));
    medianEcAnl = storeEc.gegtMedian();
    de("medianEcAnl: " + String(medianEcAnl));
#else
    getData(EC);
#endif
    deln("EC: " + String(mEc) + " uS/cm");
}
void ecTestModeOff() {
    if (!ecTest) {
        tEcTestMode.setCallback(&ecTestModeOn);
        tMeasEcInTestMode.disable();
    }
}

void testOn(bool aBool, Task *aTask, void (*aCallback)(), void (*on)()) {
    if (aBool) {
        aTask->setCallback(aCallback);
        (*on)();
    }
}
void testOff(bool aBool, Task *aTask, void (*aCallback)(), void (*off)()) {
    if (!aBool) {
        aTask->setCallback(aCallback);
        (*off)();
    }
}

void fertA_PumpTestOn() { testOn(fertA_PumpTest, &tFertA_PumpTest, &fertA_PumpTestOff, fertA_PumpOn); }
void fertA_PumpTestOff() { testOff(fertA_PumpTest, &tFertA_PumpTest, &fertA_PumpTestOn, fertA_PumpOff); }

void fertB_PumpTestOn() { testOn(fertB_PumpTest, &tFertB_PumpTest, &fertB_PumpTestOff, fertB_PumpOn); }
void fertB_PumpTestOff() { testOff(fertB_PumpTest, &tFertB_PumpTest, &fertB_PumpTestOn, fertB_PumpOff); }

void phUpPumpTestOn() { testOn(phUpPumpTest, &tPhUpPumpTest, &phUpPumpTestOff, phUpPumpOn); }
void phUpPumpTestOff() { testOff(phUpPumpTest, &tPhUpPumpTest, &phUpPumpTestOn, phUpPumpOff); }

void phDownPumpTestOn() { testOn(phDownPumpTest, &tPhDownPumpTest, &phDownPumpTestOff, phDownPumpOn); }
void phDownPumpTestOff() { testOff(phDownPumpTest, &tPhDownPumpTest, &phDownPumpTestOn, phDownPumpOff); }

void phUpPumpTest20s() {
    if (phUpPumpTest20) {
        tPhUpPumpTest20.disable();
        tPhUpPumpTest20Cb.restart();
    }
}
void phUpPumpTest20Cb() { pumpTest(&tPhUpPumpTest20, &tPhUpPumpTest20Cb, phUpPumpOn, phUpPumpOff, &phUpPumpTest20); }

void phDownPumpTest20s() {
    if (phDownPumpTest20) {
        tPhDownPumpTest20.disable();
        tPhDownPumpTest20Cb.restart();
    }
}
void phDownPumpTest20Cb() { pumpTest(&tPhDownPumpTest20, &tPhDownPumpTest20Cb, phDownPumpOn, phDownPumpOff, &phDownPumpTest20); }

void stirringPumpTestOn() { testOn(stirringPumpTest, &tStirringPumpTest, &stirringPumpTestOff, stirringPumpOn); }
void stirringPumpTestOff() { testOff(stirringPumpTest, &tStirringPumpTest, &stirringPumpTestOn, stirringPumpOff); }

void oxygenPumpTestOn() { testOn(oxygenPumpTest, &tOxygenPumpTest, &oxygenPumpTestOff, oxygenPumpOn); }
void oxygenPumpTestOff() { testOff(oxygenPumpTest, &tOxygenPumpTest, &oxygenPumpTestOn, oxygenPumpOff); }

void phWtempTestModeOn() {
    if (phWtempTest) {
        tPhWtempTestMode.setCallback(&phWtempTestModeOff);
        if (ecTest) ecTest = false;  // Prevent overlaping work between phWtempTestModeOn and ecTestModeOn
        tMeasPhWtempInTestMode.enable();
    }
}
void measPhWtempInTestMode() {
#ifdef simulation
    storePh.add(analogRead(phPin));
    medianPhAnl = storePh.getMedian();
    de("medianPhAnl: " + String(medianPhAnl));
#else
    getData(PH);
    getData(WATERTEMP);
#endif
    deln("pH: " + String(mPh, 1) + " / " + String(mWaterTemp, 1) + " C");
}
void phWtempTestModeOff() {
    if (!phWtempTest) {
        tPhWtempTestMode.setCallback(&phWtempTestModeOn);
        tMeasPhWtempInTestMode.disable();
    }
}

// **************************************
void waterLvTestModeOn() {
    if (waterLvTest) {
        tWaterLvTestMode.setCallback(&waterLvTestModeOff);
        tMeasWaterLvInTestMode.enable();
    }
}
void measWaterLvInTestMode() {
    getData(WATERLEVEL);
    deln("WaterLv: " + String(mWaterLv) + " mm");
}
void waterLvTestModeOff() {
    if (!waterLvTest) {
        tWaterLvTestMode.setCallback(&waterLvTestModeOn);
        tMeasWaterLvInTestMode.disable();
        // tState.enable();
    }
}

// *****************************************

// bool fetchDataEn() {
//     deln("EC measurement initialized");
//     stirringPumpOn();
//     return true;
// }

// void fetchData() {
//     // Read Sensors
//     readData();
//     readWaterLv();
//     readEcPh(mEc, ecPin, storeFetchEc, ecMeas_1, ecStd_1, ecMeas_2, ecStd_2);  // EC
//     readEcPh(mPh, phPin, storeFetchPh, phMeas_1, phStd_1, phMeas_2, phStd_2);  // pH

//     // Send Data
//     if (mqtt.connected()) {
//         mqtt.publish(pubMonitorWaterLv, String(mWaterLv).c_str());
//         mqtt.publish(pubMonitorEc, String(mEc).c_str());
//         mqtt.publish(pubMonitorPh, String(mPh).c_str());
//         mqtt.publish(pubMonitorAirTemp, String(mAirTemp).c_str());
//         mqtt.publish(pubMonitorHumi, String(mHumi).c_str());
//         mqtt.publish(pubMonitorWaterTemp, String(mWaterTemp).c_str());
//         deln("Data sending is done.");
//     } else {
//         deln("Data sending is failed. Check MQTT connection.");
//     }

//     tFetchStirringPumpOff.restartDelayed();
// }

// void fetchDataStateCb() {
//     if (!state) {
//         if (fetchDataState) {
//             tFetchDataState.disable();
//             tFetchData.restartDelayed();
//         }
//     } else {
//         if (fetchDataState) {
//             fetchDataState = 0;
//             deln("\tTurn the status off before fetching data");
//         }
//     }
// }

//-------------------------------- Setup ------------------------------------//
void setup() {
    // pf.begin("memory", false);
    getValues();
    // Serial.begin(115200);
    pinMode(resetPin, INPUT_PULLUP);
    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, LOW);
    // pinMode(waterPumpPin, OUTPUT);
    pinMode(stirringPumpPin, OUTPUT);
    digitalWrite(stirringPumpPin, turnOff);
    pinMode(oxygenPumpPin, OUTPUT);
    digitalWrite(oxygenPumpPin, turnOff);

#ifdef simulation
    pinMode(ecPin, INPUT);
    pinMode(phPin, INPUT);
#endif

    pinMode(reDePin, OUTPUT);
    digitalWrite(reDePin, LOW);

    pinMode(cwFertA_PumpPin, OUTPUT);
    pinMode(ccwFertA_PumpPin, OUTPUT);
    pinMode(enableFertA_PumpPin, OUTPUT);
    pinMode(cwFertB_PumpPin, OUTPUT);
    pinMode(ccwFertB_PumpPin, OUTPUT);
    pinMode(enableFertB_PumpPin, OUTPUT);

    // configure LED PWM functionalitites
    ledcSetup(pwmChannel0, frequency, resolution);
    ledcSetup(pwmChannel1, frequency, resolution);

    // attach the channel to the GPIO to be controlled
    ledcAttachPin(enableFertB_PumpPin, pwmChannel1);
    ledcAttachPin(enableFertA_PumpPin, pwmChannel0);

    pinMode(phUpPumpPin, OUTPUT);
    pinMode(phDownPumpPin, OUTPUT);
    digitalWrite(phUpPumpPin, turnOff);
    digitalWrite(phDownPumpPin, turnOff);

    Serial.begin(115200);
    Serial2.begin(baud, SERIAL_8N1, rxPin, txPin);  // IOXESP32 Modbus RTU shield (Lite)
    modbus1.begin(slaveId1, Serial2);
    modbus1.preTransmission(preTransmission);
    modbus1.postTransmission(postTransmission);
    modbus2.begin(slaveId2, Serial2);
    modbus2.preTransmission(preTransmission);
    modbus2.postTransmission(postTransmission);
    modbus3.begin(slaveId3, Serial2);
    modbus3.preTransmission(preTransmission);
    modbus3.postTransmission(postTransmission);

    // setup rotary encoder
    clickEncoder.setAccelerationEnabled(true);
    // ESP32 timer
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, 1000, true);
    timerAlarmEnable(timer);

    Wire.begin();
    u8g2.setI2CAddress(0x3F * 2);
    u8g2.begin();
    u8g2.setFont(fontName);

    nav.showTitle = false;  // Disable the title bar

    // dht.begin();

    // tMenuLoop.enable();
    wifiManagerSetup();
    otaWebUpdateSetup();

    mqtt.setServer(mqtt_server, atoi(mqtt_port));
    mqtt.setCallback(mqttCallback);
    tConnectMqttLoop.enable();

    tMenuLoop.enable();
    tWifiResetting.enable();  // Monitor the WiFi resetting button

    tState.enable();
    tOnlyAdjustEc.enable();
    tOnlyAdjustPh.enable();
    tCalibration.enable();
    tDeviceTest.enable();
    // tFetchDataState.enable();

    // Must be at the end of your setup
    // gpio_viewer.setSamplingInterval(25);  // You can set the sampling interval in ms, if not set default is 100ms
    gpio_viewer.begin();
}

//******************************** Loop *************************************//
void loop() {
    server.handleClient();  // OTA Web Update
    menMqt.execute();
    calib.execute();
    test.execute();
    mix.execute();
    // fetch.execute();
}
