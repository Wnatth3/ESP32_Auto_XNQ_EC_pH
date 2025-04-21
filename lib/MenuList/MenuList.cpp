#include "Arduino.h"
#include "MenuList.h"

// void onTimer() { clickEncoder.service(); }  // ESP32 timer, Start the timer to read the clickEncoder every 1 ms

//-----Custom floatField---------------
// void savUChar(const char *kName, uint8_t val, const char *pubTopic) {
//     pf.putUChar(kName, val);
//     mqtt.publish(pubTopic, String(val).c_str());
// }
// void savUShort(const char *kName, uint16_t val, const char *pubTopic) {
//     pf.putUShort(kName, val);
//     mqtt.publish(pubTopic, String(val).c_str());
// }
// void savFloat(const char *kName, float val, const char *pubTopic) {
//     pf.putFloat(kName, val);
//     mqtt.publish(pubTopic, String(val).c_str());
// }

// void savState() { savUChar("kState", state, pubSetState); }
// void savWaterLvMax() { savUShort("kWaterLvMax", waterLvMax, pubSetWaterLvMax); }
// void savWaterLvMin() { savUShort("kWaterLvMin", waterLvMin, pubSetWaterLvMin); }
// void savEcMax() { savUShort("kEcMax", ecMax, pubSetEcMax); }
// void savEcMin() { savUShort("kEcMin", ecMin, pubSetEcMin); }
// void savFertAPumpPct() { savUChar("kFertAPumpPct", fertAPumpPct, pubSetFertAPumpPct); }
// void savFertBPumpPct() { savUChar("kFertBPumpPct", fertBPumpPct, pubSetFertBPumpPct); }
// void savFertOn() { savUShort("kFertOn", fertOn, pubSetFertOn); }
// void savFertOff() { savUShort("kFertOff", fertOff, pubSetFertOff); }
// void savPhMax() { savFloat("kPhMax", phMax, pubSetPhMax); }
// void savPhMin() { savFloat("kPhMin", phMin, pubSetPhMin); }
// void savPhOn() { savUShort("kPhOn", phOn, pubSetPhOn); }
// void savPhOff() { savUShort("kPhOff", phOff, pubSetPhOff); }


// void espRestart() { ESP.restart(); }