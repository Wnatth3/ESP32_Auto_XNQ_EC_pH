#include "Arduino.h"
#include "Mqtt.h"

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
uint8_t savUCharInput(uint8_t prev, const char *kPrev, String msg, const char *kVal, const char *pubTopic) {
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

void espRestart() { ESP.restart(); }

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
    } else if (!strcmp(topic, subInputFertAPumpPct) || !strcmp(topic, subInputFertBPumpPct)) {
        if (!strcmp(topic, subInputFertAPumpPct)) {
            fertOn = savUCharInput(prevFertAPumpPct, "kPrevAPumpPct", msg, "kFertAPumpPct", pubSetFertAPumpPct);
        } else if (!strcmp(topic, subInputFertBPumpPct)) {
            fertOff = savUCharInput(prevFertBPumpPct, "kPrevBPumpPct", msg, "kFertBPumpPct", pubSetFertBPumpPct);
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
    mqtt.publish(pubSetFertAPumpPct, String(fertAPumpPct).c_str());
    mqtt.publish(pubSetFertBPumpPct, String(fertBPumpPct).c_str());
    mqtt.publish(pubSetFertOn, String(fertOn).c_str());
    mqtt.publish(pubSetFertOff, String(fertOff).c_str());
    mqtt.publish(pubSetPhMax, String(phMax).c_str());
    mqtt.publish(pubSetPhMin, String(phMin).c_str());
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
    mqtt.subscribe(subInputFertAPumpPct);
    mqtt.subscribe(subInputFertBPumpPct);
    mqtt.subscribe(subInputFertOn);
    mqtt.subscribe(subInputFertOff);
    mqtt.subscribe(subInputPhMax);
    mqtt.subscribe(subInputPhMin);
    mqtt.subscribe(subInputPhOn);
    mqtt.subscribe(subInputPhOff);
    // mqtt.subscribe(subInputWaterTMax);
    // mqtt.subscribe(subInputWaterTMin);
}
void reconnectMqtt() {
    if (WiFi.status() == WL_CONNECTED) {
        de("Connecting MQTT... ");
        if (mqtt.connect("ESP32", mqtt_user, mqtt_pass)) {
            tReconnectMqtt.disable();
            deln("connected");
            tConnectMqttLoop.enable();
            mqttConnectedLed.blinkNumberOfTimes(200, 200, 3);  // 250ms ON, 750ms OFF, repeat 10 times, blink immediately
            // publishSetting();                                  // Once connected, publish the topics
            deln("Published setting");
            // Publish node in node red, the retain value must be "true".
            // subscribeInput();  // Once connected, subscribe to the topics
            deln("Subscribe Input");
        } else {
            deln("failed");
        }

        if (tReconnectMqtt.getRunCounter() > 10) ESP.restart();
    } else {
        deln("WiFi is not connected");
    }
}

void connectMqttLoop() {
    // mqttConnectedLed.loop();  // MUST call the led.loop() function in loop()

    if (!mqtt.connected()) {
        tConnectMqttLoop.disable();
        tReconnectMqtt.enable();
    } else {
        mqtt.loop();
    }
}