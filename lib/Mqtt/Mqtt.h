#ifndef Mqtt_h
#define Mqtt_h

#include "Arduino.h"
#include "Header.h"

// // MQTT
// char mqtt_server[16] = "192.168.0.10";
// char mqtt_port[6]    = "1883";
// char mqtt_user[15]   = "pi";
// char mqtt_pass[15]   = "Berry1911";

int savIntInput(int prev, const char *kPrev, String msg, const char *kVal, const char *pubTopic);
uint8_t savUCharInput(uint8_t prev, const char *kPrev, String msg, const char *kVal, const char *pubTopic);
uint16_t savUShortInput(uint16_t prev, const char *kPrev, String msg, const char *kVal, const char *pubTopic);
float savFloatInput(float prev, const char *kPrev, String msg, const char *kVal, const char *pubTopic);

void espRestart();
void mqttCallback(char *topic, byte *message, unsigned int length);
void publishSetting();
void subscribeInput();

//----------------- MQTT ----------------------//
void connectMqttLoop();
void reconnectMqtt();
Task tConnectMqttLoop(0, TASK_FOREVER, &connectMqttLoop, &menMqt, false);
Task tReconnectMqtt(5000, TASK_FOREVER, &reconnectMqtt, &menMqt, false);

#endif