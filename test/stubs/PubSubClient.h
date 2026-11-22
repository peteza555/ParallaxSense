#pragma once
#include "WiFi.h"
typedef void (*MqttCallback)(char*, uint8_t*, unsigned int);
struct PubSubClient { PubSubClient(WiFiClient&); void setServer(const char*,uint16_t); bool setBufferSize(uint16_t); void setSocketTimeout(uint16_t); bool connect(const char*,const char*,const char*); bool connected(); int state(); bool loop(); bool publish(const char*,const char*);
 PubSubClient& setCallback(MqttCallback); bool subscribe(const char*); };
