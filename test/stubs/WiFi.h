#pragma once
#include "Arduino.h"
#define WIFI_STA 1
#define WL_CONNECTED 3
struct IP { String toString() const; };
struct WiFiClass { void mode(int); bool disconnect(); String macAddress(); uint8_t* macAddress(uint8_t*); int scanNetworks(bool,bool); String SSID(int); int32_t channel(int); int32_t channel(); void scanDelete();
 void setSleep(bool); void setAutoReconnect(bool); void begin(const char*,const char*); int status(); IP localIP(); int8_t RSSI(); };
extern WiFiClass WiFi;
class WiFiClient {};
