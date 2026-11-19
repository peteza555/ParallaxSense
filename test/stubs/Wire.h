#pragma once
#include "Arduino.h"
struct TwoWire { bool begin(int,int,uint32_t); void beginTransmission(int); size_t write(uint8_t); uint8_t endTransmission(bool=true); uint8_t requestFrom(int,int); int read(); };
extern TwoWire Wire;
