#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include <math.h>
#include <string>
#include <algorithm>
using std::min; using std::max;
#define RAD_TO_DEG 57.295779513082320876798154814105
#define OUTPUT 1
#define INPUT 0
#define LOW 0
#define HIGH 1
typedef int esp_err_t;
#define ESP_OK 0
uint32_t millis(); uint32_t micros(); void delay(uint32_t); void delayMicroseconds(uint32_t);
void pinMode(int,int); void digitalWrite(int,int); unsigned long pulseIn(int,int,unsigned long);
class String { public: std::string v; String(const char*s=""):v(s){} const char* c_str() const {return v.c_str();} bool operator==(const char*o) const {return v==o;} String toString() const {return *this;} };
struct HwSerial { void begin(int); int printf(const char*, ...) __attribute__((format(printf,2,3))); void print(const char*); void println(const char*); };
extern HwSerial Serial;
struct EspClass { void restart(); }; extern EspClass ESP;
typedef void* QueueHandle_t; typedef int BaseType_t;
#define pdTRUE 1
QueueHandle_t xQueueCreate(int,int); BaseType_t xQueueSend(QueueHandle_t,const void*,int); BaseType_t xQueueReceive(QueueHandle_t,void*,int);
uint32_t analogReadMilliVolts(uint8_t); int digitalRead(uint8_t);
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define IRAM_ATTR
#define INPUT_PULLDOWN 9
#define CHANGE 3
#define RISING 1
#define digitalPinToInterrupt(p) (p)
void attachInterrupt(uint8_t, void (*)(void), int);
uint32_t ledcSetup(uint8_t, uint32_t, uint8_t); void ledcAttachPin(uint8_t, uint8_t); void ledcWrite(uint8_t, uint32_t);
