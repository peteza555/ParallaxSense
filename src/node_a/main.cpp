// ParallaxSense — Sensor Node A (สวมที่คอ/ศีรษะ)
// MPU6050 (I2C) + เซนเซอร์แสง 3 ขา (analog/digital) -> ESP-NOW -> Gateway
//
// ต่อสาย:
//   MPU6050 : VCC -> 3V3, GND -> GND, SDA -> GPIO21, SCL -> GPIO22, AD0 -> GND (address 0x68)
//   แสง 3 ขา: VCC(+) -> 3V3 (ห้าม 5V), GND(-) -> GND, ขาสัญญาณ -> LIGHT_PIN (ค่าเริ่มต้นใน platformio.ini = D34 analog)
//   ตั้งค่าเซนเซอร์แสงได้ที่ platformio.ini [env:node_a] build_flags

#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_idf_version.h>
#include <math.h>
#include "packet.h"
#if __has_include("secrets.h")
#include "secrets.h"
#else
#error "Missing include/secrets.h -> copy include/secrets.h.example to include/secrets.h and fill in real values"
#endif

// ---------------- Config ----------------
static const int      PIN_SDA            = 21;
static const int      PIN_SCL            = 22;
static const uint32_t IMU_PERIOD_US      = 10000;  // อ่าน IMU 100 Hz ให้ filter แม่น
static const uint32_t SEND_INTERVAL_MS   = 200;    // ส่งให้ gateway 5 Hz
static const uint32_t STATS_INTERVAL_MS  = 2000;
static const uint32_t RESCAN_AFTER_FAILS = 15;     // ส่งไม่ถึงติดกัน 15 ครั้ง -> หา channel ใหม่
static const uint32_t IMU_REINIT_AFTER   = 50;     // อ่าน IMU พลาดติดกัน 50 ครั้ง -> init ใหม่

// ---------------- MPU6050: driver ระดับ register (จุดเริ่มของ E8) ----------------
namespace mpu {
const uint8_t ADDR             = 0x68;
const uint8_t REG_SMPLRT_DIV   = 0x19;
const uint8_t REG_CONFIG       = 0x1A;
const uint8_t REG_GYRO_CONFIG  = 0x1B;
const uint8_t REG_ACCEL_CONFIG = 0x1C;
const uint8_t REG_ACCEL_XOUT_H = 0x3B;
const uint8_t REG_PWR_MGMT_1   = 0x6B;
const uint8_t REG_WHO_AM_I     = 0x75;
const float   ACCEL_LSB_PER_G  = 16384.0f;  // ช่วง ±2 g
const float   GYRO_LSB_PER_DPS = 131.0f;    // ช่วง ±250 dps

struct Sample {
  float ax, ay, az;  // g
  float gx, gy, gz;  // deg/s
};

bool writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool readRegs(uint8_t reg, uint8_t *buf, int len) {
  Wire.beginTransmission(ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;  // repeated start
  if (Wire.requestFrom((int)ADDR, len) != len) return false;
  for (int i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

bool begin() {
  uint8_t who = 0;
  if (!readRegs(REG_WHO_AM_I, &who, 1)) return false;
  // ของแท้ได้ 0x68; บอร์ด clone บางรุ่นได้ 0x70/0x72/0x98 แต่ใช้ register ชุดเดียวกัน
  Serial.printf("[MPU] WHO_AM_I = 0x%02X\n", who);
  bool ok = writeReg(REG_PWR_MGMT_1, 0x01)      // ปลุกจาก sleep, ใช้ clock จาก PLL gyro X
         && writeReg(REG_SMPLRT_DIV, 9)         // 1 kHz / (1 + 9) = 100 Hz
         && writeReg(REG_CONFIG, 0x03)          // DLPF ~44 Hz ลด noise
         && writeReg(REG_GYRO_CONFIG, 0x00)     // ±250 dps
         && writeReg(REG_ACCEL_CONFIG, 0x00);   // ±2 g
  delay(50);
  return ok;
}

static inline int16_t be16(const uint8_t *b) { return (int16_t)((b[0] << 8) | b[1]); }

bool read(Sample &s) {
  uint8_t b[14];  // ACCEL_X..Z (6), TEMP (2), GYRO_X..Z (6)
  if (!readRegs(REG_ACCEL_XOUT_H, b, sizeof(b))) return false;
  s.ax = be16(&b[0]) / ACCEL_LSB_PER_G;
  s.ay = be16(&b[2]) / ACCEL_LSB_PER_G;
  s.az = be16(&b[4]) / ACCEL_LSB_PER_G;
  s.gx = be16(&b[8]) / GYRO_LSB_PER_DPS;
  s.gy = be16(&b[10]) / GYRO_LSB_PER_DPS;
  s.gz = be16(&b[12]) / GYRO_LSB_PER_DPS;
  return true;
}
}  // namespace mpu

// ---------------- เซนเซอร์แสง 3 ขา ----------------
// ตั้งค่าผ่าน build_flags ใน platformio.ini:
//   LIGHT_ENABLED      1 = ต่อแล้ว, 0 = ยังไม่ต่อ (gateway จะไม่ส่ง a_light)
//   LIGHT_PIN          ขาที่ต่อสัญญาณ
//   LIGHT_DIGITAL_ONLY 1 = อ่านแบบ digital สว่าง/มืด (ใช้ขา GPIO ไหนก็ได้ เช่น 19)
//                      0 = อ่านแบบ analog 0-100% (ต้องเป็น ADC1: GPIO32-39 เพราะ ADC2 ใช้ไม่ได้ตอนเปิด ESP-NOW)
//   LIGHT_INVERT       1 = เอามือบังแล้วค่ากลับด้าน
#ifndef LIGHT_ENABLED
#define LIGHT_ENABLED 0
#endif
#ifndef LIGHT_PIN
#define LIGHT_PIN 19
#endif
#ifndef LIGHT_DIGITAL_ONLY
#define LIGHT_DIGITAL_ONLY 1
#endif
#ifndef LIGHT_INVERT
#define LIGHT_INVERT 0
#endif

namespace light {
const bool  ENABLED       = LIGHT_ENABLED;
const int   PIN           = LIGHT_PIN;
const bool  DIGITAL_ONLY  = LIGHT_DIGITAL_ONLY;
const bool  INVERT        = LIGHT_INVERT;
const float FULL_SCALE_MV = 3100.0f;  // ADC ของ ESP32 อ่านได้สูงสุดราว 3.1 V

void begin() {
  if (ENABLED && DIGITAL_ONLY) pinMode(PIN, INPUT);
}

// คืนค่าความสว่าง 0-100% (100 = สว่างสุด) — เป็นค่าสัมพัทธ์ ไม่ใช่ lux
float readPercent() {
  float pct;
  if (DIGITAL_ONLY) {
    pct = digitalRead(PIN) ? 100.0f : 0.0f;
  } else {
    uint32_t sumMv = 0;
    for (int i = 0; i < 8; i++) sumMv += analogReadMilliVolts(PIN);  // เฉลี่ย 8 ครั้งลด noise
    pct = constrain(sumMv / 8.0f / FULL_SCALE_MV * 100.0f, 0.0f, 100.0f);
  }
  return INVERT ? 100.0f - pct : pct;
}
}  // namespace light

// ---------------- Attitude (complementary filter) ----------------
static float pitchDeg = 0, rollDeg = 0, gyroMag = 0;
static bool  attitudeReady = false;

static float wrap180(float a) {
  while (a > 180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

void updateAttitude(const mpu::Sample &s, float dt) {
  float accPitch = atan2f(-s.ax, sqrtf(s.ay * s.ay + s.az * s.az)) * RAD_TO_DEG;
  float accRoll  = atan2f(s.ay, s.az) * RAD_TO_DEG;
  gyroMag = sqrtf(s.gx * s.gx + s.gy * s.gy + s.gz * s.gz);
  if (!attitudeReady) {
    pitchDeg = accPitch;
    rollDeg  = accRoll;
    attitudeReady = true;
    return;
  }
  const float alpha = 0.96f;  // เชื่อ gyro ระยะสั้น, accel ระยะยาว
  // เขียนแบบ g + (1-alpha)*(acc - g) แล้ว wrap ส่วนต่างให้อยู่ใน ±180
  // ไม่งั้นตอนมุมข้าม ±180 (เช่น -179 -> +179) ค่าจะเฉลี่ยผิดและไหลเกิน ±180
  float gPitch = wrap180(pitchDeg + s.gy * dt);
  float gRoll  = wrap180(rollDeg + s.gx * dt);
  pitchDeg = wrap180(gPitch + (1.0f - alpha) * wrap180(accPitch - gPitch));
  rollDeg  = wrap180(gRoll + (1.0f - alpha) * wrap180(accRoll - gRoll));
}

// ---------------- ESP-NOW ----------------
static uint8_t gatewayMac[6] = GATEWAY_MAC;
static int32_t currentChannel = 0;
static volatile uint32_t sendOk = 0, sendFail = 0, consecutiveFail = 0;

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0)
// Arduino-ESP32 3.3+ (ESP-IDF 5.5) เปลี่ยน signature ของ send callback
void onSent(const esp_now_send_info_t *info, esp_now_send_status_t status) {
#else
void onSent(const uint8_t *mac, esp_now_send_status_t status) {
#endif
  if (status == ESP_NOW_SEND_SUCCESS) {
    sendOk++;
    consecutiveFail = 0;
  } else {
    sendFail++;
    consecutiveFail++;
  }
}

// ESP-NOW ต้องอยู่ channel เดียวกับ Wi-Fi ที่ gateway ต่ออยู่ จึงสแกนหา channel ของ hotspot
bool tuneToGatewayChannel() {
  Serial.printf("[NOW] scanning for \"%s\" ...\n", WIFI_SSID);
  int n = WiFi.scanNetworks(false, true);
  int32_t ch = 0;
  for (int i = 0; i < n; i++) {
    if (WiFi.SSID(i) == WIFI_SSID) {
      ch = WiFi.channel(i);
      break;
    }
  }
  WiFi.scanDelete();
  if (ch <= 0) {
    Serial.println("[NOW] SSID not found (hotspot off?) -> retry later");
    return false;
  }
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel((uint8_t)ch, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  currentChannel = ch;
  Serial.printf("[NOW] using channel %ld\n", (long)ch);
  return true;
}

bool setupEspNow() {
  if (esp_now_init() != ESP_OK) return false;
  esp_now_register_send_cb(onSent);
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, gatewayMac, 6);
  peer.channel = 0;  // 0 = ใช้ channel ปัจจุบันของ interface
  peer.ifidx   = WIFI_IF_STA;
  peer.encrypt = false;
  return esp_now_add_peer(&peer) == ESP_OK;
}

// ---------------- Main ----------------
static bool     imuOk = false;
static float    lightPct = 0;
static uint32_t seq = 0, imuErrors = 0;

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== ParallaxSense Node A ===");

  Wire.begin(PIN_SDA, PIN_SCL, 400000);
  imuOk = mpu::begin();
  light::begin();
  if (light::ENABLED) {
    Serial.printf("[I2C] MPU6050 %s | light sensor on GPIO%d (%s)\n", imuOk ? "OK" : "FAIL", light::PIN,
                  light::DIGITAL_ONLY ? "digital" : "analog");
  } else {
    Serial.printf("[I2C] MPU6050 %s | light sensor disabled\n", imuOk ? "OK" : "FAIL");
  }

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  Serial.printf("[NOW] my MAC = %s\n", WiFi.macAddress().c_str());
  while (!tuneToGatewayChannel()) delay(3000);
  if (!setupEspNow()) {
    Serial.println("[NOW] init failed -> restart");
    delay(1000);
    ESP.restart();
  }
}

void loop() {
  static uint32_t lastImuUs = micros(), lastSendMs = 0, lastStatsMs = 0;

  uint32_t nowUs = micros();
  if (nowUs - lastImuUs >= IMU_PERIOD_US) {
    float dt = (nowUs - lastImuUs) / 1e6f;
    lastImuUs = nowUs;
    mpu::Sample s;
    imuOk = mpu::read(s);
    if (imuOk) {
      imuErrors = 0;
      updateAttitude(s, dt);
    } else if (++imuErrors >= IMU_REINIT_AFTER) {
      Serial.println("[MPU] read failing -> re-init");
      imuErrors = 0;
      mpu::begin();
    }
  }

  uint32_t nowMs = millis();
  if (nowMs - lastSendMs >= SEND_INTERVAL_MS) {
    lastSendMs = nowMs;
    if (light::ENABLED) lightPct = light::readPercent();

    NodePacket p = {};
    p.version      = PACKET_VERSION;
    p.node_id      = NODE_A;
    p.flags        = (imuOk && attitudeReady ? FLAG_IMU_OK : 0) | (light::ENABLED ? FLAG_LIGHT_OK : 0);
    p.seq          = ++seq;
    p.uptime_ms    = nowMs;
    p.pitch_deg    = pitchDeg;
    p.roll_deg     = rollDeg;
    p.gyro_mag_dps = gyroMag;
    p.light_pct    = lightPct;
    p.fsr          = 0;
    esp_err_t err = esp_now_send(gatewayMac, (const uint8_t *)&p, sizeof(p));
    if (err != ESP_OK) Serial.printf("[NOW] esp_now_send error %d\n", (int)err);
  }

  if (consecutiveFail >= RESCAN_AFTER_FAILS) {
    consecutiveFail = 0;
    Serial.println("[NOW] gateway unreachable -> rescan channel");
    tuneToGatewayChannel();
  }

  if (nowMs - lastStatsMs >= STATS_INTERVAL_MS) {
    lastStatsMs = nowMs;
    uint32_t ok = sendOk, fail = sendFail;
    float pdr = (ok + fail) ? 100.0f * ok / (ok + fail) : 0;
    Serial.printf("[A] seq=%lu pitch=%.1f roll=%.1f gyro=%.1f light=%.0f%% | ch=%ld sent ok=%lu fail=%lu (%.1f%%)\n",
                  (unsigned long)seq, pitchDeg, rollDeg, gyroMag, lightPct, (long)currentChannel,
                  (unsigned long)ok, (unsigned long)fail, pdr);
  }
}
