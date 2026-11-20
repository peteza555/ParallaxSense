// ParallaxSense — Sensor Node A (สวมที่คอ/ศีรษะ)
// MPU6050 (I2C) + BH1750 (I2C, lux) หรือโมดูลแสง LDR 3 ขา -> ESP-NOW -> Gateway
//
// ต่อสาย (MPU6050 กับ BH1750 ใช้สาย I2C ร่วมกัน ต่อขนานกันได้เลย):
//   MPU6050 : VCC -> 3V3, GND -> GND, SDA -> GPIO21, SCL -> GPIO22, AD0 -> GND (address 0x68)
//   BH1750  : VCC -> 3V3, GND -> GND, SDA -> GPIO21, SCL -> GPIO22, ADDR -> ไม่ต่อ/GND (address 0x23)
//   (ถ้าใช้โมดูล LDR 3 ขาแทน: VCC -> 3V3, GND -> GND, สัญญาณ -> LIGHT_PIN แล้วตั้ง LIGHT_BH1750=0)
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

// ---------------- เซนเซอร์แสง: BH1750 (lux) หรือโมดูล LDR 3 ขา ----------------
// ตั้งค่าผ่าน build_flags ใน platformio.ini:
//   LIGHT_BH1750       1 = ใช้ BH1750 ทาง I2C (ไม่สนค่า LIGHT_* ด้านล่าง), 0 = ใช้โมดูล LDR
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
#ifndef LIGHT_BH1750
#define LIGHT_BH1750 0
#endif

namespace light {
const bool  BH1750        = LIGHT_BH1750;
const bool  ENABLED       = LIGHT_ENABLED || LIGHT_BH1750;
const int   PIN           = LIGHT_PIN;
const bool  DIGITAL_ONLY  = LIGHT_DIGITAL_ONLY;
const bool  INVERT        = LIGHT_INVERT;
const float FULL_SCALE_MV = 3100.0f;  // ADC ของ ESP32 อ่านได้สูงสุดราว 3.1 V

// BH1750 ระดับ register: ส่ง opcode 1 byte, อ่านค่าดิบ 2 byte (big-endian), lux = raw / 1.2
const uint8_t  BH_ADDR      = 0x23;  // ขา ADDR ไม่ต่อ/GND (ต่อ 3V3 จะเป็น 0x5C)
const uint8_t  BH_POWER_ON  = 0x01;
const uint8_t  BH_CONT_HRES = 0x10;  // วัดต่อเนื่อง ละเอียด 1 lx, ได้ค่าใหม่ทุก ~120 ms
const uint32_t BH_RETRY_MS  = 5000;  // หาไม่เจอ/หลุด -> ลองใหม่ทุก 5 วินาที (เสียบสายทีหลังได้)
bool     bhOk = false;
uint32_t bhLastTry = 0;

bool bhCmd(uint8_t op) {
  Wire.beginTransmission(BH_ADDR);
  Wire.write(op);
  return Wire.endTransmission() == 0;
}

bool bhBegin() {
  bhLastTry = millis();
  bhOk = bhCmd(BH_POWER_ON) && bhCmd(BH_CONT_HRES);
  return bhOk;
}

void begin() {
  if (BH1750) bhBegin();
  else if (ENABLED && DIGITAL_ONLY) pinMode(PIN, INPUT);
}

const char *unit() { return BH1750 ? "lx" : "%"; }

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

// อ่านค่าล่าสุด: true = ได้ค่าจริง (BH1750 -> lux, LDR -> %)
bool read(float &out) {
  if (BH1750) {
    if (!bhOk && (millis() - bhLastTry < BH_RETRY_MS || !bhBegin())) return false;
    if (Wire.requestFrom((int)BH_ADDR, 2) != 2) {
      bhOk = false;
      return false;
    }
    uint16_t raw = (uint16_t)Wire.read() << 8;
    raw |= Wire.read();
    out = raw / 1.2f;
    return true;
  }
  if (!ENABLED) return false;
  out = readPercent();
  return true;
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

// ---------------- Calibration ----------------
// 1) gyro bias : วางนิ่ง ~2 วินาที เฉลี่ยค่า gyro (ควรเป็น 0) แล้วลบออกทุก sample ไม่งั้นมุมค่อย ๆ ไหล
// 2) ท่าอ้างอิง: เก็บทิศแรงโน้มถ่วงตอน "นั่งตัวตรง" แล้วหมุนทุก sample ให้ทิศนั้นเป็นแกน +Z
//    => นั่งตรง = pitch/roll 0° ไม่ว่าจะติดบอร์ดเอียง/ตะแคงแค่ไหนในกล่อง
// ทำตอนบูต และทุกครั้งที่กดปุ่ม BOOT (GPIO0) — ISR แค่ตั้ง flag แล้วไปทำต่อใน loop (deferred processing)
//
// ตั้งค่าใน platformio.ini [env:node_a] ถ้าก้มหน้าแล้วค่าไปขยับผิดช่อง:
//   IMU_SWAP_PITCH_ROLL=1  ก้มหน้าแล้ว roll เปลี่ยนแทน pitch
//   IMU_INVERT_PITCH=1     ก้มหน้าแล้ว pitch ติดลบ (อยากให้ก้ม = บวก)
//   IMU_INVERT_ROLL=1      เอียงขวาแล้ว roll ติดลบ
#ifndef IMU_SWAP_PITCH_ROLL
#define IMU_SWAP_PITCH_ROLL 0
#endif
#ifndef IMU_INVERT_PITCH
#define IMU_INVERT_PITCH 0
#endif
#ifndef IMU_INVERT_ROLL
#define IMU_INVERT_ROLL 0
#endif

namespace calib {
const int      PIN_BUTTON   = 0;     // ปุ่ม BOOT บนบอร์ด ESP32 DevKit
const int      SAMPLES      = 200;   // 200 x 10 ms = 2 วินาที
const float    STILL_DPS    = 3.0f;  // gyro แกว่งเกินนี้ = ยังขยับอยู่
const int      MAX_TRIES    = 5;

float bias[3] = {0, 0, 0};                      // deg/s
float R[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};  // หมุนทิศ gravity ตอนนั่งตรง -> +Z
volatile bool requested = false;
volatile uint32_t lastPressMs = 0;

void IRAM_ATTR onButton() {
  uint32_t now = millis();
  if (now - lastPressMs > 500) {  // debounce
    lastPressMs = now;
    requested = true;
  }
}

static void rotate(float v[3]) {
  float x = v[0], y = v[1], z = v[2];
  for (int i = 0; i < 3; i++) v[i] = R[i][0] * x + R[i][1] * y + R[i][2] * z;
}

// ใช้กับทุก sample ก่อนเข้า filter
void apply(mpu::Sample &s) {
  float a[3] = {s.ax, s.ay, s.az};
  float g[3] = {s.gx - bias[0], s.gy - bias[1], s.gz - bias[2]};
  rotate(a);
  rotate(g);
  s.ax = a[0]; s.ay = a[1]; s.az = a[2];
  s.gx = g[0]; s.gy = g[1]; s.gz = g[2];
}

// สร้าง R ที่หมุนเวกเตอร์หนึ่งหน่วย u ไปเป็น (0,0,1) ด้วยสูตร Rodrigues
static void alignToZ(const float u[3]) {
  float c = u[2];  // cos มุมระหว่าง u กับ +Z
  if (c < -0.9999f) {  // คว่ำพอดี: หมุน 180° รอบแกน X
    float flip[3][3] = {{1, 0, 0}, {0, -1, 0}, {0, 0, -1}};
    memcpy(R, flip, sizeof(R));
    return;
  }
  // v = u x z = (uy, -ux, 0), K = [v]x, R = I + K + K^2 / (1 + c)
  float vx = u[1], vy = -u[0];
  float K[3][3] = {{0, 0, vy}, {0, 0, -vx}, {-vy, vx, 0}};
  float k = 1.0f / (1.0f + c);
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) {
      float k2 = 0;
      for (int m = 0; m < 3; m++) k2 += K[i][m] * K[m][j];
      R[i][j] = (i == j ? 1.0f : 0.0f) + K[i][j] + k2 * k;
    }
}

// วัดนิ่ง ๆ 2 วินาที: ได้ gyro bias + ทิศ gravity -> ตั้ง R ใหม่ แล้วให้ filter เริ่มใหม่จากท่านี้
bool run() {
  Serial.println("[CAL] keep still and sit straight for 2 s ...");
  for (int attempt = 1; attempt <= MAX_TRIES; attempt++) {
    double sa[3] = {0, 0, 0}, sg[3] = {0, 0, 0};
    float gmin[3] = {1e9, 1e9, 1e9}, gmax[3] = {-1e9, -1e9, -1e9};
    int n = 0;
    for (int i = 0; i < SAMPLES; i++) {
      mpu::Sample s;
      if (mpu::read(s)) {
        float g[3] = {s.gx, s.gy, s.gz};
        sa[0] += s.ax; sa[1] += s.ay; sa[2] += s.az;
        for (int k = 0; k < 3; k++) {
          sg[k] += g[k];
          gmin[k] = min(gmin[k], g[k]);
          gmax[k] = max(gmax[k], g[k]);
        }
        n++;
      }
      delay(10);
    }
    if (n < SAMPLES / 2) {
      Serial.println("[CAL] FAIL: MPU6050 not responding");
      return false;
    }
    float spread = 0;
    for (int k = 0; k < 3; k++) spread = max(spread, gmax[k] - gmin[k]);
    bool still = spread < 2 * STILL_DPS;
    if (!still && attempt < MAX_TRIES) {
      Serial.printf("[CAL] moving (gyro spread %.1f dps) -> try again %d/%d\n", spread, attempt + 1, MAX_TRIES);
      continue;
    }

    float a[3] = {(float)(sa[0] / n), (float)(sa[1] / n), (float)(sa[2] / n)};
    float mag = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    if (mag < 0.5f) {
      Serial.println("[CAL] FAIL: accel reads ~0 g");
      return false;
    }
    for (int k = 0; k < 3; k++) {
      bias[k] = (float)(sg[k] / n);
      a[k] /= mag;
    }
    alignToZ(a);
    attitudeReady = false;  // filter เริ่มใหม่จากท่าอ้างอิง -> pitch/roll ~ 0
    Serial.printf("[CAL] %s gyro bias=(%.2f, %.2f, %.2f) dps | gravity=(%.2f, %.2f, %.2f) |g|=%.2f | mount tilt=%.0f deg\n",
                  still ? "OK" : "WARN (was moving, accepted anyway)", bias[0], bias[1], bias[2], a[0], a[1],
                  a[2], mag, acosf(constrain(a[2], -1.0f, 1.0f)) * RAD_TO_DEG);
    if (mag < 0.85f || mag > 1.15f) Serial.println("[CAL] warn: |g| far from 1.0 -> check MPU6050 wiring/mounting");
    return true;
  }
  return false;
}

void begin() {
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_BUTTON), onButton, FALLING);
}
}  // namespace calib

// ค่าที่ส่งออก (หลังสลับแกน/กลับเครื่องหมายตาม build flag)
static float outPitch() {
  float v = IMU_SWAP_PITCH_ROLL ? rollDeg : pitchDeg;
  return IMU_INVERT_PITCH ? -v : v;
}
static float outRoll() {
  float v = IMU_SWAP_PITCH_ROLL ? pitchDeg : rollDeg;
  return IMU_INVERT_ROLL ? -v : v;
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
static float    lightVal = 0;
static bool     lightOk = false;
static uint32_t seq = 0, imuErrors = 0;

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== ParallaxSense Node A ===");

  Wire.begin(PIN_SDA, PIN_SCL, 400000);
  imuOk = mpu::begin();
  light::begin();
  if (light::BH1750) {
    Serial.printf("[I2C] MPU6050 %s | BH1750 %s\n", imuOk ? "OK" : "FAIL",
                  light::bhOk ? "OK" : "not found (check SDA/SCL/3V3, will retry every 5 s)");
  } else if (light::ENABLED) {
    Serial.printf("[I2C] MPU6050 %s | light sensor on GPIO%d (%s)\n", imuOk ? "OK" : "FAIL", light::PIN,
                  light::DIGITAL_ONLY ? "digital" : "analog");
  } else {
    Serial.printf("[I2C] MPU6050 %s | light sensor disabled\n", imuOk ? "OK" : "FAIL");
  }
  calib::begin();
  if (imuOk) calib::run();
  Serial.println("[CAL] press BOOT button anytime to re-zero (sit straight first)");

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

  if (calib::requested) {  // กดปุ่ม BOOT -> ISR ตั้ง flag ไว้ มาทำงานหนักตรงนี้
    calib::requested = false;
    calib::run();
    lastImuUs = micros();
  }

  uint32_t nowUs = micros();
  if (nowUs - lastImuUs >= IMU_PERIOD_US) {
    float dt = (nowUs - lastImuUs) / 1e6f;
    lastImuUs = nowUs;
    mpu::Sample s;
    imuOk = mpu::read(s);
    if (imuOk) {
      imuErrors = 0;
      calib::apply(s);
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
    lightOk = light::read(lightVal);

    NodePacket p = {};
    p.version      = PACKET_VERSION;
    p.node_id      = NODE_A;
    p.flags        = (imuOk && attitudeReady ? FLAG_IMU_OK : 0) | (lightOk ? FLAG_LIGHT_OK : 0) |
                     (lightOk && light::BH1750 ? FLAG_LIGHT_LUX : 0);
    p.seq          = ++seq;
    p.uptime_ms    = nowMs;
    p.pitch_deg    = outPitch();
    p.roll_deg     = outRoll();
    p.gyro_mag_dps = gyroMag;
    p.light_pct    = lightOk ? lightVal : 0;
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
    char lightStr[16];
    if (lightOk) snprintf(lightStr, sizeof(lightStr), "%.0f%s", lightVal, light::unit());
    else snprintf(lightStr, sizeof(lightStr), "%s", light::ENABLED ? "n/a" : "off");
    Serial.printf("[A] seq=%lu pitch=%.1f roll=%.1f gyro=%.1f light=%s | ch=%ld sent ok=%lu fail=%lu (%.1f%%)\n",
                  (unsigned long)seq, outPitch(), outRoll(), gyroMag, lightStr, (long)currentChannel,
                  (unsigned long)ok, (unsigned long)fail, pdr);
  }
}
