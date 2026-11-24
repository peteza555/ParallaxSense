// ParallaxSense - Sensor Node B (เก้าอี้)
// MPU6050 (gyro + accelerometer, I2C) + two analog FSRs -> ESP-NOW -> Gateway
//
// ต่อสาย:
//   MPU6050 : VCC 3V3, GND GND, SDA GPIO21, SCL GPIO22, AD0 GND
//   FSR 1   : ขาหนึ่ง -> 3V3; อีกขา -> GPIO32 และตัวต้านทาน 10kΩ ลง GND
//   FSR 2   : ขาหนึ่ง -> 3V3; อีกขา -> GPIO33 และตัวต้านทาน 10kΩ ลง GND

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

static const int PIN_SDA = 21;
static const int PIN_SCL = 22;
static const uint32_t IMU_PERIOD_US = 10000;
static const uint32_t SEND_INTERVAL_MS = 200;
static const uint32_t STATS_INTERVAL_MS = 2000;
static const uint32_t RESCAN_AFTER_FAILS = 15;

#ifndef FSR1_PIN
#define FSR1_PIN 32
#endif
#ifndef FSR2_PIN
#define FSR2_PIN 33
#endif
#ifndef FSR_FULL_SCALE_MV
#define FSR_FULL_SCALE_MV 3100
#endif
#ifndef FSR_INVERT
#define FSR_INVERT 0
#endif
#ifndef IMU_SWAP_PITCH_ROLL
#define IMU_SWAP_PITCH_ROLL 0
#endif
#ifndef IMU_INVERT_PITCH
#define IMU_INVERT_PITCH 0
#endif
#ifndef IMU_INVERT_ROLL
#define IMU_INVERT_ROLL 0
#endif

namespace mpu {
const uint8_t ADDR = 0x68;
const uint8_t REG_SMPLRT_DIV = 0x19;
const uint8_t REG_CONFIG = 0x1A;
const uint8_t REG_GYRO_CONFIG = 0x1B;
const uint8_t REG_ACCEL_CONFIG = 0x1C;
const uint8_t REG_ACCEL_XOUT_H = 0x3B;
const uint8_t REG_PWR_MGMT_1 = 0x6B;
const uint8_t REG_WHO_AM_I = 0x75;

struct Sample {
  float ax, ay, az;
  float gx, gy, gz;
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
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)ADDR, len) != len) return false;
  for (int i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

bool begin() {
  uint8_t who = 0;
  if (!readRegs(REG_WHO_AM_I, &who, 1)) return false;
  Serial.printf("[MPU] WHO_AM_I = 0x%02X\n", who);
  bool ok = writeReg(REG_PWR_MGMT_1, 0x01)
         && writeReg(REG_SMPLRT_DIV, 9)
         && writeReg(REG_CONFIG, 0x03)
         && writeReg(REG_GYRO_CONFIG, 0x00)
         && writeReg(REG_ACCEL_CONFIG, 0x00);
  delay(50);
  return ok;
}

static int16_t be16(const uint8_t *b) {
  return (int16_t)((b[0] << 8) | b[1]);
}

bool read(Sample &s) {
  uint8_t b[14];
  if (!readRegs(REG_ACCEL_XOUT_H, b, sizeof(b))) return false;
  s.ax = be16(&b[0]) / 16384.0f;
  s.ay = be16(&b[2]) / 16384.0f;
  s.az = be16(&b[4]) / 16384.0f;
  s.gx = be16(&b[8]) / 131.0f;
  s.gy = be16(&b[10]) / 131.0f;
  s.gz = be16(&b[12]) / 131.0f;
  return true;
}
} // namespace mpu

namespace fsr {
float readPercent(int pin) {
  uint32_t sumMv = 0;
  for (int i = 0; i < 8; i++) sumMv += analogReadMilliVolts(pin);
  float pct = constrain(sumMv / 8.0f / (float)FSR_FULL_SCALE_MV * 100.0f, 0.0f, 100.0f);
  return FSR_INVERT ? 100.0f - pct : pct;
}
} // namespace fsr

static float pitchDeg = 0, rollDeg = 0, gyroMag = 0;
static bool attitudeReady = false;

static float wrap180(float a) {
  while (a > 180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

void updateAttitude(const mpu::Sample &s, float dt) {
  float accPitch = atan2f(-s.ax, sqrtf(s.ay * s.ay + s.az * s.az)) * RAD_TO_DEG;
  float accRoll = atan2f(s.ay, s.az) * RAD_TO_DEG;
  gyroMag = sqrtf(s.gx * s.gx + s.gy * s.gy + s.gz * s.gz);
  if (!attitudeReady) {
    pitchDeg = accPitch;
    rollDeg = accRoll;
    attitudeReady = true;
    return;
  }
  const float alpha = 0.96f;
  float gPitch = wrap180(pitchDeg + s.gy * dt);
  float gRoll = wrap180(rollDeg + s.gx * dt);
  pitchDeg = wrap180(gPitch + (1.0f - alpha) * wrap180(accPitch - gPitch));
  rollDeg = wrap180(gRoll + (1.0f - alpha) * wrap180(accRoll - gRoll));
}

namespace calib {
const int PIN_BUTTON = 0;
const int SAMPLES = 200;
float bias[3] = {0, 0, 0};
float R[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
volatile bool requested = false;
volatile uint32_t lastPressMs = 0;

void IRAM_ATTR onButton() {
  uint32_t now = millis();
  if (now - lastPressMs > 500) {
    lastPressMs = now;
    requested = true;
  }
}

static void rotate(float v[3]) {
  float x = v[0], y = v[1], z = v[2];
  for (int i = 0; i < 3; i++) v[i] = R[i][0] * x + R[i][1] * y + R[i][2] * z;
}

void apply(mpu::Sample &s) {
  float a[3] = {s.ax, s.ay, s.az};
  float g[3] = {s.gx - bias[0], s.gy - bias[1], s.gz - bias[2]};
  rotate(a);
  rotate(g);
  s.ax = a[0]; s.ay = a[1]; s.az = a[2];
  s.gx = g[0]; s.gy = g[1]; s.gz = g[2];
}

static void alignToZ(const float u[3]) {
  float c = u[2];
  if (c < -0.9999f) {
    float flip[3][3] = {{1, 0, 0}, {0, -1, 0}, {0, 0, -1}};
    memcpy(R, flip, sizeof(R));
    return;
  }
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

bool run() {
  Serial.println("[CAL] keep chair and body still for 2 s ...");
  double sa[3] = {0, 0, 0}, sg[3] = {0, 0, 0};
  int n = 0;
  for (int i = 0; i < SAMPLES; i++) {
    mpu::Sample s;
    if (mpu::read(s)) {
      sa[0] += s.ax; sa[1] += s.ay; sa[2] += s.az;
      sg[0] += s.gx; sg[1] += s.gy; sg[2] += s.gz;
      n++;
    }
    delay(10);
  }
  if (n < SAMPLES / 2) {
    Serial.println("[CAL] MPU6050 not responding");
    return false;
  }
  float a[3] = {(float)(sa[0] / n), (float)(sa[1] / n), (float)(sa[2] / n)};
  float mag = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
  if (mag < 0.5f) {
    Serial.println("[CAL] accel reads ~0 g");
    return false;
  }
  for (int k = 0; k < 3; k++) {
    bias[k] = (float)(sg[k] / n);
    a[k] /= mag;
  }
  alignToZ(a);
  attitudeReady = false;
  Serial.printf("[CAL] OK gyro bias=(%.2f, %.2f, %.2f) dps\n", bias[0], bias[1], bias[2]);
  return true;
}

void begin() {
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_BUTTON), onButton, FALLING);
}
} // namespace calib

static float outPitch() {
  float v = IMU_SWAP_PITCH_ROLL ? rollDeg : pitchDeg;
  return IMU_INVERT_PITCH ? -v : v;
}

static float outRoll() {
  float v = IMU_SWAP_PITCH_ROLL ? pitchDeg : rollDeg;
  return IMU_INVERT_ROLL ? -v : v;
}

static uint8_t gatewayMac[6] = GATEWAY_MAC;
static int32_t currentChannel = 0;
static volatile uint32_t sendOk = 0, sendFail = 0, consecutiveFail = 0;

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0)
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
  if (ch <= 0) return false;
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
  peer.channel = 0;
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;
  return esp_now_add_peer(&peer) == ESP_OK;
}

static bool imuOk = false;
static float fsr1Percent = 0, fsr2Percent = 0;
static uint32_t seq = 0, imuErrors = 0;

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== ParallaxSense Node B ===");

  Wire.begin(PIN_SDA, PIN_SCL, 400000);
  imuOk = mpu::begin();
  pinMode(FSR1_PIN, INPUT);
  pinMode(FSR2_PIN, INPUT);
  calib::begin();
  if (imuOk) calib::run();
  Serial.printf("[I2C] MPU6050 %s | FSR analog GPIO%d + GPIO%d\n",
                imuOk ? "OK" : "FAIL", FSR1_PIN, FSR2_PIN);
  Serial.println("[CAL] press BOOT button anytime to re-zero");

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
  static uint32_t lastImuUs = micros();
  static uint32_t lastSendMs = 0, lastStatsMs = 0;

  if (calib::requested) {
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
    } else if (++imuErrors >= 50) {
      Serial.println("[MPU] read failing -> re-init");
      imuErrors = 0;
      mpu::begin();
    }
  }

  uint32_t nowMs = millis();
  if (nowMs - lastSendMs >= SEND_INTERVAL_MS) {
    lastSendMs = nowMs;
    fsr1Percent = fsr::readPercent(FSR1_PIN);
    fsr2Percent = fsr::readPercent(FSR2_PIN);

    NodePacket p = {};
    p.version = PACKET_VERSION;
    p.node_id = NODE_B;
    p.flags = (imuOk && attitudeReady ? FLAG_IMU_OK : 0) | FLAG_FSR_OK;
    p.seq = ++seq;
    p.uptime_ms = nowMs;
    p.pitch_deg = outPitch();
    p.roll_deg = outRoll();
    p.gyro_mag_dps = gyroMag;
    p.light_pct = 0;
    p.fsr1 = fsr1Percent;
    p.fsr2 = fsr2Percent;

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
    Serial.printf("[B] seq=%lu pitch=%.1f roll=%.1f gyro=%.1f fsr1=%.0f%% fsr2=%.0f%% | ch=%ld sent ok=%lu fail=%lu (%.1f%%)\n",
                  (unsigned long)seq, outPitch(), outRoll(), gyroMag, fsr1Percent, fsr2Percent,
                  (long)currentChannel, (unsigned long)ok, (unsigned long)fail, pdr);
  }
}
