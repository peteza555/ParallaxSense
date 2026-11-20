// ParallaxSense — Gateway (ESP32)
// รับ ESP-NOW จาก sensor node + อ่าน ultrasonic/PIR ของตัวเอง -> NETPIE2020 (MQTT shadow)
// + ยกธงด้วย servo เมื่อนั่งผิดท่า/ชิดจอเกินไปนานเกิน ALERT_AFTER_MS
//
// ต่อสาย (ใช้ขาฝั่งเดียวกับ VIN/GND ทั้งหมด = ฝั่งซ้ายของบอร์ด, VIN = 5V จาก USB แชร์กันผ่าน breadboard):
//   HC-SR04 : VCC -> VIN, GND -> GND, TRIG -> GPIO14
//             ECHO -> R 1kΩ -> GPIO13 -> R 2kΩ -> GND   (แบ่งแรงดัน 5V -> ~3.3V ห้ามต่อ ECHO ตรงเข้า ESP32)
//   PIR HC-SR501 : VCC -> VIN, GND -> GND, OUT -> GPIO27  (OUT เป็น 3.3V ต่อตรงได้)
//                  jumper ตั้ง H (retrigger), หมุน Time ไปทางน้อยสุด — firmware นับเวลาเอง
//   Servo SG90   : แดง -> VIN, น้ำตาล -> GND, ส้ม (สัญญาณ) -> GPIO26
//                  ถ้าบอร์ดรีเซ็ตตอน servo ขยับ: ใส่ C 470-1000µF คร่อม VIN-GND ใกล้ servo
//
// library PubSubClient ติดตั้งให้อัตโนมัติจาก lib_deps ใน platformio.ini

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_idf_version.h>
#include <PubSubClient.h>
#include "packet.h"
#if __has_include("secrets.h")
#include "secrets.h"
#else
#error "Missing include/secrets.h -> copy include/secrets.h.example to include/secrets.h and fill in real values"
#endif

// ---------------- Config ----------------
static const int      PIN_TRIG            = 14;
static const int      PIN_ECHO            = 13;
static const int      PIN_PIR             = 27;
static const int      PIN_SERVO           = 26;
static const uint32_t ULTRA_PERIOD_MS     = 100;
static const uint32_t PUBLISH_INTERVAL_MS = 1000;   // ถ้าเปลือง quota NETPIE ปรับเป็น 2000-5000
static const uint32_t NODE_TIMEOUT_MS     = 3000;   // ไม่ได้ยิน node เกินนี้ = offline
static const uint32_t MQTT_RETRY_MS       = 5000;
static const char    *MQTT_HOST           = "mqtt.netpie.io";
static const uint16_t MQTT_PORT           = 1883;   // checkpoint ใช้ plain ก่อน; ทำ E5 ค่อยเปลี่ยนเป็น 1884 + TLS
static const char    *TOPIC_SHADOW        = "@shadow/data/update";
static const int      MAX_NODES           = 4;

// เงื่อนไขยกธง (ปรับได้ตามที่ทดลองจริง)
static const uint32_t PIR_HOLD_MS         = 30000;  // เห็นการเคลื่อนไหวล่าสุดภายใน 30 s = มีคนอยู่
static const float    PRESENT_CM          = 120.0f; // หรือ ultrasonic เจออะไรใกล้กว่านี้ = มีคนอยู่
static const float    TOO_CLOSE_CM        = 40.0f;  // ใกล้จอเกินนี้ = ไม่ดี
static const float    SLOUCH_PITCH_DEG    = 20.0f;  // ก้ม/เงยเกินนี้ (หลัง calibrate นั่งตรง = 0°) = ไม่ดี
static const uint32_t ALERT_AFTER_MS      = 5000;   // ต้องไม่ดีต่อเนื่องนานเท่านี้ก่อนยกธง (กันธงกระพริบ)
static const int      SERVO_DOWN_DEG      = 10;     // ธงลง (เลี่ยง 0/180 ที่ servo บางตัวครางชนสุด)
static const int      SERVO_UP_DEG        = 100;    // ธงขึ้น

// ---------------- ESP-NOW receive -> queue ----------------
struct RxItem {
  NodePacket pkt;
  uint8_t    mac[6];
  int8_t     rssi;
  uint32_t   rx_ms;
};
static QueueHandle_t     rxQueue;
static volatile uint32_t rxDropped = 0, rxBadLen = 0;

// callback รันใน Wi-Fi task: แค่ copy ลง queue แล้วจบ ห้ามทำงานหนักหรือ print ในนี้
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  const uint8_t *mac = info->src_addr;
  int8_t rssi = info->rx_ctrl ? (int8_t)info->rx_ctrl->rssi : 0;
#else
void onRecv(const uint8_t *mac, const uint8_t *data, int len) {
  int8_t rssi = 0;
#endif
  if (len != (int)sizeof(NodePacket)) {
    rxBadLen++;
    return;
  }
  RxItem item;
  memcpy(&item.pkt, data, sizeof(NodePacket));
  memcpy(item.mac, mac, 6);
  item.rssi  = rssi;
  item.rx_ms = millis();
  if (xQueueSend(rxQueue, &item, 0) != pdTRUE) rxDropped++;
}

// ---------------- Node state ----------------
struct NodeState {
  bool       seen;
  NodePacket last;
  uint32_t   last_rx_ms;
  uint32_t   received;
  uint32_t   lost;  // นับจากช่องว่างของ seq
  int8_t     rssi;
};
static NodeState nodes[MAX_NODES + 1];  // index = node_id (1..MAX_NODES)

static char nodeTag(uint8_t id) { return (char)('a' + id - 1); }

static float nodePdr(const NodeState &n) {
  uint32_t total = n.received + n.lost;
  return total ? 100.0f * n.received / total : 0.0f;
}

void handlePacket(const RxItem &it) {
  const NodePacket &p = it.pkt;
  if (p.version != PACKET_VERSION || p.node_id < 1 || p.node_id > MAX_NODES) {
    Serial.printf("[RX] rejected packet (version=%u node=%u)\n", p.version, p.node_id);
    return;
  }
  NodeState &n = nodes[p.node_id];
  if (n.seen) {
    if (p.seq == n.last.seq) return;  // ซ้ำ (retry) ไม่นับซ้ำ
    if (p.seq > n.last.seq) {
      n.lost += p.seq - n.last.seq - 1;
    } else {
      Serial.printf("[RX] node %c restarted (seq %lu -> %lu)\n", toupper(nodeTag(p.node_id)),
                    (unsigned long)n.last.seq, (unsigned long)p.seq);
    }
  }
  n.seen       = true;
  n.last       = p;
  n.last_rx_ms = it.rx_ms;
  n.rssi       = it.rssi;
  n.received++;

  if (p.seq % 5 == 0) {  // พิมพ์ ~1 บรรทัด/วินาที/โหนด พอให้เห็นในวิดีโอ
    Serial.printf("[RX] node %c seq=%lu pitch=%.1f roll=%.1f gyro=%.1f light=%.0f%% rssi=%d PDR=%.1f%%\n",
                  toupper(nodeTag(p.node_id)), (unsigned long)p.seq, p.pitch_deg, p.roll_deg,
                  p.gyro_mag_dps, p.light_pct, it.rssi, nodePdr(n));
  }
}

// ---------------- PIR (interrupt) ----------------
// ISR (ขอบขาขึ้น) แค่นับครั้ง + จดเวลา งานที่เหลือ (presence, publish) ทำใน loop = deferred processing (E2)
// ระดับสัญญาณปัจจุบันอ่านใน loop ด้วย (pollPir) ไม่พึ่ง ISR อย่างเดียว:
// ถ้าขอบใดหลุดไป (สายแกว่ง/สัญญาณเด้ง) ค่าจะไม่ค้างผิด
static const uint32_t    PIR_DEBOUNCE_MS = 50;
static volatile uint32_t pirLastMs   = 0;   // เวลาล่าสุดที่เห็น OUT = HIGH
static volatile uint32_t pirRiseMs   = 0;
static volatile uint32_t pirRises    = 0;   // นับครั้งที่ตรวจพบการเคลื่อนไหว
static bool              pirLevel    = false;

void IRAM_ATTR onPir() {
  uint32_t now = millis();
  if (pirRises == 0 || now - pirRiseMs >= PIR_DEBOUNCE_MS) {
    pirRises++;
    pirRiseMs = now;
  }
  pirLastMs = now;
}

void pollPir(uint32_t now) {
  pirLevel = digitalRead(PIN_PIR);
  if (pirLevel) pirLastMs = now;  // ค้าง HIGH (คนยังขยับอยู่) = ยังมีคน
}

bool motionRecent(uint32_t now) { return pirLevel || (pirLastMs != 0 && now - pirLastMs < PIR_HOLD_MS); }

// ---------------- Servo ธงเตือน (PWM 50 Hz จาก LEDC ไม่ต้องใช้ library) ----------------
namespace servo {
const int      CH = 0, FREQ_HZ = 50, RES_BITS = 16;
const uint32_t MIN_US = 500, MAX_US = 2400;  // SG90: 0° ~ 0.5 ms, 180° ~ 2.4 ms
const uint32_t HOLD_MS = 700;                // ส่งพัลส์ช่วงเดียวพอให้หมุนถึง แล้วหยุด = ไม่สั่น ไม่กินไฟ
int      angle = -1;
uint32_t movedMs = 0;
bool     driving = false;

void write(int deg) {
  if (deg == angle) return;
  angle = deg;
  uint32_t us = MIN_US + (uint32_t)deg * (MAX_US - MIN_US) / 180;
  ledcWrite(CH, us * ((1u << RES_BITS) - 1) / 20000);
  movedMs = millis();
  driving = true;
}

void begin() {
  ledcSetup(CH, FREQ_HZ, RES_BITS);
  ledcAttachPin(PIN_SERVO, CH);
  write(SERVO_DOWN_DEG);
}

void idle() {
  if (driving && millis() - movedMs > HOLD_MS) {
    ledcWrite(CH, 0);
    driving = false;
  }
}
}  // namespace servo

// ---------------- Ultrasonic (median ของ 5 ค่าล่าสุด) ----------------
static float distBuf[5];
static int   distCount = 0, distIdx = 0;
static float    lastRawCm   = -1.0f;  // ค่าดิบล่าสุด (debug)
static uint32_t ultraOk     = 0, ultraFail = 0;  // นับต่อรอบ publish
static int      failStreak  = 0;

float readUltrasonicCm() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  unsigned long us = pulseIn(PIN_ECHO, HIGH, 25000UL);  // timeout ~4 m
  if (us == 0) return -1.0f;
  return us / 58.0f;
}

void sampleUltrasonic() {
  float d = readUltrasonicCm();
  lastRawCm = d;
  if (d < 2.0f || d > 400.0f) {  // timeout หรือนอกช่วงที่ HC-SR04 วัดได้
    ultraFail++;
    // อ่านพลาดติดกัน ~1 วินาที -> ทิ้งค่าเก่า ไม่ให้ dashboard โชว์ค่าค้าง
    if (++failStreak >= 10) distCount = 0;
    return;
  }
  ultraOk++;
  failStreak = 0;
  distBuf[distIdx] = d;
  distIdx = (distIdx + 1) % 5;
  if (distCount < 5) distCount++;
}

float distanceMedianCm() {
  if (distCount == 0) return -1.0f;
  float t[5];
  memcpy(t, distBuf, sizeof(float) * distCount);
  for (int i = 1; i < distCount; i++) {  // insertion sort (มีแค่ 5 ค่า)
    float v = t[i];
    int j = i - 1;
    while (j >= 0 && t[j] > v) {
      t[j + 1] = t[j];
      j--;
    }
    t[j + 1] = v;
  }
  return t[distCount / 2];
}

// ---------------- ตัดสินใจยกธง ----------------
struct Alert {
  bool     present, tooClose, slouch, bad, flagUp;
  uint32_t badSinceMs;
};
static Alert alert = {};

void updateAlert(uint32_t now) {
  float dist = distanceMedianCm();
  const NodeState &a = nodes[NODE_A];
  bool aOnline = a.seen && now - a.last_rx_ms < NODE_TIMEOUT_MS;
  alert.present  = motionRecent(now) || (dist > 0 && dist < PRESENT_CM);
  alert.tooClose = dist > 0 && dist < TOO_CLOSE_CM;
  alert.slouch   = aOnline && (a.last.flags & FLAG_IMU_OK) && fabsf(a.last.pitch_deg) > SLOUCH_PITCH_DEG;
  bool bad = alert.present && (alert.tooClose || alert.slouch);
  if (bad && !alert.bad) alert.badSinceMs = now;
  alert.bad    = bad;
  alert.flagUp = bad && now - alert.badSinceMs >= ALERT_AFTER_MS;
  servo::write(alert.flagUp ? SERVO_UP_DEG : SERVO_DOWN_DEG);
}

// ---------------- Wi-Fi + MQTT ----------------
WiFiClient   net;
PubSubClient mqtt(net);
static uint32_t publishCount = 0;

void printMacForNode() {
  uint8_t m[6];
  WiFi.macAddress(m);
  Serial.printf("[GW] MAC = %s\n", WiFi.macAddress().c_str());
  Serial.printf("[GW] copy to include/secrets.h -> #define GATEWAY_MAC {0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X}\n",
                m[0], m[1], m[2], m[3], m[4], m[5]);
}

void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);  // ปิด modem sleep ไม่งั้นรับ ESP-NOW ตกหล่น
  WiFi.setAutoReconnect(true);
  printMacForNode();
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.printf("[WiFi] connecting to \"%s\"", WIFI_SSID);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) {
    delay(300);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\n[WiFi] OK ip=%s channel=%ld rssi=%ld\n", WiFi.localIP().toString().c_str(),
                  (long)WiFi.channel(), (long)WiFi.RSSI());
  } else {
    Serial.println("\n[WiFi] not connected yet (will keep retrying in background)");
  }
}

void ensureMqtt() {
  static uint32_t lastTry = 0;
  if (WiFi.status() != WL_CONNECTED || mqtt.connected()) return;
  if (lastTry != 0 && millis() - lastTry < MQTT_RETRY_MS) return;
  lastTry = millis();
  Serial.print("[MQTT] connecting to NETPIE ... ");
  if (mqtt.connect(NETPIE_CLIENT_ID, NETPIE_TOKEN, NETPIE_SECRET)) {
    Serial.println("OK");
  } else {
    // -4 timeout, 4 bad credentials, 5 unauthorized: เช็ก Client ID / Token / Secret
    Serial.printf("FAIL rc=%d\n", mqtt.state());
  }
}

// ต่อท้าย JSON อย่างปลอดภัย ไม่ล้น buffer
struct JsonBuf {
  char   s[768];
  size_t n = 0;
  bool   first = true;
  void raw(const char *t) {
    int w = snprintf(s + n, sizeof(s) - n, "%s", t);
    if (w > 0) n += min((size_t)w, sizeof(s) - n - 1);
  }
  void num(const char *key, float v, int decimals = 1) {
    int w = snprintf(s + n, sizeof(s) - n, "%s\"%s\":%.*f", first ? "" : ",", key, decimals, v);
    if (w > 0) n += min((size_t)w, sizeof(s) - n - 1);
    first = false;
  }
};

void publishShadow() {
  JsonBuf j;
  j.raw("{\"data\":{");
  float dist = distanceMedianCm();
  if (dist > 0) j.num("gw_dist_cm", dist);
  j.num("gw_uptime_s", millis() / 1000.0f, 0);
  j.num("gw_motion", motionRecent(millis()) ? 1 : 0, 0);
  j.num("gw_present", alert.present ? 1 : 0, 0);
  j.num("gw_flag", alert.flagUp ? 1 : 0, 0);

  uint32_t now = millis();
  for (int id = 1; id <= MAX_NODES; id++) {
    const NodeState &n = nodes[id];
    if (!n.seen) continue;
    char key[24];
    char t = nodeTag(id);
    bool online = now - n.last_rx_ms < NODE_TIMEOUT_MS;
    snprintf(key, sizeof(key), "%c_online", t); j.num(key, online ? 1 : 0, 0);
    snprintf(key, sizeof(key), "%c_seq", t);    j.num(key, (float)n.last.seq, 0);
    snprintf(key, sizeof(key), "%c_pdr", t);    j.num(key, nodePdr(n));
    snprintf(key, sizeof(key), "%c_rssi", t);   j.num(key, n.rssi, 0);
    if (!online) continue;  // offline แล้วไม่ส่งค่าเก่าซ้ำ ให้ dashboard เห็นว่าหลุด
    if (n.last.flags & FLAG_IMU_OK) {
      snprintf(key, sizeof(key), "%c_pitch", t); j.num(key, n.last.pitch_deg);
      snprintf(key, sizeof(key), "%c_roll", t);  j.num(key, n.last.roll_deg);
      snprintf(key, sizeof(key), "%c_gyro", t);  j.num(key, n.last.gyro_mag_dps);
    }
    if (n.last.flags & FLAG_LIGHT_OK) {
      snprintf(key, sizeof(key), (n.last.flags & FLAG_LIGHT_LUX) ? "%c_lux" : "%c_light", t);
      j.num(key, n.last.light_pct, 0);
    }
    if (n.last.flags & FLAG_FSR_OK) {
      snprintf(key, sizeof(key), "%c_fsr", t);   j.num(key, n.last.fsr);
    }
  }
  j.raw("}}");

  Serial.printf("[US] raw=%.1fcm median=%.1fcm ok=%lu fail=%lu | PIR=%d motions=%lu | present=%d close=%d slouch=%d flag=%s\n",
                lastRawCm, dist, (unsigned long)ultraOk, (unsigned long)ultraFail, (int)pirLevel,
                (unsigned long)pirRises, alert.present, alert.tooClose, alert.slouch, alert.flagUp ? "UP" : "down");
  ultraOk = ultraFail = 0;

  if (!mqtt.connected()) {
    Serial.printf("[GW] dist=%.1fcm | MQTT offline (not published)\n", dist);
    return;
  }
  bool ok = mqtt.publish(TOPIC_SHADOW, j.s);
  if (ok) publishCount++;
  Serial.printf("[MQTT] %s #%lu %s\n", ok ? "published" : "PUBLISH FAILED", (unsigned long)publishCount, j.s);
}

// ---------------- Main ----------------
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== ParallaxSense Gateway ===");

  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);
  pinMode(PIN_PIR, INPUT_PULLDOWN);  // ยังไม่ต่อ PIR -> อ่านได้ LOW ไม่ลอย
  attachInterrupt(digitalPinToInterrupt(PIN_PIR), onPir, RISING);
  servo::begin();

  rxQueue = xQueueCreate(32, sizeof(RxItem));
  connectWiFi();

  if (esp_now_init() != ESP_OK) {
    Serial.println("[NOW] init failed -> restart");
    delay(1000);
    ESP.restart();
  }
  esp_now_register_recv_cb(onRecv);
  Serial.println("[NOW] ready, waiting for nodes ...");

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setBufferSize(1024);
  mqtt.setSocketTimeout(5);
}

void loop() {
  static uint32_t lastUltraMs = 0, lastPublishMs = 0;

  RxItem it;
  while (xQueueReceive(rxQueue, &it, 0) == pdTRUE) handlePacket(it);

  uint32_t now = millis();
  if (now - lastUltraMs >= ULTRA_PERIOD_MS) {
    lastUltraMs = now;
    sampleUltrasonic();
    pollPir(now);
    updateAlert(now);
  }
  servo::idle();

  ensureMqtt();
  mqtt.loop();

  if (now - lastPublishMs >= PUBLISH_INTERVAL_MS) {
    lastPublishMs = now;
    publishShadow();
    if (rxDropped || rxBadLen) {
      Serial.printf("[GW] warn: queue dropped=%lu bad_len=%lu\n", (unsigned long)rxDropped,
                    (unsigned long)rxBadLen);
    }
  }
}
