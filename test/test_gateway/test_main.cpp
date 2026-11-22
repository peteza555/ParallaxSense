// Unit test ของ logic gateway: PDR จาก seq, online/offline, JSON ที่ส่งขึ้น NETPIE
// รันบนคอม ไม่ต้องมีบอร์ด:  pio test -e native
// Arduino/ESP-IDF ถูกแทนด้วย header ปลอมใน test/stubs/
#include <unity.h>

#include "../../src/gateway/main.cpp"

#include <deque>
#include <string>
#include <vector>

// ---------------- fakes ----------------
static uint32_t fake_ms = 0;
uint32_t millis() { return fake_ms; }
uint32_t micros() { return fake_ms * 1000; }
void delay(uint32_t) {}
void delayMicroseconds(uint32_t) {}
void pinMode(int, int) {}
void digitalWrite(int, int) {}
static int fake_pir = 0;
int digitalRead(uint8_t) { return fake_pir; }
uint32_t analogReadMilliVolts(uint8_t) { return 0; }
static unsigned long fake_echo_us = 2900;  // 2900 us / 58 = 50 cm
unsigned long pulseIn(int, int, unsigned long) { return fake_echo_us; }
void attachInterrupt(uint8_t, void (*)(void), int) {}
static uint32_t servo_duty = 0;
uint32_t ledcSetup(uint8_t, uint32_t, uint8_t) { return 50; }
void ledcAttachPin(uint8_t, uint8_t) {}
void ledcWrite(uint8_t, uint32_t d) { servo_duty = d; }
HwSerial Serial;
void HwSerial::begin(int) {}
int HwSerial::printf(const char *, ...) { return 0; }
void HwSerial::print(const char *) {}
void HwSerial::println(const char *) {}
EspClass ESP;
void EspClass::restart() {}

static std::deque<RxItem> q;
QueueHandle_t xQueueCreate(int, int) { return (void *)1; }
BaseType_t xQueueSend(QueueHandle_t, const void *p, int) {
  q.push_back(*(const RxItem *)p);
  return 1;
}
BaseType_t xQueueReceive(QueueHandle_t, void *p, int) {
  if (q.empty()) return 0;
  *(RxItem *)p = q.front();
  q.pop_front();
  return 1;
}

WiFiClass WiFi;
void WiFiClass::mode(int) {}
String WiFiClass::macAddress() { return "AB:AB:AB:AB:AB:AB"; }
uint8_t *WiFiClass::macAddress(uint8_t *m) {
  memset(m, 0xAB, 6);
  return m;
}
void WiFiClass::setSleep(bool) {}
void WiFiClass::setAutoReconnect(bool) {}
void WiFiClass::begin(const char *, const char *) {}
int WiFiClass::status() { return WL_CONNECTED; }
IP WiFiClass::localIP() { return IP(); }
String IP::toString() const { return "10.0.0.2"; }
int32_t WiFiClass::channel() { return 6; }
int8_t WiFiClass::RSSI() { return -50; }
esp_err_t esp_now_init() { return 0; }
esp_err_t esp_now_register_recv_cb(esp_now_recv_cb_t) { return 0; }

static std::vector<std::string> published;
PubSubClient::PubSubClient(WiFiClient &) {}
void PubSubClient::setServer(const char *, uint16_t) {}
bool PubSubClient::setBufferSize(uint16_t) { return true; }
void PubSubClient::setSocketTimeout(uint16_t) {}
static bool mqtt_up = false;
bool PubSubClient::connect(const char *, const char *, const char *) { return mqtt_up = true; }
bool PubSubClient::connected() { return mqtt_up; }
int PubSubClient::state() { return 0; }
bool PubSubClient::loop() { return true; }
bool PubSubClient::publish(const char *, const char *p) {
  published.push_back(p);
  return true;
}
static MqttCallback mqtt_cb = nullptr;
static std::vector<std::string> subscribed;
PubSubClient &PubSubClient::setCallback(MqttCallback cb) {
  mqtt_cb = cb;
  return *this;
}
bool PubSubClient::subscribe(const char *t) {
  subscribed.push_back(t);
  return true;
}
// จำลอง NETPIE ส่งข้อความเข้ามา (เหมือนตอน mqtt.loop() เรียก callback)
static void mqttIn(const char *topic, const char *payload) {
  std::string t = topic;
  mqtt_cb(&t[0], (uint8_t *)payload, (unsigned int)strlen(payload));
}

// ---------------- helpers ----------------
static bool has(const std::string &s, const char *needle) { return s.find(needle) != std::string::npos; }

static void sendPkt(uint32_t seq, uint8_t flags, uint8_t version = PACKET_VERSION, float pitch = -23.45f) {
  NodePacket p = {};
  p.version      = version;
  p.node_id      = NODE_A;
  p.flags        = flags;
  p.seq          = seq;
  p.pitch_deg    = pitch;
  p.roll_deg     = 4.2f;
  p.gyro_mag_dps = 1.5f;
  p.light_pct    = 100.0f;
  uint8_t mac[6] = {1, 2, 3, 4, 5, 6};
  wifi_pkt_rx_ctrl_t rc = {};
  rc.rssi = -47;
  esp_now_recv_info_t info = {mac, nullptr, &rc};
  onRecv(&info, (const uint8_t *)&p, sizeof(p));
}

static std::string publishAt(uint32_t ms) {
  fake_ms = ms;
  loop();
  return published.empty() ? std::string() : published.back();
}

// ---------------- tests (รันตามลำดับ ใช้ state ของ gateway ต่อกัน) ----------------
void setUp() {}
void tearDown() {}

void test_boot_publishes_gateway_fields_only() {
  setup();
  for (int i = 1; i <= 9; i++) {  // เก็บค่า ultrasonic ให้ median มีข้อมูล
    fake_ms = i * 100;
    loop();
  }
  TEST_ASSERT_EQUAL_STRING(
      "{\"data\":{\"gw_dist_cm\":50.0,\"gw_uptime_s\":1,\"gw_motion\":0,\"gw_present\":1,\"gw_flag\":0}}",
      publishAt(1000).c_str());
}

void test_pdr_counts_gaps_and_ignores_duplicates() {
  // seq 4,5 หาย และ 7 ซ้ำ -> received 8, lost 2 -> PDR 80%
  for (uint32_t s : {1u, 2u, 3u, 6u, 7u, 7u, 8u, 9u, 10u}) sendPkt(s, FLAG_IMU_OK | FLAG_LIGHT_OK);
  std::string p = publishAt(2000);
  TEST_ASSERT_TRUE_MESSAGE(has(p, "\"a_online\":1"), p.c_str());
  TEST_ASSERT_TRUE_MESSAGE(has(p, "\"a_seq\":10"), p.c_str());
  TEST_ASSERT_TRUE_MESSAGE(has(p, "\"a_pdr\":80.0"), p.c_str());
  TEST_ASSERT_TRUE_MESSAGE(has(p, "\"a_pitch\":-23.5"), p.c_str());
  TEST_ASSERT_TRUE_MESSAGE(has(p, "\"a_light\":100"), p.c_str());
}

void test_node_goes_offline_after_timeout() {
  std::string p = publishAt(6000);  // เงียบไป 4 s > NODE_TIMEOUT_MS
  TEST_ASSERT_TRUE_MESSAGE(has(p, "\"a_online\":0"), p.c_str());
  TEST_ASSERT_FALSE_MESSAGE(has(p, "a_pitch"), p.c_str());  // ไม่ส่งค่าเก่าซ้ำ
}

void test_rejects_wrong_packet_version() {
  sendPkt(11, FLAG_IMU_OK, 99);
  std::string p = publishAt(7000);
  TEST_ASSERT_TRUE_MESSAGE(has(p, "\"a_seq\":10"), p.c_str());
}

void test_node_restart_and_missing_imu() {
  sendPkt(1, FLAG_LIGHT_OK);  // seq เริ่มใหม่ + IMU อ่านไม่ได้
  std::string p = publishAt(8000);
  TEST_ASSERT_TRUE_MESSAGE(has(p, "\"a_seq\":1,"), p.c_str());
  TEST_ASSERT_TRUE_MESSAGE(has(p, "\"a_online\":1"), p.c_str());
  TEST_ASSERT_FALSE_MESSAGE(has(p, "a_pitch"), p.c_str());
  TEST_ASSERT_TRUE_MESSAGE(has(p, "\"a_light\":100"), p.c_str());
}

void test_publishes_once_per_second_with_valid_json() {
  TEST_ASSERT_EQUAL_UINT32(5, published.size());  // 1000, 2000, 6000, 7000, 8000 ms
  for (const auto &s : published) {
    int depth = 0;
    for (char c : s) depth += (c == '{') - (c == '}');
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, depth, s.c_str());
    TEST_ASSERT_EQUAL_CHAR('{', s.front());
    TEST_ASSERT_EQUAL_CHAR('}', s.back());
  }
}

// ---------------- ธงเตือน (servo) / PIR / lux ----------------
static uint32_t servoDutyFor(int deg) {
  uint32_t us = servo::MIN_US + (uint32_t)deg * (servo::MAX_US - servo::MIN_US) / 180;
  return us * 65535 / 20000;
}

// เดินเวลาไปทีละ 100 ms ตั้งแต่ from ถึง to; ส่ง packet จาก node A ทุก 200 ms ด้วย pitch ที่กำหนด
static uint32_t nextSeq = 100;
static void runFor(uint32_t from, uint32_t to, float pitch, bool sendNode = true) {
  for (uint32_t t = from; t <= to; t += 100) {
    fake_ms = t;
    if (sendNode && t % 200 == 0) sendPkt(nextSeq++, FLAG_IMU_OK, PACKET_VERSION, pitch);
    loop();
  }
}

void test_flag_rises_only_after_slouching_5s() {
  runFor(10000, 14000, -23.45f);  // ก้ม 23° มา 4 วินาที -> ยังไม่ยก
  TEST_ASSERT_FALSE(alert.flagUp);
  TEST_ASSERT_TRUE(alert.slouch);
  runFor(14100, 15500, -23.45f);  // เกิน 5 วินาที -> ยกธง
  TEST_ASSERT_TRUE(alert.flagUp);
  TEST_ASSERT_TRUE_MESSAGE(has(published.back(), "\"gw_flag\":1"), published.back().c_str());
  runFor(15600, 16500, -23.45f);  // ส่งพัลส์ครบแล้วหยุด PWM (servo ไม่สั่น)
  TEST_ASSERT_EQUAL_UINT32(0, servo_duty);
  TEST_ASSERT_EQUAL_INT(SERVO_UP_DEG, servo::angle);
}

void test_flag_drops_immediately_when_posture_ok() {
  runFor(16600, 16800, 3.0f);
  TEST_ASSERT_FALSE(alert.flagUp);
  TEST_ASSERT_EQUAL_UINT32(servoDutyFor(SERVO_DOWN_DEG), servo_duty);
}

void test_too_close_raises_flag_without_node() {
  fake_echo_us = 30 * 58;  // 30 cm < TOO_CLOSE_CM
  runFor(20000, 26000, 0, false);
  TEST_ASSERT_TRUE(alert.tooClose);
  TEST_ASSERT_FALSE(alert.slouch);  // node A เงียบไปแล้ว ไม่เอามาคิด
  TEST_ASSERT_TRUE(alert.flagUp);
  fake_echo_us = 2900;
  runFor(26100, 27000, 0, false);
  TEST_ASSERT_FALSE(alert.flagUp);
}

void test_pir_interrupt_marks_motion_for_hold_time() {
  fake_echo_us = 0;  // ultrasonic ไม่เจออะไร -> presence มาจาก PIR อย่างเดียว
  runFor(28000, 29000, 0, false);
  TEST_ASSERT_FALSE(alert.present);
  fake_ms = 29050;
  fake_pir = 1; onPir();
  onPir();  // สายเด้ง: ขอบขึ้นซ้ำภายใน 50 ms ไม่นับซ้ำ
  fake_pir = 0;  // PIR ปล่อยแล้ว
  std::string p = publishAt(30000);
  TEST_ASSERT_TRUE_MESSAGE(has(p, "\"gw_motion\":1"), p.c_str());
  TEST_ASSERT_TRUE(alert.present);
  TEST_ASSERT_FALSE(pirLevel);  // ระดับปัจจุบันอ่านจาก loop ไม่ค้างที่ 1
  TEST_ASSERT_EQUAL_UINT32(1, pirRises);
  runFor(30100, 29050 + PIR_HOLD_MS + 100, 0, false);  // เลย 30 s -> ไม่นับว่ามีคน
  TEST_ASSERT_FALSE(alert.present);
}

void test_pir_level_polled_even_if_interrupt_missed() {
  uint32_t t0 = fake_ms + 100;
  fake_pir = 1;  // OUT ค้าง HIGH แต่ไม่มี ISR (ขอบหลุด)
  runFor(t0, t0 + 200, 0, false);
  TEST_ASSERT_TRUE(pirLevel);
  TEST_ASSERT_TRUE(alert.present);
  fake_pir = 0;
  runFor(t0 + 300, t0 + 300, 0, false);
  TEST_ASSERT_FALSE(pirLevel);   // ลงทันที
  TEST_ASSERT_TRUE(alert.present);  // แต่ยังนับว่ามีคนภายใน PIR_HOLD_MS
  fake_echo_us = 2900;
}

void test_bh1750_lux_published_as_a_lux() {
  sendPkt(nextSeq++, FLAG_IMU_OK | FLAG_LIGHT_OK | FLAG_LIGHT_LUX);  // light_pct = 100 -> 100 lx
  std::string p = publishAt(fake_ms + 1000);
  TEST_ASSERT_TRUE_MESSAGE(has(p, "\"a_lux\":100"), p.c_str());
  TEST_ASSERT_FALSE_MESSAGE(has(p, "a_light"), p.c_str());
}

// ---------------- AI จาก laptop (@msg/ai) ----------------
void test_subscribes_to_ai_topic_on_connect() {
  TEST_ASSERT_EQUAL_UINT32(1, subscribed.size());
  TEST_ASSERT_EQUAL_STRING("@msg/ai", subscribed[0].c_str());
}

void test_ai_bad_raises_flag_then_ignored_when_stale() {
  uint32_t t0 = fake_ms + 4000;  // node A เงียบไปแล้ว (offline) -> ไม่มี slouch จาก IMU
  runFor(t0, t0 + 1000, 0, false);
  TEST_ASSERT_FALSE(alert.slouch);
  TEST_ASSERT_FALSE(alert.flagUp);
  for (uint32_t t = t0 + 1100; t <= t0 + 7100; t += 100) {  // AI ส่งทุก 1 s ว่าท่าไม่ดี
    fake_ms = t;
    if (t % 1000 == 100) mqttIn("@msg/ai", "{\"bad\":1,\"p\":0.91,\"present\":1}");
    loop();
  }
  TEST_ASSERT_TRUE(alert.aiBad);
  TEST_ASSERT_TRUE(alert.flagUp);
  TEST_ASSERT_TRUE_MESSAGE(has(published.back(), "\"ai_online\":1,\"ai_bad\":1"), published.back().c_str());
  runFor(t0 + 7200, t0 + 13000, 0, false);  // laptop หยุดส่งเกิน 5 s -> ไม่เอาผล AI มาคิด
  TEST_ASSERT_FALSE(alert.aiBad);
  TEST_ASSERT_FALSE(alert.flagUp);
  TEST_ASSERT_TRUE_MESSAGE(has(published.back(), "\"ai_online\":0"), published.back().c_str());
  TEST_ASSERT_FALSE_MESSAGE(has(published.back(), "ai_bad"), published.back().c_str());
}

void test_ai_ignores_malformed_and_other_topics() {
  uint32_t n = ai.count, badN = ai.badMsgs;
  mqttIn("@msg/ai", "hello");
  mqttIn("@msg/ai", "{\"p\":0.5}");                   // ไม่มี bad
  mqttIn("@msg/other", "{\"bad\":1,\"p\":1}");     // topic อื่นไม่สน
  TEST_ASSERT_EQUAL_UINT32(n, ai.count);
  TEST_ASSERT_EQUAL_UINT32(badN + 2, ai.badMsgs);
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_boot_publishes_gateway_fields_only);
  RUN_TEST(test_pdr_counts_gaps_and_ignores_duplicates);
  RUN_TEST(test_node_goes_offline_after_timeout);
  RUN_TEST(test_rejects_wrong_packet_version);
  RUN_TEST(test_node_restart_and_missing_imu);
  RUN_TEST(test_publishes_once_per_second_with_valid_json);
  RUN_TEST(test_flag_rises_only_after_slouching_5s);
  RUN_TEST(test_flag_drops_immediately_when_posture_ok);
  RUN_TEST(test_too_close_raises_flag_without_node);
  RUN_TEST(test_pir_interrupt_marks_motion_for_hold_time);
  RUN_TEST(test_pir_level_polled_even_if_interrupt_missed);
  RUN_TEST(test_bh1750_lux_published_as_a_lux);
  RUN_TEST(test_subscribes_to_ai_topic_on_connect);
  RUN_TEST(test_ai_bad_raises_flag_then_ignored_when_stale);
  RUN_TEST(test_ai_ignores_malformed_and_other_topics);
  return UNITY_END();
}
