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
int digitalRead(uint8_t) { return 0; }
uint32_t analogReadMilliVolts(uint8_t) { return 0; }
unsigned long pulseIn(int, int, unsigned long) { return 2900; }  // 2900 us / 58 = 50 cm
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
bool PubSubClient::connect(const char *, const char *, const char *) { return true; }
bool PubSubClient::connected() { return true; }
int PubSubClient::state() { return 0; }
bool PubSubClient::loop() { return true; }
bool PubSubClient::publish(const char *, const char *p) {
  published.push_back(p);
  return true;
}

// ---------------- helpers ----------------
static bool has(const std::string &s, const char *needle) { return s.find(needle) != std::string::npos; }

static void sendPkt(uint32_t seq, uint8_t flags, uint8_t version = PACKET_VERSION) {
  NodePacket p = {};
  p.version      = version;
  p.node_id      = NODE_A;
  p.flags        = flags;
  p.seq          = seq;
  p.pitch_deg    = -23.45f;
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
  TEST_ASSERT_EQUAL_STRING("{\"data\":{\"gw_dist_cm\":50.0,\"gw_uptime_s\":1}}", publishAt(1000).c_str());
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

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_boot_publishes_gateway_fields_only);
  RUN_TEST(test_pdr_counts_gaps_and_ignores_duplicates);
  RUN_TEST(test_node_goes_offline_after_timeout);
  RUN_TEST(test_rejects_wrong_packet_version);
  RUN_TEST(test_node_restart_and_missing_imu);
  RUN_TEST(test_publishes_once_per_second_with_valid_json);
  return UNITY_END();
}
