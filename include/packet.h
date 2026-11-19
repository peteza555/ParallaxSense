#pragma once
// ข้อความจาก sensor node -> gateway ผ่าน ESP-NOW
// ใช้ร่วมกันทั้ง src/node_a และ src/gateway (อยู่ที่ include/ ไฟล์เดียว)
// ถ้าแก้ struct ให้เพิ่ม PACKET_VERSION แล้ว upload ใหม่ทั้งสองบอร์ด
#include <stdint.h>

#define PACKET_VERSION 1

enum NodeId : uint8_t {
  NODE_A = 1,  // คอ/ศีรษะ: MPU6050 + เซนเซอร์แสง
  NODE_B = 2,  // เก้าอี้: FSR + IMU (ทำหลัง checkpoint)
};

enum PacketFlags : uint8_t {
  FLAG_IMU_OK = 1 << 0,
  FLAG_LIGHT_OK = 1 << 1,
  FLAG_FSR_OK = 1 << 2,
};

typedef struct __attribute__((packed)) {
  uint8_t  version;       // = PACKET_VERSION
  uint8_t  node_id;       // NodeId: ใช้แยก node ที่ gateway และ dashboard (Scalability)
  uint8_t  flags;         // PacketFlags: sensor ตัวไหนอ่านได้จริงในรอบนี้
  uint32_t seq;           // เลขลำดับ ใช้คำนวณ packet loss / PDR และทำ ACK ใน E4
  uint32_t uptime_ms;     // millis() ของ node
  float    pitch_deg;     // มุมก้ม/เงย จาก complementary filter
  float    roll_deg;      // มุมเอียงซ้าย/ขวา
  float    gyro_mag_dps;  // ความเร็วเชิงมุมรวม ใช้ดูว่าขยับตัวอยู่หรือไม่
  float    light_pct;     // ความสว่าง 0-100% (ค่าสัมพัทธ์จากเซนเซอร์แสง analog ไม่ใช่ lux)
  float    fsr;           // Node B ใช้ (Node A ส่ง 0 และไม่ตั้ง FLAG_FSR_OK)
} NodePacket;

static_assert(sizeof(NodePacket) <= 250, "ESP-NOW payload must be <= 250 bytes");
