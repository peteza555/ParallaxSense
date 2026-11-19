# ParallaxSense — Checkpoint (PlatformIO)

```
Node A (ESP32: MPU6050 + เซนเซอร์แสง) --ESP-NOW--> Gateway (ESP32: HC-SR04) --MQTT--> NETPIE2020 --> Freeboard dashboard
```

## โครงสร้าง

| Path | ทำอะไร |
|---|---|
| `platformio.ini` | env `gateway`, `node_a` (ESP32) และ `native` (unit test บนคอม) |
| `src/node_a/main.cpp` | Node A: driver MPU6050 ระดับ register, pitch/roll, เซนเซอร์แสง, ส่ง ESP-NOW 5 Hz |
| `src/gateway/main.cpp` | Gateway: รับ ESP-NOW, อ่าน ultrasonic, คำนวณ PDR, publish ขึ้น NETPIE ทุก 1 วินาที |
| `include/packet.h` | Message format node → gateway (ใช้ร่วมกันทั้งสองบอร์ด) |
| `include/secrets.h.example` | แม่แบบรหัส Wi-Fi / NETPIE / MAC ของ gateway |
| `netpie/device_schema.json` | Device Schema ของ NETPIE ให้เก็บค่าลง Feed (raw data สำหรับ M5) |
| `test/test_gateway/` | Unit test (Unity) ของ logic gateway |
| `.github/workflows/ci.yml` | CI: unit test + build firmware + สแกน secret |

## ติดตั้ง

1. VS Code → Extensions → ติดตั้ง **PlatformIO IDE**
2. **File → Open Folder** เลือกโฟลเดอร์นี้ (ครั้งแรกจะดาวน์โหลด ESP32 platform ใช้เวลาหลายนาที ต้องต่อเน็ต)
3. คัดลอก `include/secrets.h.example` เป็น `include/secrets.h` แล้วใส่ค่า Wi-Fi และ NETPIE
   ถ้าลืมขั้นนี้ build จะขึ้น `Missing include/secrets.h`

## ต่อสาย

**Node A**

| ขา | ESP32 |
|---|---|
| MPU6050 VCC / GND | 3V3 / GND |
| MPU6050 SDA / SCL | D21 / D22 |
| MPU6050 AD0 | GND (address 0x68) |
| เซนเซอร์แสง VCC (`+`) / GND (`-`) | 3V3 / GND (ห้ามใช้ 5V) |
| เซนเซอร์แสง ขาสัญญาณ (`DO` หรือ `S`) | D34 (อ่านแบบ analog) |

ตั้งค่าเซนเซอร์แสงใน `platformio.ini` ส่วน `[env:node_a]`:
- ยังไม่ได้ต่อ → `LIGHT_ENABLED=0`
- เอามือบังแล้วค่ากลับด้าน → `LIGHT_INVERT=1`
- อยากอ่านแบบ digital (สว่าง/มืด) ที่ขาอื่น → `LIGHT_PIN=19` และ `LIGHT_DIGITAL_ONLY=1`

**Gateway** (HC-SR04 ส่ง ECHO ออกมา 5V ต้องแบ่งแรงดันก่อนเข้า ESP32; ทุกขาอยู่ฝั่งเดียวกับ VIN)

| HC-SR04 | ESP32 |
|---|---|
| VCC | VIN (5V, ต้องเสียบ USB) |
| GND | GND |
| TRIG | D14 |
| ECHO | ผ่าน R 1kΩ ไป D13 และจาก D13 ผ่าน R 2kΩ ลง GND |

## ตั้งค่า NETPIE2020

1. https://portal.netpie.io → สร้าง Project → Device Group → Device ชื่อ `parallaxsense-gw`
2. copy **Client ID / Token / Secret** ไปใส่ `include/secrets.h`
3. แท็บ **Schema** → วางเนื้อหา `netpie/device_schema.json` แล้ว Save
4. **Freeboard** → เพิ่ม datasource เป็น device นี้ → เพิ่ม widget:
   Gauge `a_pitch`, Gauge `gw_dist_cm`, Indicator `a_light` (สว่าง/มืด), Text `a_pdr`, Indicator `a_online`

## Upload

เสียบบอร์ดทีละตัว (หรือใส่ `upload_port` ของแต่ละ env ใน `platformio.ini` ถ้าเสียบพร้อมกัน)

1. **Gateway ก่อน**: แถบล่างของ VS Code เลือก `env:gateway` → Upload → Serial Monitor
   จะเห็นบรรทัด `#define GATEWAY_MAC {0x..., ...}` ให้ copy ไปแทนบรรทัด `GATEWAY_MAC` ใน `include/secrets.h`
2. **Node A**: เลือก `env:node_a` → Upload → Serial Monitor

หรือใช้คำสั่ง:
```bash
pio run -e gateway -t upload && pio device monitor -e gateway
pio run -e node_a  -t upload && pio device monitor -e node_a
```

Serial ของ gateway ควรเห็นประมาณนี้:
```
[WiFi] OK ip=172.20.10.3 channel=6 rssi=-41
[NOW] ready, waiting for nodes ...
[RX] node A seq=35 pitch=-21.4 roll=2.0 gyro=1.3 light=100% rssi=-38 PDR=100.0%
[MQTT] published #12 {"data":{"gw_dist_cm":48.6,"gw_uptime_s":14,"a_online":1,...}}
```

## แก้ปัญหาที่เจอบ่อย

| อาการ | สาเหตุที่น่าจะเป็น |
|---|---|
| Upload ขึ้น `Failed to connect to ESP32` | กดปุ่ม BOOT ค้างไว้ตอนขึ้น `Connecting...`; หรือยังไม่ได้ลง driver CP210x / CH340 |
| หาพอร์ตไม่เจอ | สาย USB เป็นแบบชาร์จอย่างเดียว ลองเปลี่ยนสาย |
| Node ขึ้น `SSID not found` | Hotspot ปิด หรือเป็น 5 GHz (iPhone เปิด *Maximize Compatibility*) |
| Node `sent fail` ตลอด | `GATEWAY_MAC` ผิด หรือ gateway ยังต่อ Wi-Fi ไม่ได้ |
| `[MQTT] FAIL rc=4` หรือ `rc=5` | Client ID / Token / Secret ผิด |
| `[I2C] MPU6050 FAIL` | SDA/SCL สลับ, ไม่ได้ต่อ 3V3, ขา header ไม่ได้บัดกรี |
| ไม่มี `gw_dist_cm` ใน payload | TRIG/ECHO สลับ, ไม่ได้จ่าย 5V, วัตถุใกล้กว่า 2 cm |
| ค่า `light` กระโดดมั่ว | ขาสัญญาณไม่ได้อยู่ D34 หรือยังไม่ได้ต่อ (ตั้ง `LIGHT_ENABLED=0`) |

## Tests และ CI

```bash
pio test -e native     # unit test logic ของ gateway: PDR, online/offline, JSON ที่ส่งขึ้น NETPIE
```

ทุกครั้งที่ push, GitHub Actions จะรัน unit test, build firmware ทั้งสองบอร์ด และสแกนหา secret ที่หลุดเข้า git

## งานหลัง checkpoint

- Node B (FSR + IMU): เพิ่ม `src/node_b/` และ `[env:node_b]` ใช้ `include/packet.h` เดิม ตั้ง `node_id = NODE_B`
- E4: gateway ส่ง ACK ตาม `seq` + buffer ลง LittleFS ตอน NETPIE หลุด
- E5: เปลี่ยนเป็น `WiFiClientSecure` + พอร์ต 1884 (MQTTS ของ NETPIE)
- E1: แยก sampling / filter / comms / watchdog เป็น FreeRTOS tasks บน Node A
- E2: ใช้ขา INT ของ MPU6050 เป็น interrupt แทนการวน poll
- E6: เพิ่ม env OTA (`upload_protocol = espota`)
