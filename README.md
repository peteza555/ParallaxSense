# ParallaxSense — Checkpoint (PlatformIO)

```
Node A (ESP32: MPU6050 + BH1750) --ESP-NOW--> Gateway (ESP32: HC-SR04 + PIR + servo ธง) --MQTT--> NETPIE2020 --> Freeboard dashboard
```

## โครงสร้าง

| Path | ทำอะไร |
|---|---|
| `platformio.ini` | env `gateway`, `node_a` (ESP32) และ `native` (unit test บนคอม) |
| `src/node_a/main.cpp` | Node A: driver MPU6050 / BH1750 ระดับ register, calibrate ท่านั่ง (ปุ่ม BOOT), pitch/roll, ส่ง ESP-NOW 5 Hz |
| `src/gateway/main.cpp` | Gateway: รับ ESP-NOW, อ่าน ultrasonic + PIR (interrupt), ยกธง servo, คำนวณ PDR, publish ขึ้น NETPIE ทุก 1 วินาที |
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
| BH1750 VCC / GND | 3V3 / GND |
| BH1750 SDA / SCL | D21 / D22 (ต่อขนานกับ MPU6050 ได้เลย) |
| BH1750 ADDR | ไม่ต่อ หรือ GND (address 0x23) |

BH1750 ส่งค่าเป็น lux ขึ้น dashboard ชื่อ `a_lux` — ถ้ายังไม่ได้ต่อ firmware จะลองหาใหม่ทุก 5 วินาที (เสียบทีหลังได้)

ใช้โมดูล LDR 3 ขาแทน BH1750: ตั้ง `LIGHT_BH1750=0`, `LIGHT_ENABLED=1` ใน `platformio.ini` แล้วต่อขาสัญญาณที่ D34
(VCC 3V3 ห้าม 5V; เอามือบังแล้วค่ากลับด้าน → `LIGHT_INVERT=1`) ค่าจะขึ้นเป็น `a_light` 0-100%

**Gateway** (ทุกขาอยู่ฝั่งเดียวกับ VIN; VIN/GND ใช้ร่วมกันผ่านราง breadboard)

| อุปกรณ์ | ขาอุปกรณ์ | ESP32 |
|---|---|---|
| HC-SR04 | VCC / GND | VIN (5V, ต้องเสียบ USB) / GND |
| HC-SR04 | TRIG | D14 |
| HC-SR04 | ECHO | ผ่าน R 1kΩ ไป D13 และจาก D13 ผ่าน R 2kΩ ลง GND (ECHO ออก 5V ต้องแบ่งแรงดัน) |
| PIR HC-SR501 | VCC / GND | VIN / GND |
| PIR HC-SR501 | OUT | D27 (OUT เป็น 3.3V ต่อตรงได้) — jumper ที่ H, หมุน Time ไปน้อยสุด |
| Servo SG90 | แดง / น้ำตาล | VIN / GND |
| Servo SG90 | ส้ม (สัญญาณ) | D26 |

ถ้า ESP32 รีเซ็ตตอน servo หมุน (ไฟ USB ตก) ให้ใส่ตัวเก็บประจุ 470-1000 µF คร่อม VIN-GND ใกล้ servo

ธงจะยกขึ้นเมื่อ **มีคนอยู่** (PIR เห็นการเคลื่อนไหวภายใน 30 s, ultrasonic < 120 cm หรือ AI เห็นคน) และ
**ก้ม/เงยเกิน 20°**, **ใกล้จอ < 40 cm** หรือ **AI จากกล้องบอกว่าท่าไม่ดี** ต่อเนื่อง 5 วินาที — ปรับค่าได้ที่ต้น `src/gateway/main.cpp`

## ตั้งค่า NETPIE2020

1. https://portal.netpie.io → สร้าง Project → Device Group → Device ชื่อ `parallaxsense-gw`
2. copy **Client ID / Token / Secret** ไปใส่ `include/secrets.h`
3. แท็บ **Schema** → วางเนื้อหา `netpie/device_schema.json` แล้ว Save
4. **Freeboard** → เพิ่ม datasource เป็น device นี้ → เพิ่ม widget:
   Gauge `a_pitch`, Gauge `gw_dist_cm`, Text `a_lux`, Text `a_pdr`, Indicator `a_online`,
   Indicator `gw_present` (มีคน), Indicator `gw_flag` (ธงเตือน)

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

## AI ท่านั่ง (USB webcam บน laptop)

USB webcam หนีบกลางขอบบนจอ ให้เห็นหัว + ไหล่ทั้งสองข้าง, แสงมาจากด้านหน้า
MediaPipe Pose (pre-trained) หาจุดบนร่างกาย → RandomForest ที่เทรนจากข้อมูลของผู้ใช้เองตัดสิน good/bad
ส่งแค่ผลลัพธ์ขึ้น NETPIE (`@msg/ai` และ shadow `ai_bad`, `ai_conf`) ไม่ส่งภาพ

```bash
python -m venv ai/.venv
ai/.venv/Scripts/python -m pip install -r ai/requirements.txt     # macOS/Linux: ai/.venv/bin/python

ai/.venv/Scripts/python ai/posture_ai.py cams            # หา index กล้อง (ใช้กับ --cam N)
ai/.venv/Scripts/python ai/posture_ai.py check           # กล้อง + MediaPipe ใช้ได้ไหม
ai/.venv/Scripts/python ai/posture_ai.py collect good    # นั่งตรง 60 วินาที
ai/.venv/Scripts/python ai/posture_ai.py collect bad     # ก้ม / ห่อไหล่ / เอียง / ชิดจอ 60 วินาที
ai/.venv/Scripts/python ai/posture_ai.py train           # accuracy + confusion matrix -> ai/model/metrics.json
ai/.venv/Scripts/python ai/posture_ai.py run             # ใช้งานจริง (q / ESC ออก)
```

- 1 รอบที่อัด = 1 ไฟล์ `ai/data/<ชื่อ git>/<เวลา>-good|bad.csv` (ข้อมูลเป็นพิกัดจุดบนร่างกาย ไม่มีภาพ)
- `train` ใช้ข้อมูลทุกคนใน `ai/data/` และถ้ามี 2 คนขึ้นไปจะบอก accuracy กับ "คนที่โมเดลไม่เคยเห็น" ด้วย

**ช่วยกันเก็บข้อมูล** (ทุกคนในกลุ่ม แต่ละคนอยู่ในโฟลเดอร์ของตัวเอง push พร้อมกันไม่ conflict):
```bash
git pull
ai/.venv/Scripts/python ai/posture_ai.py collect good      # 60 วินาที x 2 รอบ
ai/.venv/Scripts/python ai/posture_ai.py collect bad       # ก้ม / ห่อไหล่ / เอียง / ชิดจอ อย่างละ ~15 วินาที
git add ai/data/<ชื่อตัวเอง>/ && git commit -m "AI data: <ชื่อ>" && git push
```
ใครจะใช้งาน `run` ก็ `git pull` แล้ว `train` บนเครื่องตัวเอง (ไฟล์โมเดล `.joblib` และ `metrics.json` ไม่ขึ้น git)
- accuracy วัดจาก 20% ท้ายของแต่ละ session (ไม่สุ่มแบ่ง เพราะเฟรมติดกันเหมือนกันมาก จะได้ค่าสูงเกินจริง)
- ย้ายกล้อง/เปลี่ยนคน → collect + train ใหม่
- ส่งขึ้น NETPIE: สร้าง device `parallaxsense-ai` ใน Group เดียวกับ gateway, วาง `netpie/ai_device_schema.json`
  ในแท็บ Schema, คัดลอก `ai/.env.example` เป็น `ai/.env` แล้วใส่ key (ไฟล์นี้ไม่ขึ้น git)
- Gateway subscribe `@msg/ai`: AI บอกว่าท่าไม่ดีต่อเนื่อง 5 วินาที → ยกธง (closed loop, M4)
  ถ้าไม่ได้ยิน AI เกิน 5 วินาที (ปิด laptop) gateway ตัดผล AI ออกแล้วใช้ IMU + ultrasonic ต่อตามปกติ
  shadow ของ gateway มี `ai_online` / `ai_bad` ให้ dashboard ใช้ datasource เดียว

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
| `BH1750 not found` / `light=n/a` | SDA/SCL ไม่ได้ต่อร่วมกับ MPU6050 ที่ D21/D22 หรือไม่ได้จ่าย 3V3 |
| `PIR=0` ตลอด | OUT ไม่ได้อยู่ D27, หรือ PIR ยังอุ่นเครื่องไม่เสร็จ (รอ ~1 นาทีหลังจ่ายไฟ) |
| ESP32 รีเซ็ตตอนธงขยับ | ไฟ USB ไม่พอ ใส่ C 470-1000 µF คร่อม VIN-GND หรือแยกไฟ 5V ให้ servo (GND ต่อร่วมกัน) |

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
