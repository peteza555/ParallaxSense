// ParallaxSense — กล่อง Gateway (draft v0.1)
//
// ใส่: ESP32 DevKit (30 ขา), HC-SR04, PIR HC-SR501, Servo SG90 (บนฝา), ปุ่มกด 12 mm (บนฝา)
// ขนาดทุกค่าเป็น "ค่ามาตรฐานทั่วไป" — วัดของจริงแล้วแก้ตัวแปรด้านล่างก่อนพิมพ์
//
// export:
//   openscad -D part=\"box\"  -o gateway_box.stl  gateway_box.scad
//   openscad -D part=\"lid\"  -o gateway_lid.stl  gateway_box.scad
//   openscad -D part=\"flag\" -o gateway_flag.stl gateway_box.scad
// พิมพ์: PLA, layer 0.2 mm, infill 15-20%, กล่องวางปากขึ้น, ฝาวางด้านบนคว่ำลง (ไม่ต้องใช้ support)

part = "assembly";  // "box" | "lid" | "flag" | "assembly" / "inside" (ภาพประกอบ ไม่ใช่ไว้พิมพ์)
$fn = 64;

// ---------- โครงกล่อง ----------
wall    = 2.5;
floor_t = 2.5;
lid_t   = 2.5;
in_x = 80;   // ความกว้างภายใน (ซ้าย-ขวา)
in_y = 65;   // ความลึกภายใน (หน้า-หลัง), หน้า = ฝั่งที่หันหาผู้ใช้ (y = 0)
in_z = 55;   // ความสูงภายใน
clr  = 0.4;  // ระยะเผื่อให้ฝาสวมได้ (ถ้าฝาแน่นไป เพิ่มเป็น 0.5-0.6)

// เสามุมสำหรับสกรูยึดฝา (M3 x 8 หรือสกรูเกลียวปล่อย 3 mm)
post   = 5.5;
post_h = 2.6;  // รูนำเจาะในเสา (เกลียวปล่อย)
lid_h  = 3.4;  // รูทะลุบนฝา

// ---------- HC-SR04 (ผนังหน้า) ----------
us_d     = 16 + 1.0;  // ตัวส่ง/รับ Ø16 + เผื่อ
us_pitch = 26;        // ระยะห่างศูนย์กลางสองตา  << วัดจริง
us_z     = 14;        // ความสูงศูนย์กลางตา จากพื้นด้านใน
us_pcb   = [45, 20];

// ---------- PIR HC-SR501 (ผนังหน้า เหนือ HC-SR04) ----------
pir_d   = 23 + 0.8;   // โดมเลนส์ Ø23  << วัดจริง
pir_z   = 40;
pir_pcb = [32, 24];

// ---------- ESP32 DevKit (วางบนแท่นด้านหลัง, หัว USB ชิดผนังซ้าย) ----------
esp_len  = 52;      // << วัดจริง (บางรุ่น 55)
esp_w    = 28.5;    // << วัดจริง
esp_x0   = 6;       // ระยะจากผนังซ้ายถึงขอบบอร์ดฝั่ง USB
esp_yc   = in_y - 3.5 - esp_w / 2;
cradle_h = 22;      // ยกบอร์ดสูง ให้มีที่ว่างใต้บอร์ดสำหรับหัว jumper ตัวเมีย
pcb_t    = 1.6;
usb_w = 12;  usb_h = 8;          // ช่องเสียบสาย micro-USB
usb_z = cradle_h + pcb_t + 1.5;  // ศูนย์กลางหัว USB

// ---------- Servo SG90 (บนฝา ด้านหลังขวา) ----------
sv_body  = [12.4, 22.8];  // ช่องตัว servo (กว้าง x ยาว)
sv_screw = 27.6;          // ระยะรูสกรูหูยึด
sv_c     = [65, 45];      // ตำแหน่งศูนย์กลางบนฝา

// ---------- ปุ่มหยุดฉุกเฉิน (บนฝา ด้านหน้าขวา) ----------
btn_d = 12.2;             // ปุ่ม panel-mount 12 mm  << วัดจริง
btn_c = [69, 14];

// ======================================================================
module rounded_box(size, r = 2) {
  hull() for (x = [r, size[0] - r], y = [r, size[1] - r])
    translate([x, y, 0]) cylinder(r = r, h = size[2]);
}

post_xy = [[0, 0], [in_x - post, 0], [0, in_y - post], [in_x - post, in_y - post]];

module box() {
  difference() {
    translate([-wall, -wall, -floor_t])
      rounded_box([in_x + 2 * wall, in_y + 2 * wall, floor_t + in_z]);
    // ช่องภายใน
    cube([in_x, in_y, in_z + 1]);
    // HC-SR04: สองตา
    for (s = [-1, 1])
      translate([in_x / 2 + s * us_pitch / 2, 1, us_z]) rotate([90, 0, 0])
        cylinder(d = us_d, h = wall + 2);
    // PIR: โดม
    translate([in_x / 2, 1, pir_z]) rotate([90, 0, 0]) cylinder(d = pir_d, h = wall + 2);
    // USB ผนังซ้าย
    translate([-wall - 1, esp_yc - usb_w / 2, usb_z - usb_h / 2]) cube([wall + 2, usb_w, usb_h]);
    // ช่องร้อยสายด้านหลัง (เผื่อสาย servo/อื่น ๆ ออกนอกกล่อง)
    translate([in_x / 2 - 6, in_y - 1, 3]) cube([12, wall + 2, 6]);
  }
  // เสามุม
  for (p = post_xy) translate([p[0], p[1], 0]) difference() {
    cube([post, post, in_z]);
    translate([post / 2, post / 2, in_z - 12]) cylinder(d = post_h, h = 13);
  }
  // แท่นวาง ESP32: รองขอบสั้นสองด้าน (ระหว่างแถว pin)
  for (xe = [esp_x0, esp_x0 + esp_len - 4]) {
    translate([xe, esp_yc - 9, 0]) cube([4, 18, cradle_h]);
  }
  // ขอบกันบอร์ดเลื่อน ฝั่งปลายเสาอากาศ
  translate([esp_x0 + esp_len + 0.3, esp_yc - 9, 0]) cube([2, 18, cradle_h + pcb_t + 2]);
}

module lid() {
  difference() {
    union() {
      rounded_box([in_x + 2 * wall, in_y + 2 * wall, lid_t]);
      // ขอบสวมด้านใน (เว้นมุมที่มีเสา)
      translate([wall + clr, wall + clr, lid_t]) difference() {
        cube([in_x - 2 * clr, in_y - 2 * clr, 4]);
        translate([1.5, 1.5, -1]) cube([in_x - 2 * clr - 3, in_y - 2 * clr - 3, 6]);
        for (p = post_xy) translate([p[0] - clr - 0.5, p[1] - clr - 0.5, -1])
          cube([post + 1, post + 1, 6]);
      }
    }
    // รูสกรูมุม
    for (p = post_xy) translate([wall + p[0] + post / 2, wall + p[1] + post / 2, -1])
      cylinder(d = lid_h, h = lid_t + 2);
    // ช่อง servo + รูสกรูหู
    translate([wall + sv_c[0], wall + sv_c[1], -1]) {
      translate([-sv_body[0] / 2, -sv_body[1] / 2, 0]) cube([sv_body[0], sv_body[1], lid_t + 6]);
      for (s = [-1, 1]) translate([0, s * sv_screw / 2, 0]) cylinder(d = 2, h = lid_t + 2);
    }
    // ปุ่ม
    translate([wall + btn_c[0], wall + btn_c[1], -1]) cylinder(d = btn_d, h = lid_t + 6);
  }
}

// ธง: ติดกับแขน servo (horn แบบแขนเดียว) ด้วยกาวหรือสกรูที่แถมมากับ servo
module flag() {
  cube([4, 2, 55]);                                 // เสา
  translate([4, 0, 33]) cube([26, 1.6, 20]);        // ผืนธง
  translate([-3, 0, 0]) difference() {              // แป้นยึด horn
    cube([10, 2, 18]);
    translate([5, 3, 5])  rotate([90, 0, 0]) cylinder(d = 1.6, h = 5);
    translate([5, 3, 12]) rotate([90, 0, 0]) cylinder(d = 1.6, h = 5);
  }
}

// ---------- ภาพประกอบ: ตำแหน่งอุปกรณ์แบบคร่าว ๆ เพื่อตรวจว่าไม่ชนกัน ----------
module dummies() {
  // HC-SR04
  color("RoyalBlue") translate([in_x / 2 - us_pcb[0] / 2, 10, us_z - us_pcb[1] / 2])
    cube([us_pcb[0], 1.6, us_pcb[1]]);
  color("Silver") for (s = [-1, 1])
    translate([in_x / 2 + s * us_pitch / 2, 10, us_z]) rotate([90, 0, 0]) cylinder(d = 16, h = 12);
  // PIR
  color("ForestGreen") translate([in_x / 2 - pir_pcb[0] / 2, 8, pir_z - pir_pcb[1] / 2])
    cube([pir_pcb[0], 1.6, pir_pcb[1]]);
  color("White") translate([in_x / 2, 8, pir_z]) rotate([90, 0, 0]) cylinder(d = 23, h = 9);
  // ESP32
  color("DimGray") translate([esp_x0, esp_yc - esp_w / 2, cradle_h]) cube([esp_len, esp_w, pcb_t]);
  color("Silver") translate([esp_x0 - 1, esp_yc - 4, cradle_h + pcb_t]) cube([6, 8, 3]);
  // SG90 ห้อยลงจากฝา
  color("SteelBlue") translate([sv_c[0] - 6.1, sv_c[1] - 11.25, in_z - 22.7]) cube([12.2, 22.5, 22.7]);
  // ปุ่ม
  color("Red") translate([btn_c[0], btn_c[1], in_z - 18]) cylinder(d = 11, h = 18);
}

if (part == "box") box();
else if (part == "lid") lid();
else if (part == "flag") flag();
else if (part == "inside") { color("Gainsboro") box(); dummies(); }  // ไม่มีฝา ดูการจัดวางข้างใน
else {
  color("Gainsboro", 0.35) box();
  dummies();
  // ฝาลอยขึ้นด้านบน (exploded view)
  color("LightSkyBlue", 0.8) translate([-wall, -wall, in_z + 25]) lid();
  color("Gold") translate([sv_c[0] - 2, sv_c[1], in_z + 25 + lid_t + 4]) flag();
}
