// ParallaxSense — กล่อง Node A (สวมคอ), Node B (เก้าอี้) และแผ่นรอง FSR (draft v0.1)
//
// ขนาดทุกค่าเป็น "ค่ามาตรฐานทั่วไป" — วัดของจริงแล้วแก้ตัวแปรที่มี << ก่อนพิมพ์
//
// export (เปลี่ยน part):
//   openscad -D part=\"a_box\"   -o node_a_box.stl   node_box.scad
//   openscad -D part=\"a_lid\"   -o node_a_lid.stl   node_box.scad
//   openscad -D part=\"b_box\"   -o node_b_box.stl   node_box.scad
//   openscad -D part=\"b_lid\"   -o node_b_lid.stl   node_box.scad
//   openscad -D part=\"fsr_base\" -o fsr_base.stl    node_box.scad   (พิมพ์ 2 ชุด ซ้าย/ขวา)
//   openscad -D part=\"fsr_top\"  -o fsr_top.stl     node_box.scad
//   openscad -D part=\"fsr_clip\" -o fsr_clip.stl    node_box.scad   (ฝาครอบหางสาย)
// พิมพ์: PLA, layer 0.2 mm, infill 20%, กล่องวางปากขึ้น, ฝาคว่ำ — ไม่ต้องใช้ support
//
// การสวม Node A: พื้นกล่องแนบหลังคอ, ร้อยสายรัด/สายคล้องกว้าง ≤ 25 mm ผ่านหูสองข้าง
//   ด้านยาวฝั่งที่มีรูแสง (y = in_y) ให้หันขึ้นด้านบน

part = "assembly";  // a_box | a_lid | b_box | b_lid | fsr_base | fsr_top | fsr_clip | assembly
$fn = 48;

// ---------- โครงกล่อง ----------
wall = 2.0;  floor_t = 2.0;  lid_t = 2.0;
in_x = 86;   // ยาว: ESP32 + MPU6050 เรียงต่อกัน
in_y = 38;   // กว้าง: ESP32 + ช่องเดินสาย/โมดูลแสงด้านบน
in_z = 27;   // สูง: ใต้บอร์ด ESP32 เว้นที่ให้หัว jumper ตัวเมีย
clr  = 0.4;
post = 5;  post_h = 2.5;  lid_h = 3.2;   // เสามุม + สกรู M3 / เกลียวปล่อย 3 mm

// ---------- ESP32 DevKit 30 ขา ----------
esp_len = 52;     // << วัดจริง
esp_w   = 28.5;   // << วัดจริง
esp_x0  = 4;      // ระยะจากผนังซ้าย (ฝั่ง USB)
esp_yc  = 1.5 + esp_w / 2;
cradle_h = 21;
pcb_t = 1.6;
usb_w = 12;  usb_h = 8;  usb_z = cradle_h + pcb_t + 1.5;

// ---------- MPU6050 GY-521 (ยึดสกรู 2 รู กับเสาบนพื้น) ----------
mpu   = [21.2, 16.4];   // << วัดจริง
mpu_x0 = esp_x0 + esp_len + 5;
mpu_standoff = 4;
mpu_hole_d   = 2.5;     // รูนำสำหรับสกรู M3 เกลียวปล่อย (รูบนบอร์ด Ø3)
mpu_hole_in  = 2.6;     // ระยะศูนย์กลางรูจากขอบบอร์ด  << วัดจริง
// รูสองรูอยู่ปลายบอร์ดฝั่ง +x (ฝั่งตรงข้ามหัว pin)

// ---------- เซนเซอร์แสง (Node A): รู LDR บนผนังด้านบน ----------
ldr_d = 5.6;   // LDR Ø5 mm
ldr_x = 30;  ldr_z = 13;

// ---------- Node B: ช่องสาย FSR 2 เส้น ----------
fsr_slot = [14, 6];

// ---------- หูร้อยสาย ----------
lug_len = 7;  strap_w = 26;  strap_t = 3.5;

// ---------- FSR406 ----------
fsr_sz   = 45.6;   // วัดจริง 45 mm + เผื่อ 0.3 mm ต่อด้าน
fsr_tail = 9;      // ความกว้างหาง
pad_sz   = 62;  pad_t = 3;
puck_d   = 30;  puck_h = 1.2;
fsr_t    = 0.45;   // ความหนาแผ่น FSR  << วัดจริง
// เสาสกรูมุม: แผ่นบนสวมเสา ลอยขึ้นลงได้ แต่ถูกแหวนรองใต้หัวสกรูกันไม่ให้หลุด
//   ขันสกรูแน่นได้เต็มที่ เพราะสกรูกดหัวเสา ไม่ได้กดแผ่นบน -> เซนเซอร์ไม่ถูกกดค้าง
//   ใช้ได้ทั้งวางเบาะและติดพนักพิง (แนวตั้ง)
top_gap    = puck_h - (0.8 - fsr_t);   // ช่องว่างใต้แผ่นบน ≈ 0.85
post_d     = 5;    post_float = 0.5;   // ระยะที่แผ่นบนขยับขึ้นได้
fpost_h    = top_gap + pad_t + post_float;   // เสาสูงเหนือผิว base
post_pilot = 2.5;  // รูนำสกรูเกลียวปล่อย M3
top_hole   = post_d + 0.6;
// ส่วนป้องกันหางสาย: ลิ้นยื่นจาก base + ฝาครอบ (clip) + ช่องเข็มขัดรัดสาย
//   หางยาวรวมจุดต่อสาย 40 mm: ฟิล์มอยู่ในร่องตื้น, จุดต่อ/บัดกรีอยู่ในโพรงลึกใต้ clip,
//   zip tie รัดเฉพาะสายไฟถัดจาก clip ออกไป (ไม่รัดบนฟิล์ม)
tail_len   = 40;   // หางทั้งหมดรวมส่วนต่อสาย  (วัดจริง)
conn_len   = 10;   // ช่วงปลายหางที่เป็นจุดต่อสาย/บัดกรี  << วัดจริง
conn_extra = 6;    // ให้ clip ครอบเลยปลายหางไปอีก (ปลอกหด/หัวบัดกรี)
conn_h     = 3.2;  // ความสูงโพรงจุดต่อ = ร่องในลิ้น 1.8 + ร่องใต้ clip 1.4
pocket_d   = 1.8;
clip_t     = 3;
tongue_w   = 24;
pin_dx     = fsr_tail / 2 + 3.5;   // หมุดยึด clip อยู่สองข้างร่อง
zip_slot   = [2.2, 4.5];           // ช่องเข็มขัดรัดสาย (zip tie กว้าง ≤ 3.6 mm)
wire_ch    = [6, 1.0];             // ร่องวางสายไฟช่วง zip tie (กว้าง, ลึก)

tail_y0    = (pad_sz - fsr_sz) / 2 + 0.3;   // ขอบเซนเซอร์ฝั่งหาง
tail_end_y = tail_y0 - tail_len;            // ปลายหาง
pocket_y1  = tail_end_y + conn_len + 1;     // จุดเริ่มโพรงจุดต่อ
clip_y0    = tail_end_y - conn_extra;       // ปลาย clip
zip_y0     = clip_y0 - 2 - zip_slot[1];
tongue_len = -zip_y0 + 3;
clip_pin_y = [-5, clip_y0 + 4];

// ======================================================================
module rbox(s, r = 2) {
  hull() for (x = [r, s[0] - r], y = [r, s[1] - r]) translate([x, y, 0]) cylinder(r = r, h = s[2]);
}
post_xy = [[0, 0], [in_x - post, 0], [0, in_y - post], [in_x - post, in_y - post]];
mpu_yc = esp_yc;

module lug() {  // หูร้อยสายรัด ติดปลายกล่อง
  difference() {
    translate([0, -wall, -floor_t]) cube([lug_len, in_y + 2 * wall, floor_t + 6]);
    translate([2, (in_y - strap_w) / 2, -floor_t - 1]) cube([strap_t, strap_w, 10]);
  }
}

module node_box(kind) {
  difference() {
    union() {
      translate([-wall, -wall, -floor_t]) rbox([in_x + 2 * wall, in_y + 2 * wall, floor_t + in_z]);
      translate([-wall - lug_len + 0.5, 0, 0]) lug();
      translate([in_x + wall - 0.5, 0, 0]) lug();
    }
    cube([in_x, in_y, in_z + 1]);
    // USB ผนังซ้าย (ตัดผ่านหูด้วย)
    translate([-wall - lug_len - 1, esp_yc - usb_w / 2, usb_z - usb_h / 2])
      cube([wall + lug_len + 2, usb_w, usb_h]);
    if (kind == "a")  // รู LDR ผนังด้านบน
      translate([ldr_x, in_y - 1, ldr_z]) rotate([-90, 0, 0]) cylinder(d = ldr_d, h = wall + 2);
    if (kind == "b")  // ช่องสาย FSR ผนังด้านบน
      translate([in_x - 30, in_y - 1, 3]) cube([fsr_slot[0], wall + 2, fsr_slot[1]]);
  }
  for (p = post_xy) translate([p[0], p[1], 0]) difference() {
    cube([post, post, in_z]);
    translate([post / 2, post / 2, in_z - 10]) cylinder(d = post_h, h = 11);
  }
  // แท่นวาง ESP32
  for (xe = [esp_x0, esp_x0 + esp_len - 4]) translate([xe, esp_yc - 9, 0]) cube([4, 18, cradle_h]);
  translate([esp_x0 + esp_len + 0.3, esp_yc - 9, 0]) cube([2, 18, cradle_h + pcb_t + 2]);
  // เสา MPU6050: สองเสามีรูสกรู + หนึ่งแท่งรองฝั่งหัว pin
  for (s = [-1, 1])
    translate([mpu_x0 + mpu[0] - mpu_hole_in, mpu_yc + s * (mpu[1] / 2 - mpu_hole_in), 0]) difference() {
      cylinder(d = 5.5, h = mpu_standoff);
      cylinder(d = mpu_hole_d, h = mpu_standoff + 1);
    }
  translate([mpu_x0 + 1, mpu_yc - 5, 0]) cube([2.5, 10, mpu_standoff]);
}

module node_lid() {
  difference() {
    union() {
      rbox([in_x + 2 * wall, in_y + 2 * wall, lid_t]);
      translate([wall + clr, wall + clr, lid_t]) difference() {
        cube([in_x - 2 * clr, in_y - 2 * clr, 3]);
        translate([1.2, 1.2, -1]) cube([in_x - 2 * clr - 2.4, in_y - 2 * clr - 2.4, 5]);
        for (p = post_xy) translate([p[0] - clr - 0.5, p[1] - clr - 0.5, -1]) cube([post + 1, post + 1, 5]);
      }
    }
    for (p = post_xy) translate([wall + p[0] + post / 2, wall + p[1] + post / 2, -1])
      cylinder(d = lid_h, h = lid_t + 2);
  }
}

// แผ่นรอง FSR: base มีร่องวางแผ่น + ร่องหาง + เสาสกรูมุม, top มีปุ่มนูนกดลงกลางแผ่น
pin_xy = [[5, 5], [pad_sz - 5, 5], [5, pad_sz - 5], [pad_sz - 5, pad_sz - 5]];
module fsr_base() {
  cx = pad_sz / 2;
  difference() {
    union() {
      rbox([pad_sz, pad_sz, pad_t], 3);
      translate([cx - tongue_w / 2, -tongue_len, 0]) rbox([tongue_w, tongue_len + 3, pad_t], 3);
    }
    translate([(pad_sz - fsr_sz) / 2, (pad_sz - fsr_sz) / 2, pad_t - 0.8]) cube([fsr_sz, fsr_sz, 1]);
    // ร่องฟิล์มหาง (ตื้น 0.8)
    translate([cx - fsr_tail / 2, pocket_y1 - 1, pad_t - 0.8]) cube([fsr_tail, tail_y0 - pocket_y1 + 2, 1]);
    // โพรงจุดต่อสาย (ลึก pocket_d) ถึงปลาย clip
    translate([cx - fsr_tail / 2, clip_y0 - 0.01, pad_t - pocket_d]) cube([fsr_tail, pocket_y1 - clip_y0, pocket_d + 1]);
    // ร่องสายไฟช่วง zip tie จนสุดลิ้น
    translate([cx - wire_ch[0] / 2, -tongue_len - 1, pad_t - wire_ch[1]]) cube([wire_ch[0], tongue_len + clip_y0 + 2, wire_ch[1] + 1]);
    // ช่อง zip tie + ร่องใต้ลิ้นให้ตัวรัดจม แผ่นไม่โยก
    for (s = [-1, 1]) translate([cx + s * pin_dx - zip_slot[0] / 2, zip_y0, -1]) cube([zip_slot[0], zip_slot[1], pad_t + 2]);
    translate([cx - pin_dx, zip_y0, -1]) cube([2 * pin_dx, zip_slot[1], 1.8]);
  }
  for (p = pin_xy) translate([p[0], p[1], 0]) difference() {
    cylinder(d = post_d, h = pad_t + fpost_h);
    translate([0, 0, pad_t + fpost_h - 6]) cylinder(d = post_pilot, h = 7);
  }
  for (s = [-1, 1], y = clip_pin_y) translate([cx + s * pin_dx, y, pad_t]) cylinder(d = 3, h = 2.5);
}
// ฝาครอบหางสาย: สวมหมุด 4 ตัวบนลิ้น (ทากาวได้) ครอบฟิล์ม + จุดต่อสายทั้งหมด
// ร่องฟิล์มลึก 0.8 > หาง ~0.45 mm จึงไม่บีบฟิล์ม; ช่วงจุดต่อมีที่ว่างสูง conn_h
module fsr_clip() {
  len = -clip_y0;
  difference() {
    rbox([tongue_w, len, clip_t], 2);
    for (s = [-1, 1], y = clip_pin_y) translate([tongue_w / 2 + s * pin_dx, y - clip_y0, -1])
      cylinder(d = 3.2, h = clip_t + 2);   // 3.2 ให้สวมแน่นกับหมุด Ø3
    translate([tongue_w / 2 - fsr_tail / 2, -1, -1]) cube([fsr_tail, pocket_y1 - clip_y0 + 1, 1 + conn_h - pocket_d]);
  }
}
module fsr_top() {
  difference() {
    rbox([pad_sz, pad_sz, pad_t], 3);
    for (p = pin_xy) translate([p[0], p[1], -1]) cylinder(d = top_hole, h = pad_t + 2);
  }
  translate([pad_sz / 2, pad_sz / 2, -puck_h]) cylinder(d = puck_d, h = puck_h);  // พิมพ์โดยคว่ำ ปุ่มอยู่ด้านบน
}

// ---------- ภาพประกอบ ----------
module dummies(kind) {
  color("DimGray") translate([esp_x0, esp_yc - esp_w / 2, cradle_h]) cube([esp_len, esp_w, pcb_t]);
  color("Silver") translate([esp_x0 - 1, esp_yc - 4, cradle_h + pcb_t]) cube([6, 8, 3]);
  color("MediumPurple") translate([mpu_x0, mpu_yc - mpu[1] / 2, mpu_standoff]) cube([mpu[0], mpu[1], pcb_t]);
  if (kind == "a") {
    color("Teal") translate([ldr_x - 7, in_y - 4.5, ldr_z - 8]) cube([14, 1.6, 20]);
    color("Orange") translate([ldr_x, in_y - 3, ldr_z]) rotate([-90, 0, 0]) cylinder(d = 5, h = 5);
  }
}

if      (part == "a_box")    node_box("a");
else if (part == "b_box")    node_box("b");
else if (part == "a_lid" || part == "b_lid") node_lid();
else if (part == "fsr_base") fsr_base();
else if (part == "fsr_top")  rotate([180, 0, 0]) fsr_top();
else if (part == "fsr_clip") fsr_clip();
else {
  // Node A
  color("Gainsboro") node_box("a");  dummies("a");
  color("LightSkyBlue", 0.85) translate([-wall, -wall, in_z + 18]) node_lid();
  // Node B
  translate([0, -75, 0]) { color("Wheat") node_box("b"); dummies("b"); }
  // แผ่นรอง FSR (base + top แยกให้เห็น)
  translate([120, -75, -floor_t]) {
    color("Wheat") fsr_base();
    color("Orange", 0.9) translate([(pad_sz - fsr_sz) / 2, (pad_sz - fsr_sz) / 2, pad_t - 0.6]) cube([fsr_sz, fsr_sz, 0.5]);
    color("Orange", 0.9) translate([pad_sz / 2 - fsr_tail / 2 + 0.5, tail_end_y, pad_t - 0.6])
      cube([fsr_tail - 1, tail_len, 0.4]);
    color("Silver") translate([pad_sz / 2 - 3.5, tail_end_y, pad_t - pocket_d]) cube([7, conn_len, 2.4]);
    for (s = [-1, 1]) color(s < 0 ? "Red" : "Black")
      translate([pad_sz / 2 + s * 1.5, tail_end_y + 4, pad_t - 0.5]) rotate([90, 0, 0]) cylinder(d = 1.5, h = tongue_len + tail_end_y + 12);
    color("Tan", 0.8) translate([0, 0, pad_t + 20]) fsr_top();
    for (p = pin_xy) color("Silver") translate([p[0], p[1], pad_t + fpost_h + 24]) {
      cylinder(d = 9, h = 0.8);                        // แหวนรอง M3 (OD ≥ 8)
      translate([0, 0, 0.8]) cylinder(d = 5.5, h = 2);  // หัวสกรู
    }
    color("Tan", 0.8) translate([(pad_sz - tongue_w) / 2, clip_y0, pad_t + 12]) fsr_clip();
  }
}
