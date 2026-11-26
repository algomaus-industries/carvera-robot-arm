// Bit carousel for the SO-101 cell: fabrication parts.
//
// Generated numbers (seat positions, sizes) come from carousel_params.scad,
// written by fab/gen_fab.py from models/bit_carousel/carousel_layout.py, so
// the real carousel matches the sim model seat for seat. Units: mm.
//
// Pick a part with -D 'part="seat_plate_0"' (see fab/gen_fab.py for the list):
//   2D, laser (xTool M2, 6 mm birch ply) or Carvera (3 mm end mill, 2.5D):
//     seat_plate_0, seat_plate_1   seat plate halves (seats, screw holes)
//     base_plate_0, base_plate_1   base plate halves (rollers, servo cradle)
//     splice_strip                 joins the seat plate halves under the seam (x2;
//                                  M3 screws and nuts, between the rings)
//   3D, Bambu X1 (PETG, 0.4 mm nozzle):
//     hub, rim_segment (x RIM_SEGMENTS), roller_bracket (x ROLLERS), servo_cradle
//   assembly                       everything in place, for a look

include <carousel_params.scad>

part = "assembly";
$fn = 64;

// ------------------------------------------------------------ fit / process
KERF = 0.2;          // laser kerf: outlines grow, holes shrink by half of it each side
FDM_CLEAR = 0.2;     // radial clearance on printed holes and pockets
M3_CLEAR = 3.4;      // M3 clearance hole
M3_INSERT = 4.2;     // M3 heat-set insert (4.0 mm OD, 5.7 mm long)
M3_INSERT_DEPTH = 6;
M4_WOOD = 4.5;       // M4 wood screw clearance
M8_CLEAR = 8.3;

// ------------------------------------------------------------ derived
PLATE_BOTTOM = SEAT_Z - PLATE_T;          // above the cart top
RIM_H = PLATE_BOTTOM - RIM_Z0;
HUB_H = PLATE_BOTTOM - HORN_TOP;
ROLLER_C = RIM_Z0 - BEARING_OD / 2;       // roller axle height above the cart top
HUB_SCREW_R = 15;
SEAM_SCREW_OFF = 4.5;                     // splice-strip screws either side of the seam,
                                          // between two rings (seats keep >= 7 mm clear)
SPLICE_W = 10;
SPLICE_R = [HUB_R + 4, RIM_R0 - 4];

function polar(r, a) = [r * cos(a), r * sin(a)];
function seg_angle(k) = SEAM_A + k * 360 / RIM_SEGMENTS;

// Half plane on the +side (0) or -side (1) of the seam line through the centre.
module half(i, r) {
  rotate(SEAM_A + (i == 0 ? 0 : 180)) translate([-r - 1, 0]) square([2 * r + 2, r + 1]);
}

// ------------------------------------------------------------ seat plate
module seat_plate_2d(i) {
  difference() {
    intersection() { circle(r = PLATE_R + KERF / 2, $fn = 360); half(i, PLATE_R + 5); }
    for (s = SEATS) translate(s) circle(d = HOLE_D - KERF, $fn = 32);
    // hub screws (into the hub's inserts)
    for (k = [0:3]) translate(polar(HUB_SCREW_R, SEAM_A + 45 + 90 * k)) circle(d = M3_CLEAR - KERF);
    // rim screws: two per rim segment
    for (k = [0:RIM_SEGMENTS - 1], d = [-1, 1])
      translate(polar((RIM_R0 + RIM_R1) / 2, seg_angle(k) + d * 360 / RIM_SEGMENTS / 4))
        circle(d = M3_CLEAR - KERF);
    // splice strip screws along the seam
    for (sgn = [-1, 1], r = SPLICE_SCREWS_R, off = [-1, 1])
      rotate(SEAM_A) translate([sgn * r, off * SEAM_SCREW_OFF]) circle(d = M3_CLEAR - KERF);
    // seat labels are engraved separately (seat_plate_engrave_*)
  }
}

// ------------------------------------------------------------ base plate
module base_plate_2d(i) {
  difference() {
    intersection() { circle(r = BASE_R + KERF / 2, $fn = 360); half(i, BASE_R + 5); }
    // roller brackets: two screws each
    for (k = [0:ROLLERS - 1], d = [-1, 1])
      rotate(ROLLER_A0 + k * 360 / ROLLERS) translate([ROLLER_R + d * 9, 0])
        circle(d = M3_CLEAR - KERF);
    // servo cradle: four screws
    for (sx = [-1, 1], sy = [0, 1])
      translate([sx * (SERVO_W / 2 + 7), -SHAFT_FROM_END + sy * SERVO_L])
        circle(d = M3_CLEAR - KERF);
    // servo cable through to the bus under the cart top
    translate([0, -SHAFT_FROM_END + SERVO_L + 12]) circle(d = 12 - KERF);
    // base to cart top: countersink these by hand (M4 wood screws)
    for (k = [0:5]) translate(polar(120, SEAM_A + 30 + 60 * k)) circle(d = M4_WOOD - KERF);
  }
}

module splice_strip_2d() {
  L = SPLICE_R[1] - SPLICE_R[0];
  difference() {
    translate([0, -SPLICE_W / 2]) square([L, SPLICE_W]);
    for (r = SPLICE_SCREWS_R, off = [-1, 1])
      translate([r - SPLICE_R[0], off * SEAM_SCREW_OFF]) circle(d = M3_CLEAR - KERF);   // M3 screw + nut
  }
}

// ------------------------------------------------------------ printed parts
// Hub: servo horn below, seat plate above (heat-set inserts in the top).
module hub() {
  difference() {
    cylinder(r = HUB_R, h = HUB_H);
    translate([0, 0, -0.01]) cylinder(d = HORN_D + 2 * FDM_CLEAR, h = HORN_T + 0.2);
    for (k = [0:3]) rotate(45 + 90 * k) translate([HORN_SCREW_R, 0, -1]) {
      cylinder(d = 2.4, h = HUB_H + 2);                  // M2 into the horn
      translate([0, 0, HORN_T + 2]) cylinder(d = 4.5, h = HUB_H);   // screw head access
    }
    translate([0, 0, -1]) cylinder(d = 6, h = HUB_H + 2);           // horn centre screw
    for (k = [0:3]) rotate(SEAM_A + 45 + 90 * k)
      translate([HUB_SCREW_R, 0, HUB_H - M3_INSERT_DEPTH]) cylinder(d = M3_INSERT, h = M3_INSERT_DEPTH + 1);
  }
}

// One rim segment, centred on the +x axis; the rim rides on the rollers.
module rim_segment() {
  a = 360 / RIM_SEGMENTS - 0.6;   // small gap between segments
  difference() {
    rotate(-a / 2) rotate_extrude(angle = a, $fn = 240)
      translate([RIM_R0, 0]) square([RIM_R1 - RIM_R0, RIM_H]);
    for (d = [-1, 1]) rotate(d * 360 / RIM_SEGMENTS / 4)
      translate([(RIM_R0 + RIM_R1) / 2, 0, RIM_H - M3_INSERT_DEPTH])
        cylinder(d = M3_INSERT, h = M3_INSERT_DEPTH + 1, $fn = 24);
  }
}

// Roller bracket: two cheeks on a flange, a 608 on an M8 axle between them.
// Printed lying on its side (cheek on the bed) for strong layers.
module roller_bracket() {
  gap = BEARING_W + 2 * 1.2 + 0.4;      // bearing + two M8 washers
  cheek = 5;
  difference() {
    union() {
      translate([-12, -(gap / 2 + cheek), 0]) cube([24, gap + 2 * cheek, 4]);
      for (sd = [-1, 1]) translate([-9, sd > 0 ? gap / 2 : -gap / 2 - cheek, 0])
        cube([18, cheek, ROLLER_C - BASE_T + 8]);
    }
    translate([0, 0, ROLLER_C - BASE_T]) rotate([90, 0, 0]) cylinder(d = M8_CLEAR, h = 60, center = true);
    for (d = [-1, 1]) translate([d * 9, 0, -1]) cylinder(d = M3_CLEAR, h = 10);
    // room for the bearing to turn
    translate([0, 0, ROLLER_C - BASE_T]) rotate([90, 0, 0]) cylinder(d = BEARING_OD + 3, h = gap, center = true);
  }
}

// Servo cradle: holds the STS3215 upright with its shaft on the turntable axis.
module servo_cradle() {
  w = SERVO_W + 2 * FDM_CLEAR; l = SERVO_L + 2 * FDM_CLEAR; h = 15; wall = 3; floor_t = 1.5;
  y0 = -SHAFT_FROM_END - FDM_CLEAR;
  difference() {
    union() {
      translate([-w / 2 - wall, y0 - wall, 0]) cube([w + 2 * wall, l + 2 * wall, h]);
      for (sx = [-1, 1], sy = [0, 1])
        translate([sx * (SERVO_W / 2 + 7), -SHAFT_FROM_END + sy * SERVO_L, 0]) cylinder(r = 5, h = 4);
    }
    translate([-w / 2, y0, floor_t]) cube([w, l, h]);
    translate([-6, y0 + l - 1, floor_t + 3]) cube([12, wall + 2, h]);    // cable exit
    for (sx = [-1, 1], sy = [0, 1])
      translate([sx * (SERVO_W / 2 + 7), -SHAFT_FROM_END + sy * SERVO_L, -1]) cylinder(d = M3_CLEAR, h = 6);
  }
}

// ------------------------------------------------------------ assembly view
module assembly() {
  color("BurlyWood") for (i = [0, 1]) linear_extrude(BASE_T) base_plate_2d(i);
  color("SteelBlue") translate([0, 0, BASE_T]) servo_cradle();
  color("DimGray") translate([-SERVO_W / 2, -SHAFT_FROM_END, BASE_T + 1.5]) cube([SERVO_W, SERVO_L, SERVO_H]);
  for (k = [0:ROLLERS - 1]) rotate(ROLLER_A0 + k * 360 / ROLLERS) translate([ROLLER_R, 0, BASE_T]) {
    color("SteelBlue") rotate(90) roller_bracket();
    color("Silver") translate([0, 0, ROLLER_C - BASE_T]) rotate([0, 90, 0])
      cylinder(d = BEARING_OD, h = BEARING_W, center = true);
  }
  color("SteelBlue") translate([0, 0, HORN_TOP]) hub();
  for (k = [0:RIM_SEGMENTS - 1]) color("SteelBlue") rotate(seg_angle(k)) translate([0, 0, RIM_Z0]) rim_segment();
  color("BurlyWood") translate([0, 0, PLATE_BOTTOM]) for (i = [0, 1]) linear_extrude(PLATE_T) seat_plate_2d(i);
  color("Peru") for (sgn = [0, 1]) rotate(SEAM_A + 180 * sgn) translate([SPLICE_R[0], 0, PLATE_BOTTOM - PLATE_T])
    linear_extrude(PLATE_T) splice_strip_2d();
}

if (part == "seat_plate_0") seat_plate_2d(0);
else if (part == "seat_plate_1") seat_plate_2d(1);
else if (part == "base_plate_0") base_plate_2d(0);
else if (part == "base_plate_1") base_plate_2d(1);
else if (part == "splice_strip") splice_strip_2d();
else if (part == "hub") hub();
else if (part == "rim_segment") rim_segment();
else if (part == "roller_bracket") rotate([90, 0, 0]) roller_bracket();
else if (part == "servo_cradle") servo_cradle();
else assembly();
