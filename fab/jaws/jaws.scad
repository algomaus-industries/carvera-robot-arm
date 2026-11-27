// V-groove jaws for the SO-101 gripper: two PETG prints on the Bambu X1 that
// replace the arm kit's Wrist_Roll_Follower_SO101 (gripper body with the
// fixed finger) and Moving_Jaw_SO101.
//
// The stock fingers are flat, so a round shank can sit anywhere across the
// face. Here the fixed finger gets a 90 deg V along its length: a shank
// pressed into it always ends on the V's centre line. The moving finger gets
// a pad that stands proud of the stock face and is crowned along the finger,
// so it presses on one spot at mid grip whatever angle the jaw closes at.
// Shanks from 1/8" (3.175 mm) to 6 mm fit the same V.
//
// Both parts are TheRobotStudio's official meshes (Apache-2.0, see
// models/so101/LICENSE.SO-ARM100; fetch them with models/so101/fetch_meshes.sh)
// with new fingertips. Everything is modelled in the gripper's tool frame as
// the sim uses it (so101_gripper.tool_frame): origin on the fixed finger's
// face near its tip, z along the fingers towards the tip, the moving jaw on
// -x, y along the jaw hinge. A bit stands along z. Frames come from
// jaws_params.scad (fab/gen_fab.py). Units: mm.

include <jaws_params.scad>

part = "assembly";
$fn = 96;

MESH = "../../models/so101/official_meshes/";

// ---- the V in the fixed finger
V_DEPTH = 3.0;         // apex this far behind the finger face (x = 0)
V_ANGLE = 90;          // included angle
TIP_Z = 6.0;           // fingertips end here: 3 mm above a 5 mm collar at grasp_z = 14 mm
BLOCK_Z0 = -13.0;      // the new tips start here (19 mm of V; a shank sticks up to z = -6)
HALF_W = 5.5;          // half width of the fixed tip (stock: 5.0 at z = 0)
TIP_T = 6.0;           // fixed tip thickness at its end (3 mm of PETG behind the V)
OUTER_SLOPE = 0.365;   // the stock finger's back face: x = 7.29 - 0.365 z
RAMP = 4.0;            // 45 deg ramps where the new tips leave the stock fingers

// ---- the pad on the moving finger
SHANK_REF = 4.0;       // the pad is drawn closed on this shank...
A_REF = -0.08;         // ...at this jaw_joint angle (rad; stock limits -0.1745 .. 1.745)
PAD_R = 60;            // crown radius along the finger
PAD_HALF_W = 5.0;

SHANKS = [3.175, 4, 6];
LEVER = -JAW_PIVOT[2]; // hinge to the middle of the grip, along z

// Shank axis in the V, and the x where the pad meets the shank.
function axis_x(d) = V_DEPTH - d / 2 / sin(V_ANGLE / 2);
function front_x(d) = axis_x(d) - d / 2;
// jaw_joint angle (rad) that closes the pad on a shank of diameter d.
function jaw_angle(d) = A_REF - (front_x(d) - front_x(SHANK_REF)) / LEVER;

module fixed_stock() {
  multmatrix(TOOL_FROM_FIXED) scale(1000)
    import(str(MESH, "wrist_roll_follower_so101_v1.stl"), convexity = 10);
}

module moving_stock(a) {
  multmatrix(TOOL_FROM_JAW_JOINT) rotate([0, 0, a * 180 / PI]) translate([0, 0, JAW_MESH_Z])
    scale(1000) import(str(MESH, "moving_jaw_so101_v1.stl"), convexity = 10);
}

// A profile in the xz plane, extruded over y = -w .. w.
module xz(w) {
  translate([0, w, 0]) rotate([90, 0, 0]) linear_extrude(2 * w) children();
}

module fixed_jaw() {
  back = function(z) 7.29 - OUTER_SLOPE * z;
  difference() {
    union() {
      fixed_stock();
      xz(HALF_W) polygon([
        [0, BLOCK_Z0], [0, TIP_Z], [TIP_T, TIP_Z],
        [back(BLOCK_Z0 - RAMP), BLOCK_Z0 - RAMP], [RAMP, BLOCK_Z0 - RAMP]]);
    }
    // the V, open towards the moving jaw
    w = (V_DEPTH + 2) * tan(V_ANGLE / 2);
    translate([0, 0, BLOCK_Z0 - RAMP]) linear_extrude(TIP_Z - BLOCK_Z0 + RAMP + 1)
      polygon([[V_DEPTH, 0], [-2, w], [-2, -w]]);
    // square tip
    translate([-50, -50, TIP_Z]) cube(100);
  }
}

// Moving jaw, in the tool frame at jaw angle a.
module moving_jaw(a = A_REF) {
  xc = front_x(SHANK_REF);
  multmatrix(TOOL_FROM_JAW_JOINT) rotate([0, 0, (a - A_REF) * 180 / PI])
    multmatrix(JAW_JOINT_FROM_TOOL) difference() {
      union() {
        moving_stock(A_REF);
        intersection() {
          xz(PAD_HALF_W) polygon([
            [xc, BLOCK_Z0], [xc, TIP_Z], [-13, TIP_Z], [-17, BLOCK_Z0 - 2 * RAMP],
            [-15, BLOCK_Z0 - 2 * RAMP]]);
          // crown: one contact spot at mid grip (z = 0)
          translate([xc - PAD_R, 0, 0]) rotate([90, 0, 0])
            cylinder(r = PAD_R, h = 4 * PAD_HALF_W, center = true, $fn = 360);
        }
      }
      translate([-50, -50, TIP_Z]) cube(100);
    }
}

// A bit as the sim models it: seat top at z = 14 (grasp_z), 5 mm collar, the
// shank 15 mm above it.
module bit(d) {
  color("silver") translate([axis_x(d), 0, -6]) cylinder(d = d, h = 20 + 15);
  color([0.2, 0.2, 0.22]) translate([axis_x(d), 0, 9]) cylinder(d = d + 6, h = 5);
}

module gripper(d) {
  color([0.94, 0.94, 0.92]) fixed_jaw();
  color([0.35, 0.6, 0.85]) moving_jaw(jaw_angle(d));
  bit(d);
}

if (part == "wrist_roll_follower_vjaw") multmatrix(FIXED_FROM_TOOL) fixed_jaw();
else if (part == "moving_jaw_vjaw")
  translate([0, 0, -JAW_MESH_Z]) rotate([0, 0, -A_REF * 180 / PI])
    multmatrix(JAW_JOINT_FROM_TOOL) moving_jaw(A_REF);
else if (part == "fixed_jaw") fixed_jaw();
else if (part == "moving_jaw") moving_jaw(A_REF);
else if (part == "section")   // across the grip at z = 0, for checking
  projection(cut = true) for (i = [0:2]) translate([i * 45, 0, 0]) {
    fixed_jaw(); moving_jaw(jaw_angle(SHANKS[i]));
    translate([axis_x(SHANKS[i]), 0, -1]) difference() {
      cylinder(d = SHANKS[i], h = 2); cylinder(d = SHANKS[i] - 0.3, h = 5, center = true);
    }
  }
else   // the fingertips closed on each shank size (tips up, so the V shows)
  for (i = [0:2]) translate([i * 45, 0, 0]) rotate([0, 0, 180]) intersection() {
    gripper(SHANKS[i]);
    translate([-30, -20, -32]) cube([50, 40, 80]);
  }
