// V-groove jaws for the SO-101 gripper: two PETG prints on the Bambu X1 that
// replace the arm kit's Wrist_Roll_Follower_SO101 (gripper body with the
// fixed finger) and Moving_Jaw_SO101, plus a small TPU pad for the moving
// finger (X1, external spool).
//
// The stock fingers are flat, so a round shank can sit anywhere across the
// face. Here the fixed finger gets a 90 deg V along its length: a shank
// pressed into it always ends on the V's centre line, and a 45 deg funnel at
// the V's mouth guides the shank in. The moving finger's new tip carries a
// TPU pad with a rib across the finger: it presses on one spot at mid grip
// whatever angle the jaw closes at, and its give holds the bit against
// tilting better than a rigid point would. Shanks from 1/8" (3.175 mm) to
// 6 mm fit the same V.
//
// Both PETG parts are TheRobotStudio's official meshes (Apache-2.0, see
// models/so101/LICENSE.SO-ARM100; fetch them with models/so101/fetch_meshes.sh)
// with new fingertips. Everything is modelled in the gripper's tool frame as
// the sim uses it (so101_gripper.tool_frame): origin on the fixed finger's
// face near its tip, z along the fingers towards the tip, the moving jaw on
// -x, y along the jaw hinge. A bit stands along z. All numbers come from
// jaws_params.scad (fab/gen_fab.py, table JAW). Units: mm.

include <jaws_params.scad>

part = "assembly";
$fn = 96;

MESH = "../../models/so101/official_meshes/";

SHANKS = [3.175, 4, 6];
LEVER = -JAW_PIVOT[2]; // hinge to the middle of the grip, along z

// Shank axis in the V, and the x where the pad's rib meets the shank.
function axis_x(d) = V_DEPTH - d / 2 / sin(V_ANGLE / 2);
function front_x(d) = axis_x(d) - d / 2;
// jaw_joint angle (rad) that closes the rib on a shank of diameter d.
function jaw_angle(d) = A_REF - (front_x(d) - front_x(SHANK_REF)) / LEVER;

XC = front_x(SHANK_REF);   // the rib's crest (moving jaw at A_REF)
XF = XC - RIB_H;           // the moving tip's PETG face

module fixed_stock() {
  multmatrix(TOOL_FROM_FIXED) scale(1000)
    import(str(MESH, "wrist_roll_follower_so101_v1.stl"), convexity = 10);
}

module moving_stock() {
  multmatrix(TOOL_FROM_JAW_JOINT) rotate([0, 0, A_REF * 180 / PI]) translate([0, 0, JAW_MESH_Z])
    scale(1000) import(str(MESH, "moving_jaw_so101_v1.stl"), convexity = 10);
}

// A profile in the xz plane, extruded over y = -w .. w.
module xz(w) {
  translate([0, w, 0]) rotate([90, 0, 0]) linear_extrude(2 * w) children();
}

// The V's cross-section, its faces set back by f.
module v_section(f = 0) {
  w = (V_DEPTH + 2) * tan(V_ANGLE / 2);
  translate([f / sin(V_ANGLE / 2), 0]) polygon([[V_DEPTH, 0], [-2, w], [-2, -w]]);
}

module fixed_jaw() {
  back = function(z) BACK_X0 - BACK_SLOPE * z;
  difference() {
    union() {
      fixed_stock();
      xz(HALF_W) polygon([
        [0, BLOCK_Z0], [0, TIP_Z], [TIP_T, TIP_Z],
        [back(BLOCK_Z0 - RAMP), BLOCK_Z0 - RAMP], [RAMP, BLOCK_Z0 - RAMP]]);
    }
    // the V, open towards the moving jaw
    translate([0, 0, BLOCK_Z0 - RAMP]) linear_extrude(TIP_Z - BLOCK_Z0 + RAMP + 1) v_section();
    // lead-in: the V's faces and the finger's face chamfered 45 deg by FUNNEL
    // at the tip, so a shank a little off the V's line slides into it
    hull() {
      translate([0, 0, TIP_Z - FUNNEL]) linear_extrude(0.01) v_section();
      translate([0, 0, TIP_Z]) linear_extrude(0.01) v_section(FUNNEL);
    }
    hull() {
      translate([-10, -HALF_W - 1, TIP_Z - FUNNEL]) cube([10, 2 * HALF_W + 2, 0.01]);
      translate([-10, -HALF_W - 1, TIP_Z]) cube([10 + FUNNEL, 2 * HALF_W + 2, 0.01]);
    }
    // square tip
    translate([-50, -50, TIP_Z]) cube(100);
  }
}

// The TPU pad in the tool frame (moving jaw at A_REF): a dovetailed base in
// the moving tip's slot (across the finger, along y) and a rib across the
// finger, its crest at z = 0, x = XC. clear grows it (for the slot).
module dovetail(clear = 0) {
  u = INSERT_T * tan(DOVETAIL);
  xz(PAD_HALF_W + 1) polygon([
    [XF + 0.01, INSERT_Z0 - clear], [XF + 0.01, INSERT_Z1 + clear],
    [XF - INSERT_T - clear, INSERT_Z1 + u + clear], [XF - INSERT_T - clear, INSERT_Z0 - u - clear]]);
}

module pad() {
  intersection() {
    translate([-50, -PAD_HALF_W, -50]) cube([100, 2 * PAD_HALF_W, 100]);
    union() {
      dovetail();
      intersection() {
        translate([XF - 0.01, -PAD_HALF_W, INSERT_Z0]) cube([RIB_H + 0.02, 2 * PAD_HALF_W, INSERT_Z1 - INSERT_Z0]);
        translate([XC - RIB_R, 0, 0]) rotate([90, 0, 0])
          cylinder(r = RIB_R, h = 2 * PAD_HALF_W + 2, center = true, $fn = 240);
      }
    }
  }
}

// The moving jaw's PETG part in the tool frame at A_REF. The stock finger
// goes from LEG_Z0 on; a new leg (the stock finger's section at LEG_Z0,
// tapering to the tip's width) carries the tip, PAD_T thick and dog-legged
// towards the fixed finger, so the open jaw reaches no further from the bit
// than the stock one did (the carousel's seats are spaced for that).
module moving_petg() {
  difference() {
    union() {
      difference() {
        moving_stock();
        xz(30) polygon([[-80, LEG_Z0], [40, LEG_Z0], [40, TIP_Z + 10], [-80, TIP_Z + 10]]);
      }
      hull() {
        translate([LEG_BACK, -LEG_HALF_W, LEG_Z0 - 1]) cube([LEG_FRONT - LEG_BACK, 2 * LEG_HALF_W, 1]);
        translate([XF - PAD_T, -PAD_HALF_W, BLOCK_Z0]) cube([PAD_T, 2 * PAD_HALF_W, 0.01]);
      }
      translate([XF - PAD_T, -PAD_HALF_W, BLOCK_Z0])
        cube([PAD_T, 2 * PAD_HALF_W, TIP_Z_MOVING - BLOCK_Z0]);
    }
    // dovetail slot for the TPU pad, through the tip's width
    dovetail(INSERT_CLEAR);
  }
}

// Turned from A_REF to jaw angle a about the hinge.
module at_angle(a) {
  multmatrix(TOOL_FROM_JAW_JOINT) rotate([0, 0, (a - A_REF) * 180 / PI])
    multmatrix(JAW_JOINT_FROM_TOOL) children();
}

// tool frame (jaw at A_REF) -> the official moving jaw mesh's frame
module to_jaw_mesh() {
  translate([0, 0, -JAW_MESH_Z]) rotate([0, 0, -A_REF * 180 / PI])
    multmatrix(JAW_JOINT_FROM_TOOL) children();
}

// The fingertips only, for the close-up views.
module clip_view() intersection() { children(); translate([-40, -20, -24]) cube([60, 40, 40]); }

// A bit as the sim models it: seat top at z = GRASP_Z, COLLAR_H collar, the
// shank 15 mm above it.
module bit(d) {
  color("silver") translate([axis_x(d), 0, -6]) cylinder(d = d, h = GRASP_Z - COLLAR_H + 6 + 15);
  color([0.2, 0.2, 0.22]) translate([axis_x(d), 0, GRASP_Z - COLLAR_H]) cylinder(d = d + 6, h = COLLAR_H);
}

module gripper(d) {
  color([0.94, 0.94, 0.92]) fixed_jaw();
  at_angle(jaw_angle(d)) {
    color([0.35, 0.6, 0.85]) moving_petg();
    color([0.15, 0.15, 0.17]) pad();
  }
  bit(d);
}

if (part == "wrist_roll_follower_vjaw") multmatrix(FIXED_FROM_TOOL) fixed_jaw();
else if (part == "moving_jaw_vjaw") to_jaw_mesh() moving_petg();
else if (part == "moving_jaw_pad_tpu")   // flat on the bed, rib up
  rotate([0, -90, 0]) translate([-(XF - INSERT_T), 0, 0]) pad();
else if (part == "sim_moving_jaw") to_jaw_mesh() { moving_petg(); pad(); }
else if (part == "fixed_jaw") fixed_jaw();
else if (part == "moving_jaw") { moving_petg(); pad(); }
else if (part == "view_closed") {   // closed on a 4 mm shank, from the tip
  clip_view() color([0.94, 0.94, 0.92]) fixed_jaw();
  clip_view() at_angle(jaw_angle(4)) {
    color([0.35, 0.6, 0.85]) moving_petg(); color([0.15, 0.15, 0.17]) pad();
  }
  color("silver") translate([axis_x(4), 0, -12]) cylinder(d = 4, h = 19, $fn = 64);
}
else if (part == "view_fixed") clip_view() color([0.94, 0.94, 0.92]) fixed_jaw();
else if (part == "view_moving") {   // the pad half slid into its dovetail
  clip_view() color([0.35, 0.6, 0.85]) moving_petg();
  color([0.15, 0.15, 0.17]) translate([0, 7, 0]) pad();
}
else if (part == "section")   // across the grip at z = 0, for checking
  projection(cut = true) for (i = [0:2]) translate([i * 45, 0, 0]) {
    fixed_jaw(); at_angle(jaw_angle(SHANKS[i])) { moving_petg(); pad(); }
    translate([axis_x(SHANKS[i]), 0, -1]) difference() {
      cylinder(d = SHANKS[i], h = 2); cylinder(d = SHANKS[i] - 0.3, h = 5, center = true);
    }
  }
else   // the fingertips closed on each shank size (tips up, so the V shows)
  for (i = [0:2]) translate([i * 45, 0, 0]) rotate([0, 0, 180]) intersection() {
    gripper(SHANKS[i]);
    translate([-30, -20, -32]) cube([50, 40, 80]);
  }
