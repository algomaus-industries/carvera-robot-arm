# Building the cell: carousel, pedestal and cart

Everything the SO-101 cell needs besides the Carvera, the Küpper bench and
the arm kit itself. These are the same parts the simulation uses: the sim
models and these files come from the same numbers
(`models/bit_carousel/carousel_layout.py`, `models/so101/build_so101.py`), so
a seat in the sim is a seat on the real carousel. What RViz shows of the
carousel, the pedestal and the cart *is* these parts: `gen_fab.py` also
exports them by material (`*/out/sim_*.stl`, `pedestal.stl`), and the Blender
build scripts turn those into the sim models' meshes.

| Assembly | Look | Made on | Files |
| :--- | :--- | :--- | :--- |
| **Bit carousel**: 140 seats on a turning plate; an STS3215 servo turns the wanted bit to the arm | <img src="carousel/out/assembly.png" width="260" alt="Bit carousel: plywood seat plate with 140 holes on a blue printed rim, rollers and base"> | plates: xTool M2 (6 mm birch ply) or the Carvera; small parts: Bambu X1 (PETG) | [`carousel/out/`](carousel/out/) |
| **Arm pedestal**: lifts the SO-101 to 5 cm above the grip height, overhangs the cart edge towards the Carvera | <img src="pedestal/out/pedestal.png" width="200" alt="Printed pedestal block with top lip, foot and gussets"> | Bambu X1 (PETG) | [`pedestal/out/pedestal.stl`](pedestal/out/pedestal.stl) |
| **Rolling cart**: carries both, rolls away when the Carvera's hood is shut | <img src="cart/out/assembly.png" width="200" alt="Plywood cart on four braked casters, carousel and pedestal on top"> | table saw / track saw (18 mm birch ply, 40 × 40 mm timber) | [`cart/out/`](cart/out/) |

Regenerate after changing a layout or size:

```bash
python3 fab/check_layout.py   # every seat leaves the open jaw >= 12 mm to its neighbours
python3 fab/gen_fab.py        # params + STL / DXF / SVG / PNG (needs OpenSCAD with Manifold)
```

`OPENSCAD=...` picks the OpenSCAD command; it needs the Manifold backend
(OpenSCAD 2024+ / nightly).

## Before you cut

- **Measure your parts, then set them** in `carousel_layout.py` /
  `fab/gen_fab.py` and regenerate. Taken from datasheets, not measured:
  the STS3215 body (24.7 × 45.2 × 35 mm), where its output shaft sits
  (`SERVO_SHAFT_FROM_END`, 11.5 mm), and the round horn (`HORN_D` 20 mm,
  `HORN_T` 3 mm, screws on a 7 mm radius).
- **Laser kerf**: `KERF = 0.2` mm in `carousel/carousel.scad` (outlines grow,
  holes shrink by half of it). Cut a test hole first: a 1/8" (3.175 mm) and a
  6 mm Makera bit should both drop through a seat hole (6.6 mm) and hang on
  their collar.
- **The seat plate is 352 mm across**, so it comes in two halves (the seam
  runs between two seats on every ring). Each half (352 × 176 mm) fits the
  xTool M2 and the Carvera (360 × 240 mm). If your laser takes 352 × 352 mm,
  cut it whole: union the two halves' outlines in your laser software.
- **On the Carvera**: the DXFs are 2D outlines; profile-cut 6 mm ply with a
  3.175 mm flat end mill, holes as pockets (they are larger than the tool).
  Hold-down tabs where the halves' straight edge meets the circle.

## Bill of materials

### Bit carousel

| Qty | Part | Notes |
| ---: | :--- | :--- |
| 2 | Seat plate half, 6 mm birch ply | `seat_plate_0.dxf`, `seat_plate_1.dxf` (352 × 176 mm each) |
| 2 | Base plate half, 6 mm birch ply | `base_plate_0.dxf`, `base_plate_1.dxf` (356 × 178 mm each) |
| 2 | Splice strip, 6 mm birch ply | `splice_strip.dxf`: joins the seat plate halves underneath |
| 1 | Hub, PETG | `hub.stl`: servo horn to seat plate |
| 8 | Rim segment, PETG | `rim_segment.stl`: rim under the plate's edge, rides on the rollers |
| 4 | Roller bracket, PETG | `roller_bracket.stl` (print on its side as exported) |
| 1 | Servo cradle, PETG | `servo_cradle.stl` |
| 1 | Feetech STS3215 servo with round horn | same servo as the SO-101's joints (7.4 V version); goes on the arm's bus as ID 7 |
| 1 | 3-pin servo cable, ~40 cm | daisy-chain from the arm's bus to the carousel servo |
| 4 | 608-2RS bearing (22 × 8 × 7 mm) | rollers |
| 4 | M8 × 30 bolt, nut, 2 washers | roller axles |
| 20 | M3 heat-set insert (4.0 mm OD, 5.7 mm) | hub 4, rim segments 16 |
| 20 | M3 × 10 screw | seat plate to hub and rim |
| 28 | M3 × 16 screw + nut | splice strips 16, roller brackets 8, servo cradle 4 |
| 6 | M4 × 16 countersunk wood screw | base plate to the cart top |
| ~300 g | PETG filament | |

### Arm pedestal

| Qty | Part | Notes |
| ---: | :--- | :--- |
| 1 | Pedestal, PETG (~250 g) | `pedestal.stl`, print standing as exported, 4 walls, 20 % gyroid |
| 2 | M4 × 20 countersunk wood screw | foot to the cart top (plus a bead of construction glue) |
| — | the two table clamps from the SO-101 kit | clamp the arm's base onto the pedestal's top lip |

### Rolling cart

| Qty | Part | Size (mm) | Notes |
| ---: | :--- | :--- | :--- |
| 1 | Top, 18 mm birch ply | 355 × 435 | `top.dxf`: two cable holes (arm, carousel servo) |
| 1 | Shelf, 18 mm birch ply | 315 × 395 | `shelf.dxf`: corners notched around the legs |
| 2 | Long apron, 18 mm birch ply | 325 × 60 | |
| 2 | Short apron, 18 mm birch ply | 245 × 60 | |
| 4 | Leg, planed timber | 40 × 40 × 822 | |
| 4 | Shelf cleat, timber | 20 × 20 × 100 | screwed to the legs at 150 mm (shelf top 168 mm above the floor) |
| 4 | Swivel caster with brake | 50 mm wheel, 75 mm mounting height, ≥ 25 kg | plate ~60 × 45 mm |
| 16 | 4 × 40 wood screw | | aprons to legs, top to aprons (from below, through pocket holes or angle brackets) |
| 16 | 4 × 16 wood screw | | casters |
| — | wood glue, finish (hard wax oil) | | |

The cart top ends 915 mm above the floor with these casters. That height is
what puts the carousel's seats level with the Carvera's ATC seats on the
840 mm Küpper bench (bench + 150 mm − 75 mm); with other casters, change
the leg length so the top stays at 915 mm.

## Assembly

### Cart

1. Cut the legs to 822 mm and the aprons; glue and screw the aprons between
   the legs, flush with the legs' tops and outer faces (35 mm in from the
   top's edges once the top is on).
2. Screw the cleats to the legs, shelf on the cleats.
3. Top on, flush all round, screwed from below.
4. Turn the cart over and screw on the four casters under the legs.
5. Roll it in front of the Carvera's bench and check: the top at 915 mm, its
   front edge 5 mm from the bench (`configs/carvera_so101/gen_cell_configs.py`
   has the exact position; the sim shows it).

### Pedestal and arm

1. Screw the pedestal's foot to the cart top: the block overhangs the cart's
   front edge (towards the Carvera) by 45 mm, its long side along the edge,
   the arm's pan axis 25 mm inside the Carvera's right-hand side panel line.
2. Run the arm's bus and power cables through the slot under the top lip and
   down through the cart top's small hole.
3. Clamp the SO-101's base onto the top lip with the kit's two table
   clamps, facing away from the Carvera (world −y in the sim): its pan range
   then runs from the ATC across the open machine front to the carousel and
   never towards the side panel.

### Carousel

1. **Inserts**: press the M3 heat-set inserts into the hub (4, top face)
   and the rim segments (2 each, top face).
2. **Base**: lay the two base plate halves on the cart top where the sim
   puts the carousel (its axis 203 mm / 167 mm from the arm's pan axis, see
   `CENTER_IN_PAN`), screw them down with the six M4 screws.
3. **Rollers**: a 608 between two M8 washers on an M8 × 30 bolt through each
   roller bracket, nut snug, bearing free; screw the brackets onto the base
   (M3 × 16 + nut), axles pointing at the centre.
4. **Servo**: set its bus ID to 7 first (Feetech FD software or LeRobot's
   motor setup), position mode, centre it (2048). Drop it into the cradle,
   output shaft up and on the centre mark, cable out of the slot and down
   through the base and cart top; screw the cradle down. Fit the round horn.
5. **Seat plate**: join the two halves upside down on a flat table with the
   splice strips along the seam (M3 × 16 + nut). Screw the hub (centre) and
   the eight rim segments (one centred on each end of the seam, then every
   45°) onto it with M3 × 10.
6. Turn the plate over onto the rollers, hub on the horn; screw the hub to
   the horn (M2 screws from the horn's kit, through the hub's access holes).
   It should turn freely with the servo unpowered.
7. **Zero**: with the servo at its centre position, turn the plate (loosen
   the horn screws) so the seam runs along world x, as in the sim; tighten.
   Seat `cell_R_C` is then at platter angle 0 where
   `carousel_layout.cells()` says, and `move_bit --turn=grid:R,C` turns it
   to its ring's pick position.
8. Put the bits in, ring 0 (inside) first, in the order of
   `configs/carvera_so101/bits.updates.pbtxt`, so the sim and the real
   carousel agree on which bit is where.

### Hood and cart

The Carvera's hood is opened and shut by hand and the cart is pushed by
hand: `move_bit --dock` (hood open, cart in) and `--park` (cart away, hood
shut) animate this in the sim and, on the real cell, ask you to do it and
wait for Enter. Park the cart before shutting the hood: the arm's pan axis
sits just inside the open hood's front.

## Not done yet

- There is no hardware driver for the SO-101 or the carousel servo yet
  (the arm is not bought); everything above runs in simulation.
- The dimensions marked as datasheet values above are not measured.
