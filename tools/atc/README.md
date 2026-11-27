# ATC tools (`tools/atc/`)

`move_bit` drives the SO-101 cell (`--//:setup=carvera_so101`): it moves
Makera bits between the bit carousel (`bit_grid`, slots `grid:R,C`: ring R,
seat C) and the Makera Carvera's tool rack (`atc:1` … `atc:6`; slot 0 is the
probe). It finds the bit at the source slot in the belief world, turns the
carousel, opens the Carvera hood when the ATC is involved, and picks and
places top-down.

```bash
bazel run -c opt //tools/atc:move_bit -- --list
bazel run -c opt //tools/atc:move_bit -- --move=grid:2,10:atc:3   # repeatable
bazel run -c opt //tools/atc:move_bit -- --fill_atc    # empty T1..T6 <- carousel
bazel run -c opt //tools/atc:move_bit -- --empty_atc   # T1..T6 -> free seats
bazel run -c opt //tools/atc:move_bit -- --turn=grid:2,10
bazel run -c opt //tools/atc:move_bit -- --park        # arm home, cart away, hood shut
bazel run -c opt //tools/atc:move_bit -- --dock        # hood open, cart in
bazel run -c opt //tools/atc:move_bit -- --set_atc='1=ofl4_22,2=,6=bn4_12'   # see below
bazel run -c opt //tools/atc:move_bit -- --list_json   # ATC contents as JSON
bazel run -c opt //tools/atc:move_bit -- --from_gcode=$HOME/job.nc   # the bits a job needs, see below
```

## ATC picker

[`atc_picker.py`](atc_picker.py) is a small window for choosing which bit goes
in which ATC slot: it shows what T1–T6 hold now, offers all of Makera's bits
per slot (or empty), and on **Apply** runs `move_bit --set_atc`, which first
returns every bit that should not stay to the carousel (its own seat if
free) and then fetches each wanted bit, from the carousel or another slot.
Watch the moves in RViz next to it. It uses the system Python's Tk, so run
it inside the omts container on the VNC desktop:

```bash
bazel build -c opt //tools/atc:move_bit
DISPLAY=:1 python3 tools/atc/atc_picker.py
```

In the omts container it is also on the desktop and in the Applications
menu as **ATC picker** (`~/.local/bin/atc-picker`, which builds `move_bit`
first if needed and logs to `/tmp/atc-picker.log`).

### Loading the ATC for a Makera Studio job

Makera Studio writes a header into every G-code file it exports, with one
line per tool: its number (= ATC slot), name and dimensions.
[`gcode_tools.py`](gcode_tools.py) reads it and finds each tool in the bit
catalogue. It needs only Python, so it runs anywhere:

```bash
python3 tools/atc/gcode_tools.py job.nc
#   T2  3.175*0.8*5.5mm Corn      = corn18_0p8
#   T3  3.175*1*7mm Corn          = corn18_1
#   T4  3.175*2*10.5mm Corn       = corn18_2
# move_bit --set_atc='2=corn18_0p8,3=corn18_1,4=corn18_2'
```

- `=`: a tool from Studio's own library. Each of its 129 tools is exactly
  one bit of the catalogue (`tests/unit/test_gcode_tools.py` checks all of
  them against `models/makera_bits/makera_studio_cutters.json`).
- `~`: a custom tool (one you defined in Studio). It has no bit of its own,
  so the closest bit of the same kind and shank is proposed. Check it, or
  name the bit: `--map 2=fe18_1p5x8`.
- `!`: something that stops the plan: a tool number above 6, two slots
  wanting the same bit, a library tool the catalogue does not know, a
  header tool the program never changes to, or G-code without a Studio
  header (then name every tool's bit with `--map`).

`move_bit --from_gcode=job.nc` does the same and then loads the ATC like
`--set_atc`. Slots the job does not use stay as they are. It moves nothing
while a `!` is open, and stops at a `~` unless `--gcode_map 2=<bit id>`
names the bit or `--accept_closest` takes the proposal. In the picker,
**From G-code...** fills in the job's slots; nothing moves until Apply.
Studio keeps the files it sends to the machine in its `GCodes` folder
(under Wine: `~/.wine-makera/drive_c/users/<you>/AppData/Roaming/MakeraStudio/GCodes`;
`ATC_GCODE_DIR` sets where the picker's file dialog opens). `move_bit` and
the picker run inside the omts container, so the file has to be reachable
there.

### Keeping it in step with the real cell

`move_bit` keeps the cell's state on disk,
`~/.local/state/carvera_so101/cell_state.json` (or `$CARVERA_CELL_STATE`):
which bit is in which ATC slot and carousel seat, the carousel's angle,
the cart (docked / parked) and the hood. It is written after every move,
turn, park and dock, so on the real cell it is the record that outlives a
restart of the solution, whose world model starts from the configs' layout.

- `--restore_state` puts the world back to the saved state (no arm motion);
  `--save_state` saves the world as it is.
- `--set_state='3=oflm4_12,5='` records bits changed by hand, in the same
  form as `--set_atc`: the world and the file follow, the arm does not move.
- `--list_json` also says what differs between the world and the file.

The picker shows that difference as a banner with **Restore saved state** /
**Keep this world**, and its **I changed them by hand** box makes Apply
record instead of move. On the real cell, `move_bit` asks you to make sure
the Carvera's spindle is empty before the arm goes to the ATC: after a job
the Carvera can keep a bit in the spindle, and its slot would look free.

`--demo='1=<bit id>,…'` picks those bits one by one and presses Apply by
itself (the demo video uses it).

Explicit `--move`s run first, then `--fill_atc` and `--empty_atc`, each planned
against the world as the earlier moves left it.

## How a move runs

Each move is a behaviour tree: above the source, down, close the jaw and
attach the bit to the gripper, lift, carry, down into the target, open, detach,
lift.

The planner's generic IK does not converge for this 5-axis arm, so
[`so101_ik.py`](so101_ik.py) solves for joint positions that keep the gripper
vertical (yaw free) and put the shank into the fixed finger's V (the
V-groove jaws of [`fab/jaws`](../../fab/README.md): the shank's axis is
`3 − 0.707 d` mm in front of the finger's face, and the jaw closes to the
angle at which the moving finger's pad meets a shank of that diameter).
`move_bit` sends those as joint moves, which the planner still
collision-checks. Carries go via the `home` configuration with the elbow up,
clear of the Carvera.

A slot the arm cannot reach with the gripper vertical fails before anything
moves ("SO-101 cannot reach … with the gripper vertical"). In the SO-101
cell all six ATC slots and every carousel ring's pick position are in reach.

**Carousel.** The SO-101 cell's bits sit on a turning seat plate
(`models/bit_carousel`). Before a move, `move_bit` turns its
`carousel_joint` so the seat involved comes to its ring's pick position
(`carousel_layout.platter_angle_for`); bits are reparented to their seat
frames, so they turn with the plate. A move between two carousel seats goes
via a free ATC slot. `--turn=grid:R,C` only turns the carousel.

**Jaw direction.** On the carousel the neighbouring bits stand closer than
the open jaw is wide, so for each seat `move_bit` turns the jaw axis (wrist
roll) to the direction that leaves the jaws most room to the neighbouring
seats. `fab/check_layout.py` checks that every seat leaves at least 12 mm.

**Cart.** `--park` sends the arm home, rolls the cart (and everything on it)
away from the Carvera and shuts the hood; `--dock` opens the hood and rolls
the cart back in. The sim animates both; on the real cell they ask you to do
it by hand and wait for Enter.

**Stuck arm.** A move that fails part-way leaves the arm where it stopped,
and a world reset does not move it. `joint_probe 'SET:[q...]'` puts it at
given joints (e.g. home) in the world and resets the simulator.

In simulation, after the moves `move_bit` writes each bit's new seat into the
world and resets the simulator, because the SO-101 jaw never closes on the
shank in Gazebo and the physics would otherwise leave the bit behind.

Set `MOVE_BIT_PRINT_JOINTS=1` to print every SO-101 joint target.

## Layout

- SO-101 cell: [`configs/carvera_so101/`](../../configs/carvera_so101/)
  (`cell.updates.pbtxt`, `bits.updates.pbtxt`, generated by
  `gen_cell_configs.py`).
- Models: `models/bit_carousel`, `models/makera_bits`, `models/makera_carvera`.
