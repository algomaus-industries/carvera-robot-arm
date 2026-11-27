"""ATC picker: choose a Makera bit for each Carvera ATC slot, press Apply,
and watch the SO-101 swap them (in RViz on the same desktop).

A small Tk window for the SO-101 cell. It shows which bit sits in T1..T6
now, lets you pick another bit (or none) per slot, and runs
`move_bit --set_atc=...`, which returns unwanted bits to the carousel and
fetches the wanted ones. Runs with the system Python (Tk) from the repo root
inside the omts container, on the VNC display:

  DISPLAY=:1 python3 tools/atc/atc_picker.py
  # scripted, e.g. for the demo video: fill the choices one by one, then Apply
  DISPLAY=:1 python3 tools/atc/atc_picker.py --demo='1=ofl4_22,2=bn18_1x4,6=' --exit_when_done

What it shows comes from the world model; move_bit also keeps the cell's
state on disk (which bit is where, the carousel, cart and hood). When the
two disagree (after a sim reset, or a restart on the real cell) the picker
says so and offers to restore the saved state or to keep the world as it
is. If you swap bits by hand, tick "I changed them by hand", pick what is
really in the slots now and Apply: the world and the saved state follow,
the arm does not move.

"From G-code..." reads a job exported by Makera Studio and picks the bit of
each tool the job uses (tools/atc/gcode_tools.py); slots the job does not
use keep their choice. Nothing moves until Apply.

move_bit is taken from bazel-bin (build it first: bazel build -c opt
//tools/atc:move_bit) or set with MOVE_BIT.
"""
import argparse
import json
import os
import queue
import re
import shlex
import subprocess
import sys
import threading
import time
import tkinter as tk
from tkinter import filedialog, ttk

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gcode_tools  # noqa: E402  (plain import: the picker runs outside bazel)

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
BITS_JSON = os.path.join(REPO, "models", "makera_bits", "makera_bits.json")
SLOTS = range(1, 7)
EMPTY = "(empty)"


def move_bit_cmd() -> list[str]:
  if os.environ.get("MOVE_BIT"):
    return shlex.split(os.environ["MOVE_BIT"])
  binary = os.path.join(REPO, "bazel-bin", "tools", "atc", "move_bit")
  if os.path.exists(binary):
    return [binary]
  return ["bazel", "run", "-c", "opt", "//tools/atc:move_bit", "--"]


class Picker:
  def __init__(self, root: tk.Tk, demo: str, exit_when_done: bool, demo_delay: float):
    self.root = root
    self.exit_when_done = exit_when_done
    self.demo_delay = demo_delay
    self.out: queue.Queue = queue.Queue()
    self.busy = False
    self.demo_mode = bool(demo)
    bits = json.load(open(BITS_JSON))["bits"]
    self.title_of = {"bit_" + b["id"]: b["title"] for b in bits}
    self.label_of = {}
    for b in sorted(bits, key=lambda b: b["title"].lower()):
      self.label_of["bit_" + b["id"]] = f'{b["title"]}  ({b["id"]})'
    self.bit_of_label = {v: k for k, v in self.label_of.items()}
    choices = [EMPTY] + list(self.label_of.values())

    root.title("Carvera ATC picker")
    style = ttk.Style(root)
    style.configure("Head.TLabel", font=("DejaVu Sans", 11, "bold"))
    style.configure("Now.TLabel", font=("DejaVu Sans", 9), foreground="#1b7f3b")
    style.configure("Warn.TLabel", font=("DejaVu Sans", 9), foreground="#9a5b00")
    style.configure("Status.TLabel", font=("DejaVu Sans", 10, "bold"), foreground="#1d3f73")
    frame = ttk.Frame(root, padding=6)
    frame.pack(fill="both", expand=True)
    ttk.Label(frame, text="Carvera ATC: pick a bit per slot, then Apply", style="Head.TLabel").grid(
      row=0, column=0, columnspan=3, sticky="w", pady=(0, 4))
    # Shown when the world and the saved cell state disagree (after a sim
    # reset, or a restart of the solution on the real cell).
    self.banner = ttk.Frame(frame)
    self.banner_text = ttk.Label(self.banner, style="Warn.TLabel", wraplength=600, justify="left")
    self.banner_text.pack(side="top", anchor="w")
    ttk.Button(self.banner, text="Restore saved state", command=self.restore).pack(side="left")
    ttk.Button(self.banner, text="Keep this world", command=self.keep).pack(side="left", padx=4)
    self.now, self.pick = {}, {}
    for n in SLOTS:
      r = n + 1
      ttk.Label(frame, text=f"T{n}", width=3, font=("DejaVu Sans", 10, "bold")).grid(row=r, column=0)
      self.pick[n] = ttk.Combobox(frame, values=choices, width=34, state="readonly")
      self.pick[n].grid(row=r, column=1, sticky="we", pady=1)
      self.now[n] = ttk.Label(frame, text="now: ?", style="Now.TLabel", width=30)
      self.now[n].grid(row=r, column=2, sticky="w", padx=(6, 0))
    buttons = ttk.Frame(frame)
    buttons.grid(row=8, column=0, columnspan=3, sticky="we", pady=4)
    self.apply_btn = ttk.Button(buttons, text="Apply", command=self.apply)
    self.apply_btn.pack(side="left")
    ttk.Button(buttons, text="Refresh", command=self.refresh).pack(side="left", padx=4)
    ttk.Button(buttons, text="Empty all", command=self.empty_all).pack(side="left")
    ttk.Button(buttons, text="From G-code...", command=self.from_gcode).pack(side="left", padx=4)
    self.record_only = tk.BooleanVar(value=False)
    ttk.Checkbutton(buttons, text="I changed them by hand (record only)",
                    variable=self.record_only).pack(side="left", padx=6)
    self.status = ttk.Label(frame, text="", style="Status.TLabel", wraplength=620, justify="left")
    self.status.grid(row=9, column=0, columnspan=3, sticky="w")
    # Progress over the whole run: each move's share is split into turning
    # the carousel, working out the joint path and the arm's motion. move_bit
    # only hears back once the arm has finished a move, so within the motion
    # the bar runs on the previous move's duration and stops short of its end.
    self.bar = ttk.Progressbar(frame, maximum=1000, mode="determinate")
    self.bar.grid(row=10, column=0, columnspan=3, sticky="we", pady=(2, 2))
    self.prog = None          # last @progress dict, with its arrival time
    self.arm_time = 20.0      # expected seconds of arm motion per move
    self.log = tk.Text(frame, height=2, width=70, font=("DejaVu Sans Mono", 8))
    self.log.grid(row=11, column=0, columnspan=3, sticky="nsew")
    frame.columnconfigure(1, weight=1)
    frame.rowconfigure(10, weight=1)
    self.moving = None   # (bit, src, dst) of the move move_bit is doing
    root.after(100, self.poll)
    self.refresh(then=(lambda: self.run_demo(demo)) if demo else None)

  # ------------------------------------------------------------- move_bit
  def run(self, args, on_done=None):
    self.busy = True
    self.apply_btn.state(["disabled"])

    def worker():
      env = dict(os.environ, PYTHONUNBUFFERED="1")
      p = subprocess.Popen(move_bit_cmd() + args, cwd=REPO, stdout=subprocess.PIPE,
                           stderr=subprocess.STDOUT, text=True, env=env)
      lines = []
      for line in p.stdout:
        lines.append(line)
        if line.startswith("@progress "):
          try:
            self.out.put(("progress", json.loads(line[len("@progress "):])))
          except json.JSONDecodeError:
            pass
          continue
        if any(k in line for k in ("Plan", "->", "Moving", "✓", "[!]", "turning", "Error", "No saved")):
          self.out.put(("log", line.rstrip()))
      p.wait()
      self.out.put(("done", (on_done, "".join(lines), p.returncode)))
    threading.Thread(target=worker, daemon=True).start()

  def poll(self):
    try:
      while True:
        kind, val = self.out.get_nowait()
        if kind == "progress":
          self.on_progress(val)
        elif kind == "log":
          self.log.insert("end", val + "\n")
          self.log.see("end")
          self.track(val)
        else:
          on_done, text, code = val
          self.busy = False
          self.apply_btn.state(["!disabled"])
          if on_done:
            on_done(text, code)
    except queue.Empty:
      pass
    self.tick()
    self.root.after(100, self.poll)

  # ------------------------------------------------------------- progress
  # Where each phase starts within a move's share of the bar, and its width.
  PHASES = {"turn": (0.0, 0.10), "plan": (0.10, 0.05), "arm": (0.15, 0.85)}

  def on_progress(self, p):
    now = time.monotonic()
    if p["phase"] == "done" and self.prog and self.prog["phase"] == "arm":
      self.arm_time = max(3.0, now - self.prog["t"])
    p["t"] = now
    self.prog = p
    i, n = p["i"], max(1, p["n"])
    if p["phase"] in self.PHASES:
      self.status.config(text=f"Move {i + 1} of {n}: {p['move']}\n{p['text']} ...")
    elif p["phase"] == "done":
      self.status.config(text=f"{i} of {n} done: {p['text']}")
    else:
      self.status.config(text=p["text"][0].upper() + p["text"][1:] + " ...")
    self.tick()

  def tick(self):
    p = self.prog
    if p is None:
      return
    n = max(1, p["n"])
    if p["phase"] in self.PHASES:
      start, width = self.PHASES[p["phase"]]
      frac = 0.0
      if p["phase"] == "arm":
        frac = min(0.95, (time.monotonic() - p["t"]) / self.arm_time)
      value = (p["i"] + start + width * frac) / n
    else:
      value = p["i"] / n
    # Keep the last 2 % for updating the simulator and homing the arm.
    self.bar["value"] = 1000 * 0.98 * min(1.0, value)

  def show_now(self, n, bit):
    title = self.title_of.get(bit, bit) if bit else "empty"
    self.now[n].config(text="now: " + (title if len(title) <= 28 else title[:27] + "…"))

  def track(self, line):
    """Keeps the "now:" labels up to date while move_bit runs: a move's
    source slot empties and its target fills once move_bit reports it done."""
    m = re.match(r"Moving (\S+) .*: (\S+) -> (\S+) \.\.\.", line)
    if m:
      self.moving = m.groups()
      return
    m = re.match(r"\[✓\] (\S+) is now in (\S+)", line)
    if m and self.moving and self.moving[0] == m.group(1):
      bit, src, dst = self.moving
      for slot, now in ((src, None), (dst, bit)):
        if slot.startswith("atc:") and int(slot[4:]) in self.now:
          self.show_now(int(slot[4:]), now)
      self.moving = None

  # ------------------------------------------------------------- actions
  def refresh(self, then=None, quiet=False):
    """Reads the ATC from the world. quiet: leave the status line alone."""
    if not quiet:
      self.status.config(text="reading the ATC ...")

    def done(text, code):
      try:
        state = json.loads(next(l for l in reversed(text.splitlines()) if l.startswith("{")))
      except (StopIteration, json.JSONDecodeError):
        self.status.config(text="could not read the world (is the cell deployed?)")
        return
      for n in SLOTS:
        bit = state["atc"].get(str(n))
        self.show_now(n, bit)
        self.pick[n].set(self.label_of.get(bit, EMPTY) if bit else EMPTY)
      differs = state.get("differs") or []
      if differs and not self.demo_mode:
        more = f" (+{len(differs) - 2} more)" if len(differs) > 2 else ""
        self.banner_text.config(text=f"The world differs from the cell state saved {(state.get('saved') or '').replace('T', ' ')}: "
                                + "; ".join(differs[:2]) + more)
        self.banner.grid(row=1, column=0, columnspan=3, sticky="we", pady=(0, 4))
      else:
        self.banner.grid_remove()
      if not quiet:
        self.status.config(text="ready" + (f"  (cart {state['cart']}, hood {state['hood']})"
                                           if "cart" in state else ""))
      if then:
        then()
    self.run(["--list_json"], done)

  def restore(self):
    self.status.config(text="restoring the saved state ...")
    self.run(["--restore_state"], lambda text, code: self.refresh())

  def keep(self):
    self.status.config(text="saving this world as the cell state ...")
    self.run(["--save_state"], lambda text, code: self.refresh())

  def empty_all(self):
    for n in SLOTS:
      self.pick[n].set(EMPTY)

  def from_gcode(self, path: str = ""):
    """Picks the bits a Makera Studio job needs, tool number = ATC slot."""
    path = path or filedialog.askopenfilename(
      parent=self.root, title="G-code exported by Makera Studio",
      initialdir=os.environ.get("ATC_GCODE_DIR", os.path.expanduser("~")),
      filetypes=[("G-code", "*.nc *.gcode *.cnc *.tap"), ("All files", "*")])
    if not path:
      return
    try:
      job = gcode_tools.plan(path)
    except OSError as e:
      self.status.config(text=f"Cannot read {path}: {e}")
      return
    self.log.insert("end", f"{os.path.basename(path)}\n{gcode_tools.report(job)}\n")
    self.log.see("end")
    for n, bit in job.targets.items():
      self.pick[n].set(self.label_of["bit_" + bit])
    slots = ", ".join(f"T{n}" for n in sorted(job.targets)) or "no slot"
    text = f"{os.path.basename(path)}: picked {slots}."
    if job.guesses:
      text += (" " + ", ".join(f"T{n}" for n in job.guesses) +
               ": custom tool, closest bit picked - check it.")
    if job.problems:
      text += " Not picked: " + "; ".join(job.problems) + "."
    self.status.config(text=text + " Nothing moves until Apply.")

  def apply(self):
    if self.busy:
      return
    spec = ",".join(f"{n}={self.bit_of_label.get(self.pick[n].get(), '')}" for n in SLOTS)
    record = self.record_only.get()
    self.log.insert("end", f"{'set_state' if record else 'set_atc'} {spec}\n")
    self.status.config(text="Recording ..." if record else "Working out which bits go where ...")

    self.prog = None
    self.bar["value"] = 0

    def done(text, code):
      self.prog = None
      self.bar["value"] = 1000 if code == 0 else self.bar["value"]
      self.status.config(text="Done: the ATC holds the bits above." if code == 0 else
                         "Failed, see the log.")
      self.refresh(then=(lambda: self.root.after(2500, self.root.destroy)) if self.exit_when_done else None,
                   quiet=True)
    if record:
      self.record_only.set(False)
      self.run([f"--set_state={spec}"], done)
    else:
      self.run([f"--set_atc={spec}", "--home_frame=view"], done)

  def run_demo(self, spec: str):
    """Sets the given choices one by one (visibly), then presses Apply."""
    items = [p.split("=", 1) for p in spec.split(",") if p]

    def step(i):
      if i == len(items):
        self.root.after(int(self.demo_delay * 1000), self.apply)
        return
      n, bit = int(items[i][0].lstrip("Tt")), items[i][1]
      bit = ("bit_" + bit if bit and not bit.startswith("bit_") else bit)
      self.pick[n].set(self.label_of.get(bit, EMPTY) if bit else EMPTY)
      self.pick[n].focus_set()
      self.root.after(int(self.demo_delay * 1000), lambda: step(i + 1))
    self.root.after(int(self.demo_delay * 1000), lambda: step(0))


def main():
  ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
  ap.add_argument("--geometry", default="", help="Tk geometry, e.g. 640x360+1280+720")
  ap.add_argument("--demo", default="", help="'1=<bit id>,2=,...': pick these, then Apply")
  ap.add_argument("--demo_delay", type=float, default=1.5)
  ap.add_argument("--exit_when_done", action="store_true")
  args = ap.parse_args()
  root = tk.Tk(className="ATC picker")
  if args.geometry:
    root.geometry(args.geometry)
  Picker(root, args.demo, args.exit_when_done, args.demo_delay)
  root.mainloop()


if __name__ == "__main__":
  main()
