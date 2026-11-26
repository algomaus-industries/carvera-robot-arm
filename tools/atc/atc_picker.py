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

move_bit is taken from bazel-bin (build it first: bazel build -c opt
//tools/atc:move_bit) or set with MOVE_BIT.
"""
import argparse
import json
import os
import queue
import shlex
import subprocess
import threading
import tkinter as tk
from tkinter import ttk

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
    self.record_only = tk.BooleanVar(value=False)
    ttk.Checkbutton(buttons, text="I changed them by hand (record only)",
                    variable=self.record_only).pack(side="left", padx=6)
    self.status = ttk.Label(frame, text="")
    self.status.grid(row=9, column=0, columnspan=3, sticky="w")
    self.log = tk.Text(frame, height=3, width=70, font=("DejaVu Sans Mono", 8))
    self.log.grid(row=10, column=0, columnspan=3, sticky="nsew")
    frame.columnconfigure(1, weight=1)
    frame.rowconfigure(10, weight=1)
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
        if any(k in line for k in ("Plan", "->", "Moving", "✓", "[!]", "turning", "Error", "No saved")):
          self.out.put(("log", line.rstrip()))
      p.wait()
      self.out.put(("done", (on_done, "".join(lines), p.returncode)))
    threading.Thread(target=worker, daemon=True).start()

  def poll(self):
    try:
      while True:
        kind, val = self.out.get_nowait()
        if kind == "log":
          self.log.insert("end", val + "\n")
          self.log.see("end")
          if "Moving" in val:
            self.status.config(text=val.split(":")[0].replace("Moving ", "moving "))
        else:
          on_done, text, code = val
          self.busy = False
          self.apply_btn.state(["!disabled"])
          if on_done:
            on_done(text, code)
    except queue.Empty:
      pass
    self.root.after(100, self.poll)

  # ------------------------------------------------------------- actions
  def refresh(self, then=None):
    self.status.config(text="reading the ATC ...")

    def done(text, code):
      try:
        state = json.loads(next(l for l in reversed(text.splitlines()) if l.startswith("{")))
      except (StopIteration, json.JSONDecodeError):
        self.status.config(text="could not read the world (is the cell deployed?)")
        return
      for n in SLOTS:
        bit = state["atc"].get(str(n))
        title = self.title_of.get(bit, bit) if bit else "empty"
        self.now[n].config(text="now: " + (title if len(title) <= 28 else title[:27] + "…"))
        self.pick[n].set(self.label_of.get(bit, EMPTY) if bit else EMPTY)
      differs = state.get("differs") or []
      if differs and not self.demo_mode:
        more = f" (+{len(differs) - 2} more)" if len(differs) > 2 else ""
        self.banner_text.config(text=f"The world differs from the cell state saved {(state.get('saved') or '').replace('T', ' ')}: "
                                + "; ".join(differs[:2]) + more)
        self.banner.grid(row=1, column=0, columnspan=3, sticky="we", pady=(0, 4))
      else:
        self.banner.grid_remove()
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

  def apply(self):
    if self.busy:
      return
    spec = ",".join(f"{n}={self.bit_of_label.get(self.pick[n].get(), '')}" for n in SLOTS)
    record = self.record_only.get()
    self.log.insert("end", f"{'set_state' if record else 'set_atc'} {spec}\n")
    self.status.config(text="recording ..." if record else "planning ...")

    def done(text, code):
      self.status.config(text="done" if code == 0 else "failed, see the log")
      self.refresh(then=(lambda: self.root.after(2500, self.root.destroy)) if self.exit_when_done else None)
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
