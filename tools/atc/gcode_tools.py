"""Which bits a Makera Studio job needs, and in which ATC slots.

Makera Studio writes a header into every G-code file it exports: one line
per tool with its ATC number, its name and its dimensions, e.g.

  ;@MKR|TOOL|number=2|id=111218083806|name=3.175*0.8*5.5mm Corn|type=Flat End|
      handlediameter=3.175|...|flutelength=5.5|diameter=0.8|tipdiameter=0.8|...

This module reads those lines and finds each tool in the bit catalogue
(models/makera_bits/makera_bits.json). A tool from Studio's own library
(`id` set) is one of the catalogue's 129 bits and matches exactly. A custom
tool (`id` empty) that carries a library tool's name is that bit with
corrected dimensions (say "Custom 3.175*1.5*6mm Flat End(Metal) - adjusted
diameter", made because the bit measured 1.32 mm): it is the same bit. Any
other custom tool has no bit of its own, so the closest bit of the same kind
and shank is proposed and marked as a guess.

  python3 tools/atc/gcode_tools.py job.nc                # table + --set_atc spec
  python3 tools/atc/gcode_tools.py job.nc --map 2=fe18_1p5x8
  python3 tools/atc/gcode_tools.py job.nc --json

`move_bit --from_gcode=job.nc` uses it to load the ATC for a job. Only the
standard library is needed here, so it also runs outside bazel.
"""

import argparse
import dataclasses
import json
import math
import os
import re
import sys

BITS_JSON = os.path.join(
  os.path.dirname(os.path.abspath(__file__)),
  "..",
  "..",
  "models",
  "makera_bits",
  "makera_bits.json",
)
CUTTERS_JSON = os.path.join(
  os.path.dirname(BITS_JSON), "makera_studio_cutters.json"
)
# Makera Studio's tool categories (t_MakeraCutterList.cutterCategoryId) as the
# header's type field. "Flat End" and "Engraving" are as Studio 0.1.3 writes
# them; the others are not confirmed, and kind_of also reads the name.
CATEGORY_TYPES = {
  0: "Ball Nose",
  1: "Flat End",
  2: "Chamfer",
  3: "Engraving",
  5: "Drill",
  6: "Thread",
  7: "Ball Nose Engraving",
}
ATC_SLOTS = range(1, 7)
HEADER = ";@MKR|"
# A custom tool this close to a catalogue bit is taken to be that bit.
EXACT_D_MM, EXACT_CUT_MM, EXACT_ANGLE_DEG = 0.011, 0.06, 0.6


@dataclasses.dataclass(frozen=True)
class StudioTool:
  """One ;@MKR|TOOL line. Lengths in mm, angles in degrees."""

  number: int
  id: str
  name: str
  type: str
  shank: float
  diameter: float
  tip: float
  flute: float
  corner_radius: float
  angle: float
  half_angle: float

  @property
  def custom(self) -> bool:
    return not self.id

  @property
  def metal(self) -> bool:
    return "(metal)" in self.name.lower()


@dataclasses.dataclass(frozen=True)
class Match:
  tool: StudioTool
  bit: dict | None
  exact: bool
  note: str = ""

  @property
  def bit_id(self) -> str | None:
    return self.bit["id"] if self.bit else None


def _f(fields: dict, key: str) -> float:
  try:
    return float(fields.get(key) or 0)
  except ValueError:
    return 0.0


def tool_from_fields(fields: dict) -> StudioTool:
  shank = _f(fields, "handlediameter")
  diameter = _f(fields, "diameter")
  return StudioTool(
    number=int(fields["number"]),
    id=fields.get("id", "").strip(),
    name=fields.get("name", "").strip(),
    type=fields.get("type", "").strip(),
    # The solder-mask tool has no handle diameter: its diameter is the shank.
    shank=shank or diameter,
    diameter=diameter,
    tip=_f(fields, "tipdiameter"),
    flute=_f(fields, "flutelength"),
    corner_radius=_f(fields, "cornerradius"),
    angle=_f(fields, "angle"),
    half_angle=_f(fields, "halfAngle"),
  )


def read_tools(path: str) -> tuple[list[StudioTool], list[int]]:
  """(tools from the Studio header, tool numbers the program changes to).

  The second list comes from the M6 lines of the program itself, in order
  of first use: it shows header tools the program never loads, and it is
  all there is for G-code that Makera Studio did not write.
  """
  tools: dict[int, StudioTool] = {}
  used: list[int] = []
  m6 = re.compile(r"^\s*(?:M0*6\s*T(\d+)|T(\d+)\s*M0*6)\b", re.IGNORECASE)
  with open(path, errors="replace") as fh:
    for line in fh:
      if line.startswith(HEADER):
        parts = line.rstrip("\r\n").split("|")
        if len(parts) > 2 and parts[1] == "TOOL":
          fields = dict(p.split("=", 1) for p in parts[2:] if "=" in p)
          if fields.get("number", "").lstrip("-").isdigit():
            tool = tool_from_fields(fields)
            tools[tool.number] = tool
      elif line[:1] != ";":
        hit = m6.match(line)
        if hit:
          n = int(hit[1] or hit[2])
          if n not in used:
            used.append(n)
  return [tools[n] for n in sorted(tools)], used


def kind_of(tool: StudioTool) -> str | None:
  """The catalogue's `kind` for a Studio tool, from its type and name."""
  text = f"{tool.type} {tool.name}".lower()
  for word, kind in (
    ("solder mask", "spring_tool"),
    ("ball nose engraving", "tapered_ball"),
    ("taper", "tapered_ball"),
    ("ball", "ballnose"),
    ("corn", "corn"),
    ("drill", "drill"),
    ("thread", "threadmill"),
    ("chamfer", "chamfer"),
    ("engrav", "vbit"),
    ("flat", "endmill"),
    ("end mill", "endmill"),
  ):
    if word in text:
      return kind
  return None


def _bit_metal(bit: dict) -> bool:
  return "for Metal" in bit["title"]


def _thread_size(text: str) -> float | None:
  hit = re.search(r"\bM(\d+(?:\.\d+)?)", text)
  return float(hit[1]) if hit else None


def _distance(tool: StudioTool, kind: str, bit: dict) -> tuple[float, bool]:
  """(how far the bit is from the tool, whether it is the same bit).

  The distance only ranks bits of one kind and shank against each other.
  """
  d_mm, cut_mm = bit["d"] * 1000, bit["cut"] * 1000
  if kind == "spring_tool":
    return 0.0, True
  if kind == "chamfer":
    da = abs(bit.get("angle", 0) - tool.angle) if tool.angle else 0.0
    return da, da <= EXACT_ANGLE_DEG
  if kind == "threadmill":
    want, have = _thread_size(tool.name), _thread_size(bit["title"])
    if want is None or have is None:
      return math.inf, False
    return abs(want - have), abs(want - have) < 1e-6
  if kind == "vbit":
    da = abs(bit.get("angle", 0) - 2 * tool.half_angle)
    dd = abs(d_mm - tool.tip)
    metal = _bit_metal(bit) != tool.metal
    exact = da <= EXACT_ANGLE_DEG and dd <= EXACT_D_MM and not metal
    return da / 10 + dd * 10 + (0.5 if metal else 0.0), exact
  if kind == "tapered_ball":
    ball = 2 * tool.corner_radius or tool.tip
    dd = abs(d_mm - ball)
    return dd, dd <= EXACT_D_MM
  # Cutters with a diameter and a flute length. Corn bits and drills come in
  # one finish; end mills and ball noses also "for Metal".
  dd, dc = abs(d_mm - tool.diameter), abs(cut_mm - tool.flute)
  metal = kind in ("endmill", "ballnose") and _bit_metal(bit) != tool.metal
  exact = dd <= EXACT_D_MM and dc <= EXACT_CUT_MM and not metal
  return dd * 10 + dc + (2.0 if metal else 0.0), exact


def library_fields(cutter: dict) -> dict:
  """A Studio library cutter as the fields of the header line it gets."""
  return {
    "number": "0",
    "id": cutter["g_ID"],
    "name": cutter["cutterName"],
    "type": CATEGORY_TYPES.get(cutter["cutterCategoryId"], ""),
    "handlediameter": str(cutter["cutterDiameter"]),
    "flutelength": str(cutter["cutterFluteLength"]),
    "diameter": str(cutter["cutterMaxDiameter"]),
    "tipdiameter": str(cutter["cutterTipDiameter"]),
    "cornerradius": str(cutter["cutterCornerRadius"]),
    "angle": str(cutter["cutterAngle"]),
    "halfAngle": str(cutter["cutterHalfAngle"]),
  }


def load_library(path: str = CUTTERS_JSON) -> list[StudioTool]:
  """Makera Studio's own tools; empty if the library file is not there."""
  if not os.path.exists(path):
    return []
  with open(path) as fh:
    return [
      tool_from_fields(library_fields(c)) for c in json.load(fh)["cutters"]
    ]


def library_tool_named_in(
  tool: StudioTool, library: list[StudioTool]
) -> StudioTool | None:
  """The library tool whose name a custom tool's name contains (the longest:
  "...Flat End" is also inside "...Flat End(Metal)"), with the same shank."""
  name = tool.name.lower()
  named = [
    t
    for t in library
    if t.name.lower() in name and abs(t.shank - tool.shank) < 0.02
  ]
  return max(named, key=lambda t: len(t.name), default=None)


def match_tool(
  tool: StudioTool, bits: list[dict], library: list[StudioTool] | None = None
) -> Match:
  if tool.custom and library:
    original = library_tool_named_in(tool, library)
    if original:
      match = match_tool(original, bits)
      if match.exact:
        return Match(
          tool, match.bit, True, f"your copy of Studio's '{original.name}'"
        )
  kind = kind_of(tool)
  if kind is None:
    return Match(tool, None, False, f"unknown tool type '{tool.type}'")
  same = [
    b
    for b in bits
    if b["kind"] == kind and abs(b["shank"] * 1000 - tool.shank) < 0.02
  ]
  if not same:
    return Match(
      tool, None, False, f"no {kind} bit with a {tool.shank:g} mm shank"
    )
  ranked = sorted(
    ((*_distance(tool, kind, b), b["id"], b) for b in same), key=lambda r: r[:3]
  )
  dist, exact, _, bit = ranked[0]
  if math.isinf(dist):
    return Match(tool, None, False, f"cannot tell which {kind} bit this is")
  if exact:
    return Match(tool, bit, True)
  if not tool.custom:
    # A library tool that is not in the catalogue: the library has changed.
    return Match(
      tool,
      None,
      False,
      f"Studio library tool {tool.id} is not in the bit catalogue "
      f"(closest: {bit['id']})",
    )
  return Match(tool, bit, False, f"custom tool, closest bit: {bit['title']}")


def load_bits(path: str = BITS_JSON) -> list[dict]:
  with open(path) as fh:
    return json.load(fh)["bits"]


def parse_map(specs: list[str]) -> dict[int, str]:
  """['2=fe18_1p5x8', 'T3=bit_corn18_1'] -> {2: 'fe18_1p5x8', 3: 'corn18_1'}."""
  out = {}
  for spec in specs:
    for part in filter(None, spec.split(",")):
      n, _, bit = part.partition("=")
      bit = bit.strip()
      out[int(n.strip().lstrip("Tt"))] = (
        bit[4:] if bit.startswith("bit_") else bit
      )
  return out


@dataclasses.dataclass(frozen=True)
class Plan:
  """What a job needs in the ATC.

  targets: ATC slot -> bit id, for every tool that could be placed.
  matches: one per tool of the header, in slot order.
  problems: what stops the plan from being used as it is.
  guesses: slots filled with the closest bit of a custom tool.
  """

  targets: dict[int, str]
  matches: list[Match]
  problems: list[str]
  guesses: list[int]

  @property
  def spec(self) -> str:
    return ",".join(f"{n}={b}" for n, b in sorted(self.targets.items()))


def plan(
  path: str,
  mapping: dict[int, str] | None = None,
  bits: list[dict] | None = None,
  library: list[StudioTool] | None = None,
) -> Plan:
  bits = load_bits() if bits is None else bits
  library = load_library() if library is None else library
  by_id = {b["id"]: b for b in bits}
  mapping = mapping or {}
  tools, used = read_tools(path)
  problems, guesses, matches, targets = [], [], [], {}
  if not tools:
    problems.append(
      "no ;@MKR|TOOL lines: not exported by Makera Studio"
      + (
        f" (the program changes to T{', T'.join(map(str, used))}; name their "
        "bits with --map)"
        if used
        else ""
      )
    )
    tools = [StudioTool(n, "", "", "", 0, 0, 0, 0, 0, 0, 0) for n in used]
  for n in mapping:
    if n not in [t.number for t in tools]:
      problems.append(f"--map names T{n}, which the job does not use")
  for tool in tools:
    if tool.number in mapping:
      bit = by_id.get(mapping[tool.number])
      if bit is None:
        problems.append(
          f"T{tool.number}: --map names an unknown bit '{mapping[tool.number]}'"
        )
      match = Match(tool, bit, bit is not None, "set with --map")
    elif not tool.name and not tool.type:
      match = Match(tool, None, False, "no tool data in the file")
    else:
      match = match_tool(tool, bits, library)
    matches.append(match)
    if tool.number not in ATC_SLOTS:
      problems.append(
        f"T{tool.number} ({tool.name}): the ATC has slots 1..6 only"
      )
      continue
    if match.bit is None:
      if tool.number not in mapping:
        problems.append(f"T{tool.number} ({tool.name}): {match.note}")
      continue
    if not match.exact:
      guesses.append(tool.number)
    targets[tool.number] = match.bit["id"]
  twice = {b for b in targets.values() if list(targets.values()).count(b) > 1}
  for b in sorted(twice):
    slots = ", ".join(f"T{n}" for n, x in sorted(targets.items()) if x == b)
    problems.append(f"{slots} all want bit {b}: the carousel holds one of each")
  unused = [t.number for t in tools if used and t.number not in used]
  if unused and tools and tools[0].name:
    problems.extend(
      f"T{n} is in the header but the program never changes to it"
      for n in unused
    )
  return Plan(targets, matches, problems, guesses)


def report(p: Plan) -> str:
  lines = []
  for m in p.matches:
    t = m.tool
    mark = "=" if m.exact else ("~" if m.bit else "?")
    lines.append(
      f"  T{t.number}  {t.name or '(no name)':48s} {mark} "
      f"{m.bit_id or '-':18s} {m.note}"
    )
  for msg in p.problems:
    lines.append(f"  ! {msg}")
  return "\n".join(lines)


def main():
  ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
  ap.add_argument("gcode", help="G-code file exported by Makera Studio (.nc)")
  ap.add_argument(
    "--map",
    action="append",
    default=[],
    help="Bit for a tool number, e.g. 2=fe18_1p5x8 (repeatable); "
    "overrides the match, needed for custom tools.",
  )
  ap.add_argument("--json", action="store_true", help="Print the plan as JSON.")
  args = ap.parse_args()
  p = plan(args.gcode, parse_map(args.map))
  if args.json:
    print(
      json.dumps(
        {
          "set_atc": p.spec,
          "targets": p.targets,
          "guesses": p.guesses,
          "problems": p.problems,
          "tools": [
            {
              "number": m.tool.number,
              "name": m.tool.name,
              "studio_id": m.tool.id,
              "bit": m.bit_id,
              "exact": m.exact,
              "note": m.note,
            }
            for m in p.matches
          ],
        },
        indent=2,
      )
    )
  else:
    print(report(p))
    if p.guesses:
      print(
        "  ~ = closest bit for a custom tool: check it, or name the bit with --map"
      )
    if p.targets:
      print(f"\nmove_bit --set_atc='{p.spec}'")
  sys.exit(1 if p.problems else 0)


if __name__ == "__main__":
  main()
