"""Tests for tools/atc/gcode_tools.py: Makera Studio G-code header -> bits."""

import json
import os
import tempfile
import unittest

from tools.atc import gcode_tools

REPO = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..")
CUTTERS_JSON = os.path.join(REPO, "models", "makera_bits", "makera_studio_cutters.json")
# Makera Studio's tool categories (t_MakeraCutterList.cutterCategoryId) as the
# header's type field. "Flat End" and "Engraving" are as Studio 0.1.3 writes
# them; the others are not confirmed, and the matcher also reads the name.
TYPES = {0: "Ball Nose", 1: "Flat End", 2: "Chamfer", 3: "Engraving", 5: "Drill",
         6: "Thread", 7: "Ball Nose Engraving"}

# Header lines as Makera Studio 0.1.3.0 wrote them (2026-11).
JOB = """;@MKR|BEGIN
;@MKR|SCHEMA|v=1.0.0
;@MKR|MACHINE|id=C1|name=Carvera
;@MKR|TOOL|number=1|id=122111023830|name=3.175*0.2mm*30º Engraving(Metal)|type=Engraving|handlediameter=3.175|sticklength=0|shoulderlength=0|flutelength=5|diameter=3.175|tipdiameter=0.2|cornerradius=0|angle=0|halfAngle=15
;@MKR|TOOL|number=2|id=111218083806|name=3.175*0.8*5.5mm Corn|type=Flat End|handlediameter=3.175|sticklength=0|shoulderlength=5.5|flutelength=5.5|diameter=0.8|tipdiameter=0.8|cornerradius=0|angle=0|halfAngle=0
;@MKR|TOOL|number=5|id=122011034730|name=3.175mm Solder Mask Removal|type=Engraving|handlediameter=0|sticklength=0|shoulderlength=0|flutelength=3|diameter=3.175|tipdiameter=0.3|cornerradius=0|angle=0|halfAngle=15
;@MKR|END
; T1-3.175*0.2mm*30 Engraving(Metal)
T1 M6
G0 X0 Y0
M6 T2
G1 X1 F100
T5 M06
"""
CUSTOM = (";@MKR|TOOL|number=2|id=|name=Custom 3.175*1.5*6mm Flat End(Metal) - adjusted diameter"
          "|type=Flat End|handlediameter=3.175|sticklength=0|shoulderlength=6|flutelength=6"
          "|diameter=1.32|tipdiameter=1.32|cornerradius=0|angle=0|halfAngle=0\nT2 M6\n")


def _plan(text, mapping=None):
  with tempfile.NamedTemporaryFile("w", suffix=".nc", delete=False) as fh:
    fh.write(text)
  try:
    return gcode_tools.plan(fh.name, mapping)
  finally:
    os.unlink(fh.name)


def _header_fields(cutter):
  return {
    "number": "1", "id": cutter["g_ID"], "name": cutter["cutterName"],
    "type": TYPES[cutter["cutterCategoryId"]],
    "handlediameter": str(cutter["cutterDiameter"]),
    "flutelength": str(cutter["cutterFluteLength"]),
    "diameter": str(cutter["cutterMaxDiameter"]),
    "tipdiameter": str(cutter["cutterTipDiameter"]),
    "cornerradius": str(cutter["cutterCornerRadius"]),
    "angle": str(cutter["cutterAngle"]),
    "halfAngle": str(cutter["cutterHalfAngle"]),
  }


class GcodeToolsTest(unittest.TestCase):

  def test_every_studio_library_tool_is_exactly_one_bit(self):
    with open(CUTTERS_JSON) as fh:
      cutters = json.load(fh)["cutters"]
    bits = gcode_tools.load_bits()
    found = {}
    for cutter in cutters:
      tool = gcode_tools.tool_from_fields(_header_fields(cutter))
      match = gcode_tools.match_tool(tool, bits)
      self.assertTrue(match.exact, f"{cutter['cutterName']}: {match.note}")
      self.assertNotIn(match.bit_id, found, cutter["cutterName"])
      found[match.bit_id] = cutter["cutterName"]
    self.assertEqual(set(found), {b["id"] for b in bits})

  def test_studio_job(self):
    job = _plan(JOB)
    self.assertEqual(job.targets, {1: "engm18_30x0p2", 2: "corn18_0p8", 5: "uv_soldermask"})
    self.assertEqual(job.spec, "1=engm18_30x0p2,2=corn18_0p8,5=uv_soldermask")
    self.assertEqual((job.problems, job.guesses), ([], []))

  def test_custom_tool_is_a_guess(self):
    job = _plan(CUSTOM)
    self.assertEqual(job.targets, {2: "oflm18_1p5x6"})
    self.assertEqual(job.guesses, [2])
    self.assertEqual(job.problems, [])

  def test_map_overrides_and_is_not_a_guess(self):
    job = _plan(CUSTOM, {2: "fe18_1p5x8"})
    self.assertEqual((job.targets, job.guesses, job.problems), ({2: "fe18_1p5x8"}, [], []))
    self.assertTrue(_plan(CUSTOM, {2: "no_such_bit"}).problems)
    self.assertTrue(_plan(CUSTOM, {3: "corn18_1"}).problems)

  def test_parse_map(self):
    self.assertEqual(gcode_tools.parse_map(["2=bit_corn18_1,T3=corn18_2", "6=uv_soldermask"]),
                     {2: "corn18_1", 3: "corn18_2", 6: "uv_soldermask"})

  def test_gcode_without_studio_header(self):
    job = _plan("G21\nT3 M6\nG0 X0\nM6 T1\n")
    self.assertEqual(job.targets, {})
    self.assertIn("T3, T1", job.problems[0])
    job = _plan("G21\nT3 M6\n", {3: "corn18_1"})
    self.assertEqual(job.targets, {3: "corn18_1"})

  def test_problems(self):
    seven = JOB.replace("number=5", "number=7").replace("T5 M06", "T7 M06")
    self.assertTrue(any("slots 1..6" in p for p in _plan(seven).problems))
    twice = JOB.replace("number=5|id=122011034730|name=3.175mm Solder Mask Removal",
                        "number=5|id=111218083806|name=3.175*0.8*5.5mm Corn")
    twice = twice.replace("type=Engraving|handlediameter=0|sticklength=0|shoulderlength=0|flutelength=3"
                          "|diameter=3.175|tipdiameter=0.3",
                          "type=Flat End|handlediameter=3.175|sticklength=0|shoulderlength=5.5"
                          "|flutelength=5.5|diameter=0.8|tipdiameter=0.8")
    self.assertTrue(any("one of each" in p for p in _plan(twice).problems))
    unused = JOB.replace("T5 M06\n", "")
    self.assertTrue(any("never changes to it" in p for p in _plan(unused).problems))
    gone = JOB.replace("3.175*0.8*5.5mm Corn", "3.175*0.85*5.5mm Corn").replace(
      "diameter=0.8|tipdiameter=0.8", "diameter=0.85|tipdiameter=0.85")
    self.assertTrue(any("not in the bit catalogue" in p for p in _plan(gone).problems))


if __name__ == "__main__":
  unittest.main()
