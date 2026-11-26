"""Plans and runs single SO-101 joint moves, or sets the arm's joints.

  bazel run -c opt //tools/atc:joint_probe -- 'ANY:[q1, q2, q3, q4, q5]' ...
  bazel run -c opt //tools/atc:joint_probe -- 'SET:[-1.571, -0.743, 0.43, 1.618, 0.0]'

ANY / JOINT: plan and run a joint move of that motion type, print OK or FAIL
with the collisions the planner reports. SET: put the arm at those joints in
the belief world and reset the simulator to it (no motion planning), e.g. to
get the arm out of a collision a failed move left it in; a world reset does
not move the arm.
"""
import json, sys
from intrinsic.solutions import deployments
from src.core.config import load_app_config
from src.hardware.robot import Robot
from intrinsic.world.python import object_world_resources

config = load_app_config("configs/carvera_so101/app_config.yaml")
solution = deployments.connect(address="localhost:17080")
arm = solution.world.get_kinematic_object(config.robot.arm_part_name)
print("start", [round(v, 3) for v in arm.joint_positions])
robot = Robot(solution=solution, config=config.robot)
for spec in sys.argv[1:]:
  motion, q = spec.split(":", 1)
  if motion == "SET":
    solution.world.update_joint_positions(arm, json.loads(q))
    solution.simulator.reset()
    print("SET ", q)
    continue
  q = object_world_resources.JointConfiguration(joint_position=json.loads(q))
  task = robot.build_move_joint_task(q, name="probe", motion_type=motion,
                                     excluded_collision_pairs=[("so101_gripper", "so101_arm")])
  try:
    solution.executive.run(task)
    print("OK  ", motion, q)
  except Exception as e:  # pylint: disable=broad-except
    msg = str(e)
    keep = [l.strip() for l in msg.splitlines() if any(k in l for k in ("Left", "Right", "Reason", "Config", "could not", "Invalid"))]
    print("FAIL", motion, q, "|", " / ".join(dict.fromkeys(keep))[:600])
