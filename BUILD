load("@bazel_skylib//rules:common_settings.bzl", "string_flag")
load("@intrinsic-core//intrinsic/assets/build_defs:solution.bzl", "intrinsic_solution")
load("@intrinsic-core//intrinsic_sdk/intrinsic/assets/build_defs:asset.bzl", "intrinsic_asset_instance")
load("//bazel:imported_asset.bzl", "imported_asset_bundle")
load("//bazel:solution_manifest.bzl", "solution_manifest_only")
load("//models/makera_bits:bits.bzl", "MAKERA_BITS")

package(default_visibility = ["//visibility:public"])

# Flag to parameterize hardware setup (omts vs lab_bb_01 vs carvera_so101).
#
# carvera_so101: simulation-only cell with an SO-ARM101 hobby arm tending the
# Carvera ATC from a rolling cart (models/so101, configs/carvera_so101/). No
# camera, perception, Hand-E or UR in it.
#
# configs/kr_10/ exists but is not a value here: the solution below is wired to
# the UR hardware modules throughout, so selecting a KUKA cell would produce a
# solution that cannot start. Add the value with the rest of the KUKA wiring.
string_flag(
    name = "setup",
    build_setting_default = "omts",
    values = [
        "omts",
        "lab_bb_01",
        "carvera_so101",
    ],
)

config_setting(
    name = "is_carvera_so101",
    flag_values = {
        ":setup": "carvera_so101",
    },
)

config_setting(
    name = "is_lab_bb_01",
    flag_values = {
        ":setup": "lab_bb_01",
    },
)

# Flag to swap the lab_bb_01 workpiece for a mesh grasping demo part.
# The demo parts are simulation only; see configs/lab_bb_01_objects/.
string_flag(
    name = "workpiece",
    build_setting_default = "raw_stock_2x3x5",
    values = [
        "raw_stock_2x3x5",
        "banana",
        "bracket",
    ],
)

config_setting(
    name = "is_lab_banana",
    flag_values = {
        ":setup": "lab_bb_01",
        ":workpiece": "banana",
    },
)

config_setting(
    name = "is_lab_bracket",
    flag_values = {
        ":setup": "lab_bb_01",
        ":workpiece": "bracket",
    },
)

# Every bit Makera sells for the Carvera, one instance each (bit_<id>), placed
# in the bit grid by configs/<cell>/bit_grid.updates.pbtxt / bits.updates.pbtxt.
BIT_ASSETS = ["//models/makera_bits:" + bit for bit in MAKERA_BITS]

BIT_INSTANCES = [":bit_" + bit for bit in MAKERA_BITS]

_SO101_ASSETS = [
    "//models/so101:so101_hardware_module",
    "//models/so101_gripper",
    "//models/so101_stand",
    "//models/carvera_bench",
    "//models/robot_cart",
    "//models/bit_carousel",
    "//models/makera_carvera",
    "//models/schunk_egp_64nnb",
]

_OMTS_SKILL_ASSETS = [
    ":hand_e_gripper_cmd_skill_asset",
    ":moveit_plan_grasp_skill_asset",
    "//src/skills/antipodal_mesh_plan_grasp:antipodal_mesh_plan_grasp_skill",
    "@intrinsic-core//intrinsic_control/intrinsic/icon/skills:dio_read_input_skill",
    "@intrinsic-core//intrinsic_control/intrinsic/icon/skills:dio_set_output_skill",
    "@intrinsic-core//intrinsic_perception/intrinsic/perception/skills:capture_images_skill",
    "@intrinsic-core//intrinsic_perception/intrinsic/perception/skills/calibration:calibrate_camera_to_robot_skill",
    "@intrinsic-core//intrinsic_perception/intrinsic/perception/skills/calibration:collect_calibration_data_skill",
    "@intrinsic-core//intrinsic_perception/intrinsic/perception/skills/calibration:initialize_calibration_skill",
    "@intrinsic-core//intrinsic_perception/intrinsic/perception/skills/calibration:sample_calibration_poses_skill",
    "@intrinsic-core//intrinsic_perception/intrinsic/perception/skills/multi_view:estimate_pose_multi_view_skill",
    "@intrinsic-core//incode/motion_planning/skills:clear_motion_planner_service_cache_skill",
    "@intrinsic-core//incode/motion_planning/skills:move_robot_skill",
    "@intrinsic-core//incode/motion_planning/skills:preplan_motion_skill",
    "@intrinsic-core//intrinsic/manipulation/skills/force:move_to_contact_skill",
    "@intrinsic-core//intrinsic/skills/apps:attach_object_to_robot_skill",
    "@intrinsic-core//intrinsic/skills/apps:detach_object_skill",
    "@intrinsic-core//intrinsic/skills/apps:update_world_skill",
    "@intrinsic-core//intrinsic/world/skills/create_object:create_object_skill",
]

# Solution deployment definition
intrinsic_solution(
    name = "omts_solution",
    assets = _OMTS_SKILL_ASSETS + [
        "@intrinsic-core//intrinsic/resources/catalog/resourcedata/gripper:robotiq_pinch_gripper_resource_type",
        "@intrinsic-core//intrinsic/simulation/gazebo/asset:gazebo_simulator_type",
        "@intrinsic-core//intrinsic_perception/intrinsic/perception/calibration/charuco_boards:charuco_9x12_30mm_22mm_dict_5x5",
        "@intrinsic-core//intrinsic_perception/intrinsic/perception/calibration/charuco_boards:charuco_9x12_30mm_22mm_dict_5x5_estimator",
        "@intrinsic-core//intrinsic_perception/intrinsic/perception/calibration/charuco_boards:charuco_9x14_20mm_15mm_dict_5x5",
        "@intrinsic-core//intrinsic_perception/intrinsic/perception/calibration/charuco_boards:charuco_9x14_20mm_15mm_dict_5x5_estimator",
        "@intrinsic-core//intrinsic_perception/intrinsic/perception/calibration/services/v1:calibration_service",
        "@intrinsic-core//intrinsic_perception/intrinsic/perception/cameras/hardware_devices:orbbec_gemini_335le_hardware_device",
        "@intrinsic-core//intrinsic_motion_planning/intrinsic/motion_planning/service:motion_planner_service_asset",
        "@intrinsic-core//intrinsic_control/intrinsic/icon/machines/common:generic_icon_mainloop_type",
        "@intrinsic-core//intrinsic_perception/intrinsic/perception/service/nvidia_pose_estimator:nvidia_pose_estimator_service_asset",
        "@intrinsic-core//intrinsic_perception/intrinsic/perception/service/ioc_train_service:ioc_train_service_asset",
        ":flowstate_ros_bridge_asset",
        ":hand_e_gripper_service_asset",
        ":orbbec_gemini_driver_asset",
        "//models/raw_stock_2x3x5",
    ] + select({
        ":is_lab_banana": ["//models/banana"],
        ":is_lab_bracket": ["//models/bracket"],
        "//conditions:default": [],
    }) + select({
        ":is_lab_bb_01": [
            "@intrinsic-core//intrinsic/apps/bluebird_caw/resources:caw_enclosure",
            "@intrinsic-core//intrinsic_control/intrinsic/icon/hardware_modules/universal_robots:ur3e_hardware_module_core",
        ],
        ":is_carvera_so101": _SO101_ASSETS,
        "//conditions:default": [
            "@intrinsic-core//intrinsic_control/intrinsic/icon/hardware_modules/universal_robots:ur5e_hardware_module_core",
            "//models/camera_mount",
            "//models/bit_grid",
            "//models/makera_carvera",
            "//models/omts_enclosure",
            "//models/schunk_egp_64nnb",
        ],
    }) + select({
        ":is_lab_bb_01": [],
        "//conditions:default": BIT_ASSETS,
    }),
    default_operation_mode = "real",
    instances = [
        ":gazebo_simulator",
        ":icon",
        ":motion_planner_service",
        ":flowstate_ros_bridge",
    ] + select({
        ":is_carvera_so101": [
            ":so101_arm",
            ":so101_gripper",
            ":so101_stand",
            ":carvera_bench",
            ":robot_cart",
        ],
        "//conditions:default": [
            ":enclosure",
            ":robotiq_pinch_gripper",
            ":charuco_9x12_30mm_22mm_dict_5x5",
            ":charuco_9x14_20mm_15mm_dict_5x5",
            ":calibration_service_instance",
            ":ur_module",
            ":orbbec_camera",
            ":orbbec_gemini_driver",
            ":nvidia_pose_estimator_service",
            ":train_service",
            ":hand_e_gripper_service",
        ],
    }) + select({
        ":is_lab_banana": [":banana"],
        ":is_lab_bracket": [":bracket"],
        ":is_carvera_so101": [],
        "//conditions:default": [":raw_stock_2x3x5"],
    }) + select({
        ":is_lab_bb_01": [],
        "//conditions:default": BIT_INSTANCES,
    }) + select({
        ":is_lab_bb_01": [
        ],
        ":is_carvera_so101": [
            ":cnc_enclosure",
            ":bit_grid",
            ":schunk_egp_64nnb",
        ],
        "//conditions:default": [
            ":camera_mount",
            ":cnc_enclosure",
            ":bit_grid",
            ":schunk_egp_64nnb",
        ],
    }),
    object_world_updates = select({
        ":is_lab_banana": [
            "//configs:lab_bb_01/ur_module.attachments.updates.pbtxt",
            "//configs:lab_bb_01_objects/waypoints.updates.pbtxt",
            "//configs:lab_bb_01_objects/banana.updates.pbtxt",
            "//configs:lab_bb_01/align_robot.updates.pbtxt",
            "//configs:lab_bb_01/orbbec_gemini.updates.pbtxt",
        ],
        ":is_lab_bracket": [
            "//configs:lab_bb_01/ur_module.attachments.updates.pbtxt",
            "//configs:lab_bb_01_objects/waypoints.updates.pbtxt",
            "//configs:lab_bb_01_objects/bracket.updates.pbtxt",
            "//configs:lab_bb_01/align_robot.updates.pbtxt",
            "//configs:lab_bb_01/orbbec_gemini.updates.pbtxt",
        ],
        ":is_lab_bb_01": [
            "//configs:lab_bb_01/ur_module.attachments.updates.pbtxt",
            "//configs:lab_bb_01/scene.updates.pbtxt",
            "//configs:lab_bb_01/align_robot.updates.pbtxt",
            "//configs:lab_bb_01/orbbec_gemini.updates.pbtxt",
        ],
        ":is_carvera_so101": [
            "//configs:carvera_so101/cell.updates.pbtxt",
            "//configs:omts/schunk.updates.pbtxt",
            "//configs:carvera_so101/bits.updates.pbtxt",
        ],
        "//conditions:default": [
            "//configs:omts/ur_module.limits.updates.pbtxt",
            "//configs:omts/ur_module.attachments.updates.pbtxt",
            "//configs:omts/scene.updates.pbtxt",
            "//configs:omts/align_robot.updates.pbtxt",
            "//configs:omts/cnc_enclosure.updates.pbtxt",
            "//configs:omts/schunk.updates.pbtxt",
            "//configs:omts/carvera_atc.updates.pbtxt",
            "//configs:omts/bit_grid.updates.pbtxt",
            "//configs:omts/camera_mount.updates.pbtxt",
            "//configs:omts/orbbec_gemini.updates.pbtxt",
        ],
    }),
)

# Default aliases
alias(
    name = "omts",
    actual = ":omts_solution",
)

alias(
    name = "omts_app",
    actual = "//src:omts_app",
)

intrinsic_asset_instance(
    name = "enclosure",
    asset = select({
        ":is_lab_bb_01": "ai.intrinsic.caw_enclosure",
        "//conditions:default": "ai.intrinsic.omts_enclosure",
    }),
    instance_name = "enclosure",
)

# The CNC is a Makera Carvera. The instance keeps the name "cnc_enclosure" so
# app_config.yaml, the tools and the workshop guide refer to it unchanged.
intrinsic_asset_instance(
    name = "cnc_enclosure",
    asset = "ai.intrinsic.makera_carvera",
    instance_name = "cnc_enclosure",
)

# Tool-bit storage grid (35 mm pitch for the Hand-E; 21 mm in the SO-101 cell)
# and the Makera bits in it.
intrinsic_asset_instance(
    name = "bit_grid",
    asset = select({
        ":is_carvera_so101": "ai.intrinsic.bit_carousel",
        "//conditions:default": "ai.intrinsic.bit_grid",
    }),
    instance_name = "bit_grid",
)

[
    intrinsic_asset_instance(
        name = "bit_" + bit,
        asset = "ai.intrinsic.makera_bit_" + bit,
        instance_name = "bit_" + bit,
    )
    for bit in MAKERA_BITS
]

# SO-101 cell (--//:setup=carvera_so101).
intrinsic_asset_instance(
    name = "so101_arm",
    asset = "ai.intrinsic.so101_hardware_module",
    instance_name = "so101_arm",
    service_config = "//configs:carvera_so101/so101_module_config.textproto",
)

intrinsic_asset_instance(
    name = "so101_gripper",
    asset = "ai.intrinsic.so101_gripper",
    instance_name = "so101_gripper",
)

intrinsic_asset_instance(
    name = "so101_stand",
    asset = "ai.intrinsic.so101_stand",
    instance_name = "so101_stand",
)

intrinsic_asset_instance(
    name = "carvera_bench",
    asset = "ai.intrinsic.carvera_bench",
    instance_name = "carvera_bench",
)

intrinsic_asset_instance(
    name = "robot_cart",
    asset = "ai.intrinsic.robot_cart",
    instance_name = "robot_cart",
)

intrinsic_asset_instance(
    name = "camera_mount",
    asset = "ai.intrinsic.camera_mount",
)

intrinsic_asset_instance(
    name = "robotiq_pinch_gripper",
    asset = "ai.intrinsic.robotiq_gripper",
    instance_name = "gripper",
)

intrinsic_asset_instance(
    name = "raw_stock_2x3x5",
    asset = "ai.intrinsic.raw_stock_2x3x5",
)

intrinsic_asset_instance(
    name = "banana",
    asset = "ai.intrinsic.banana",
)

intrinsic_asset_instance(
    name = "bracket",
    asset = "ai.intrinsic.bracket",
)

intrinsic_asset_instance(
    name = "charuco_9x12_30mm_22mm_dict_5x5",
    asset = "ai.intrinsic.charuco_9x12_30mm_22mm_dict_5x5",
)

intrinsic_asset_instance(
    name = "charuco_9x14_20mm_15mm_dict_5x5",
    asset = "ai.intrinsic.charuco_9x14_20mm_15mm_dict_5x5",
)

intrinsic_asset_instance(
    name = "gazebo_simulator",
    asset = "ai.intrinsic.gazebo_simulator",
)

intrinsic_asset_instance(
    name = "calibration_service_instance",
    asset = "ai.intrinsic.calibration_service",
    instance_name = "calibration_service",
)

intrinsic_asset_instance(
    name = "icon",
    asset = "ai.intrinsic.generic_realtime_control_service",
    instance_name = "icon",
    service_config = select({
        ":is_lab_bb_01": "//configs:lab_bb_01/icon_config.textproto",
        ":is_carvera_so101": "//configs:carvera_so101/icon_config.textproto",
        "//conditions:default": "//configs:omts/icon_config.textproto",
    }),
)

intrinsic_asset_instance(
    name = "ur_module",
    asset = select({
        ":is_lab_bb_01": "ai.intrinsic.ur3e_hardware_module_core",
        "//conditions:default": "ai.intrinsic.ur5e_hardware_module_core",
    }),
    instance_name = "ur_module",
    service_config = select({
        ":is_lab_bb_01": "//configs:lab_bb_01/ur_module_config.textproto",
        "//conditions:default": "//configs:omts/ur_module_config.textproto",
    }),
)

intrinsic_asset_instance(
    name = "orbbec_camera",
    asset = "ai.intrinsic.orbbec_gemini_335le",
    instance_name = "orbbec_camera",
    service_config = select({
        ":is_lab_bb_01": "//configs:lab_bb_01/gemini_device_config.textproto",
        "//conditions:default": "//configs:omts/gemini_device_config.textproto",
    }),
)

intrinsic_asset_instance(
    name = "motion_planner_service",
    asset = "ai.intrinsic.motion_planner_service",
    instance_name = "motion_planner_service",
)

intrinsic_asset_instance(
    name = "nvidia_pose_estimator_service",
    asset = "ai.intrinsic.nvidia_pose_estimator_service",
    instance_name = "nvidia_pose_estimator_service",
)

intrinsic_asset_instance(
    name = "train_service",
    asset = "ai.intrinsic.ioc_train_service",
    instance_name = "train_service",
)

imported_asset_bundle(
    name = "flowstate_ros_bridge_asset",
    bundle = "@flowstate_ros_bridge_bundle//:flowstate_ros_bridge.bundle.tar",
    manifest = "//configs:common/flowstate_ros_bridge_manifest.textproto",
)

intrinsic_asset_instance(
    name = "flowstate_ros_bridge",
    asset = "ai.intrinsic.flowstate_ros_bridge",
    instance_name = "flowstate_ros_bridge",
)

# assetlocalinfogen parses skill manifests as binary proto, so the manifest is
# taken straight out of the bundle rather than being maintained as a checked-in
# copy that could drift from the released tarball.
genrule(
    name = "hand_e_gripper_cmd_skill_manifest",
    srcs = ["@hand_e_gripper_cmd_skill_bundle//:hand_e_gripper_cmd_skill.bundle.tar"],
    outs = ["hand_e_gripper_cmd_skill_manifest.binpb"],
    cmd = "tar -xOf $< skill_manifest.binpb > $@",
)

genrule(
    name = "hand_e_gripper_cmd_skill_fds",
    srcs = ["@hand_e_gripper_cmd_skill_bundle//:hand_e_gripper_cmd_skill.bundle.tar"],
    outs = ["hand_e_gripper_cmd_skill_fds.binpb"],
    cmd = "tar -xOf $< descriptors-transitive-descriptor-set.proto.bin > $@",
)

imported_asset_bundle(
    name = "hand_e_gripper_cmd_skill_asset",
    asset_type = "ASSET_TYPE_SKILL",
    bundle = "@hand_e_gripper_cmd_skill_bundle//:hand_e_gripper_cmd_skill.bundle.tar",
    file_descriptor_set = ":hand_e_gripper_cmd_skill_fds.binpb",
    manifest = ":hand_e_gripper_cmd_skill_manifest.binpb",
)

genrule(
    name = "moveit_plan_grasp_skill_manifest",
    srcs = ["@moveit_plan_grasp_skill_bundle//:moveit_plan_grasp_skill.bundle.tar"],
    outs = ["moveit_plan_grasp_skill_manifest.binpb"],
    cmd = "tar -xOf $< skill_manifest.binpb > $@",
)

genrule(
    name = "moveit_plan_grasp_skill_fds",
    srcs = ["@moveit_plan_grasp_skill_bundle//:moveit_plan_grasp_skill.bundle.tar"],
    outs = ["moveit_plan_grasp_skill_fds.binpb"],
    cmd = "tar -xOf $< descriptors-transitive-descriptor-set.proto.bin > $@",
)

imported_asset_bundle(
    name = "moveit_plan_grasp_skill_asset",
    asset_type = "ASSET_TYPE_SKILL",
    bundle = "@moveit_plan_grasp_skill_bundle//:moveit_plan_grasp_skill.bundle.tar",
    file_descriptor_set = ":moveit_plan_grasp_skill_fds.binpb",
    manifest = ":moveit_plan_grasp_skill_manifest.binpb",
)

solution_manifest_only(
    name = "omts_solution_manifest",
    extra_manifests = [
        ":BUILD",
        ":hand_e_gripper_cmd_skill_manifest",
        ":moveit_plan_grasp_skill_manifest",
    ],
    skills = _OMTS_SKILL_ASSETS,
)

imported_asset_bundle(
    name = "hand_e_gripper_service_asset",
    bundle = "@hand_e_gripper_service_bundle//:hand_e_gripper_service.bundle.tar",
    manifest = "//configs:common/hande_gripper_service_manifest.textproto",
)

intrinsic_asset_instance(
    name = "hand_e_gripper_service",
    asset = "ai.intrinsic.hande_gripper_aquarium_hande_gripper_launch_xml",
    instance_name = "hande_gripper",
)

imported_asset_bundle(
    name = "orbbec_gemini_driver_asset",
    bundle = "@orbbec_gemini_driver_bundle//:orbbec_gemini_driver.bundle.tar",
    manifest = "//configs:common/orbbec_gemini_driver_manifest.textproto",
)

intrinsic_asset_instance(
    name = "orbbec_gemini_driver",
    asset = "ai.intrinsic.orbbec_gemini_driver",
    instance_name = "orbbec_gemini_driver",
)

intrinsic_asset_instance(
    name = "schunk_egp_64nnb",
    asset = "ai.intrinsic.schunk_egp_64nnb",
    instance_name = "schunk_egp_64nnb",
)
