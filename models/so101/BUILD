load("@intrinsic-core//bazel:container.bzl", "container_image")
load("@intrinsic-core//intrinsic/scene/build_defs:sdf_scene_object.bzl", "sdf_scene_object")
load("@intrinsic-core//intrinsic_control/intrinsic/icon/hal/bzl:hardware_module_image.bzl", "hardware_module_image")
load("@intrinsic-core//intrinsic_control/intrinsic/icon/hal/bzl:resources.bzl", "hardware_module_manifest", "wrap_in_any_textproto")
load("@intrinsic-core//intrinsic_sdk/intrinsic/assets/hardware_devices/build_defs:hardware_device.bzl", "intrinsic_hardware_device")
load("@intrinsic-core//intrinsic_sdk/intrinsic/assets/scene_objects/build_defs:scene_object.bzl", "intrinsic_scene_object")
load("@intrinsic-core//intrinsic_sdk/intrinsic/assets/services/build_defs:services.bzl", "intrinsic_service")

package(default_visibility = ["//visibility:public"])

# SO-ARM101 as an ICON hardware device, simulation only, wired like
# ur_hardware_device() in intrinsic-core: a scene object (the robot) plus a
# hardware-module service. In sim, Gazebo runs the module and infers the joint
# interfaces from the scene object; the service image is the generic
# gazebo_hwm_stub, which only forwards health. A real-servo driver (Feetech
# STS3215 bus) would replace `image` later.
#
# so101.sdf and the .glb files are generated:
#   models/so101/fetch_meshes.sh        # official meshes, not checked in
#   blender -b --python models/so101/build_so101.py -- models

filegroup(
    name = "so101_meshes",
    srcs = glob(["so101_*.glb"]),
)

sdf_scene_object(
    name = "so101_scene_object",
    src = "so101.sdf",
    sdf_assets = [":so101_meshes"],
    updates_pbtxts = ["so101_limits.updates.pbtxt"],
)

intrinsic_scene_object(
    name = "so101_hardware_module_scene_object_node",
    manifest = ":so101_scene_object_manifest.textproto",
    scene_object = ":so101_scene_object",
    visibility = ["//visibility:private"],
)

# Same as @intrinsic-core//intrinsic/kubernetes:base-image-oci (not visible
# outside @intrinsic-core).
container_image(
    name = "hwm_base_image",
    base = "@distroless_base",
    entrypoint = [
        "/dumb-init",
        "--",
    ],
    files = ["@dumb-init"],
)

hardware_module_image(
    name = "so101_sim_hwm_image",
    base_image = ":hwm_base_image",
    hardware_module_binary = "@intrinsic-core//intrinsic_control/intrinsic/icon/hardware_modules/gazebo_hwm_stub:gazebo_hwm_stub_main",
)

hardware_module_manifest(
    name = "so101_service_manifest_compiled",
    image = ":so101_sim_hwm_image.tar",
    image_sim = ":so101_sim_hwm_image.tar",
    manifest = ":so101_service_manifest.textproto",
)

wrap_in_any_textproto(
    name = "so101_default_config",
    src = "so101_default_config.pbtxt",
)

intrinsic_service(
    name = "so101_hardware_module_service_node",
    default_config = ":so101_default_config",
    images = [":so101_sim_hwm_image.tar"],
    manifest = ":so101_service_manifest_compiled",
    visibility = ["//visibility:private"],
    deps = [
        "@intrinsic_apis//intrinsic/assets/services/proto/v1:service_state_proto",
        "@intrinsic_apis//intrinsic/icon/hal/proto:hardware_module_config_proto",
        "@intrinsic_apis//intrinsic/icon/hardware_modules/sim_bus:sim_bus_hardware_module_proto",
    ],
)

intrinsic_hardware_device(
    name = "so101_hardware_module",
    assets = [
        ":so101_hardware_module_scene_object_node",
        ":so101_hardware_module_service_node",
    ],
    manifest = ":so101_hardware_module_manifest.textproto",
)
