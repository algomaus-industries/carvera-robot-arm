load("@intrinsic-core//intrinsic/scene/build_defs:sdf_scene_object.bzl", "sdf_scene_object")
load("@intrinsic-core//intrinsic_sdk/intrinsic/assets/scene_objects/build_defs:scene_object.bzl", "intrinsic_scene_object")

package(default_visibility = ["//visibility:public"])

# Generated with the Carvera model: see models/makera_carvera/build_carvera.py.

sdf_scene_object(
    name = "bit_grid_21_scene_object",
    src = "bit_grid_21.sdf",
    sdf_assets = ["bit_grid_21.glb"],
)

intrinsic_scene_object(
    name = "bit_grid_21",
    manifest = ":bit_grid_21_manifest.textproto",
    scene_object = ":bit_grid_21_scene_object",
)
