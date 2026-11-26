load("@intrinsic-core//bazel:python.bzl", "py_library")
load("@intrinsic-core//intrinsic/scene/build_defs:sdf_scene_object.bzl", "sdf_scene_object")
load("@intrinsic-core//intrinsic_sdk/intrinsic/assets/scene_objects/build_defs:scene_object.bzl", "intrinsic_scene_object")

package(default_visibility = ["//visibility:public"])

# Sim model generated with the Carvera model: see
# models/makera_carvera/build_carvera.py (layout in carousel_layout.py).
# Fabrication files (OpenSCAD, STL, DXF) are in fab/.

sdf_scene_object(
    name = "bit_carousel_scene_object",
    src = "bit_carousel.sdf",
    sdf_assets = [
        "carousel_base.glb",
        "carousel_platter.glb",
    ],
)

intrinsic_scene_object(
    name = "bit_carousel",
    manifest = ":bit_carousel_manifest.textproto",
    scene_object = ":bit_carousel_scene_object",
)

py_library(
    name = "carousel_layout",
    srcs = ["carousel_layout.py"],
)
