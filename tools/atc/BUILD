load("@intrinsic-core//bazel:python.bzl", "py_binary", "py_library")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "move_bit_lib",
    srcs = ["move_bit.py"],
    data = [
        "//configs:all_configs",
        "//models/makera_bits:makera_bits.json",
    ],
    deps = [
        "//src/core:config",
        "//src/hardware:gripper",
        "//src/hardware:machine",
        "//src/hardware:robot",
        "@intrinsic-core//intrinsic_sdk/intrinsic/solutions:behavior_tree",
        "@intrinsic-core//intrinsic_sdk/intrinsic/solutions:deployments",
        "@intrinsic-core//intrinsic_sdk/intrinsic/solutions:execution",
        "@intrinsic_apis//intrinsic/world/proto:object_world_refs_py_pb2",
        "@intrinsic-core//intrinsic_sdk/intrinsic/world/proto:object_world_updates_py_pb2",
    ],
)

py_binary(
    name = "move_bit",
    srcs = ["move_bit.py"],
    data = [
        "//configs:all_configs",
        "//models/makera_bits:makera_bits.json",
    ],
    main = "move_bit.py",
    deps = [":move_bit_lib"],
)
