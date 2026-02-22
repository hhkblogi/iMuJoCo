# BUILD file for Unitree RL Gym G1 12-DOF model (BSD-3 licensed)
# Source: https://github.com/unitreerobotics/unitree_rl_gym

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "unitree_g1_rl",
    srcs = glob(
        [
            "unitree_g1_rl/**/*.xml",
            "unitree_g1_rl/**/*.stl",
            "unitree_g1_rl/**/*.STL",
            "unitree_g1_rl/LICENSE*",
        ],
        allow_empty = True,  # .stl/.STL: only one case matches per platform
    ),
)
