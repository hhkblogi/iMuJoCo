# BUILD file for MuJoCo Menagerie robot models (BSD-3 licensed)

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "agility_cassie",
    srcs = glob([
        "agility_cassie/**/*.xml",
        "agility_cassie/**/*.obj",
        "agility_cassie/**/*.png",
        "agility_cassie/LICENSE",
    ]),
)

filegroup(
    name = "unitree_g1",
    srcs = glob(
        [
            "unitree_g1/**/*.xml",
            "unitree_g1/**/*.stl",
            "unitree_g1/**/*.STL",
            "unitree_g1/LICENSE",
        ],
        allow_empty = True,  # .stl/.STL: only one case matches per platform
    ),
)

filegroup(
    name = "unitree_h1",
    srcs = glob(
        [
            "unitree_h1/**/*.xml",
            "unitree_h1/**/*.stl",
            "unitree_h1/**/*.STL",
            "unitree_h1/LICENSE",
        ],
        allow_empty = True,
    ),
)
