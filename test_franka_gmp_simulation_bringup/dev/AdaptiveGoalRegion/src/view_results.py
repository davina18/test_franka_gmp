import argparse
import time

import numpy as np

from visualizer import visualize_grasps


def read_floats_from_file(file_path):
    data = []
    with open(file_path, "r") as file:
        for line in file:
            float_values = [float(value) for value in line.split()]
            data.append(float_values)
    return data


def get_data(path):
    with np.load(path, allow_pickle=True) as data:
        return dict(data)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="AGR - Gazebo Image Capture and Save Results"
    )

    # Add the arguments
    parser.add_argument(
        "-i",
        "--input_dir",
        type=str,
        default="/home/davinasanghera/ros/noetic/system/src/test_franka_simulation_bringup/worlds/simulations/123",
        required=False,
        help="The input file",
    )

    args = parser.parse_args()
    rankings = [
        135,
        143,
        137,
        159,
        119,
        109,
        132,
        151,
        157,
        149,
        113,
        115,
        125,
        117,
        126,
        165,
        134,
        164,
        142,
        153,
        140,
        150,
        158,
        57,
        131,
        168,
        174,
        55,
        50,
        51,
        186,
        23,
        29,
        28,
        27,
        26,
        25,
        24,
        22,
        21,
        20,
        61,
        196,
        56,
        1,
        0,
        8,
        7,
        9,
        6,
        2,
        3,
        4,
        5,
        34,
        30,
        31,
        32,
        33,
        35,
        36,
        37,
        38,
        39,
        172,
        139,
        148,
        167,
        76,
        62,
        99,
        180,
        122,
        171,
        198,
        77,
        65,
        161,
        155,
        147,
        108,
        101,
        130,
        107,
        96,
        191,
        170,
        188,
        121,
        181,
        156,
        160,
        177,
        176,
        178,
        114,
        80,
        127,
        74,
        71,
        86,
        144,
        100,
        103,
        75,
        84,
        102,
        197,
        106,
        70,
        64,
        98,
        73,
        199,
        89,
        82,
        63,
        182,
        78,
        175,
        154,
        184,
        69,
        187,
        185,
        193,
        189,
        66,
        72,
        133,
        97,
        194,
        105,
        141,
        192,
        110,
        81,
        116,
        162,
        104,
        54,
        195,
        120,
        85,
        53,
        90,
        190,
        67,
        10,
        13,
        14,
        15,
        16,
        17,
        18,
        19,
        12,
        11,
        52,
        58,
        92,
        129,
        152,
        83,
        173,
        169,
        60,
        49,
        45,
        44,
        43,
        42,
        41,
        47,
        40,
        46,
        48,
        87,
        138,
        183,
        128,
        95,
        79,
        68,
        94,
        179,
        59,
        111,
        118,
        163,
        88,
        93,
        91,
        136,
        124,
        112,
        146,
        123,
        166,
        145,
    ]
    data = get_data(args.input_dir + "/predictions_scene.npz")
    results = read_floats_from_file(args.input_dir + "/agr_output_single.txt")
    data_org = get_data(args.input_dir + "/predictions_scene.npz")
    pred_grasps_cam = data["pred_grasps_cam"].item()
    # Example poses data (assuming it's a list of some sort)
# Example rankings list (as you provided)
# Assuming 'pred_grasps_cam' is your list of transformation matrices
# pred_grasps_cam = data_org["pred_grasps_cam"].item()

visualize_grasps(
    full_pc=data["pc_full"],
    pred_grasps_cam=pred_grasps_cam,  # sorted_matrices as sorted in the previous message
    scores=data_org["scores"].item(),
    plot_opencv_cam=True,
    pc_colors=data["pc_colors"],  # Using the generated color scale
    adaptive_goal_region_data=results,
    single=True,
    rankings=rankings,
)

# pc_colors=data["pc_colors"],
