import os, sys
sys.path.append(os.path.dirname(os.getcwd()))
sys.path.append(os.path.dirname(__file__) + "/proto")
sys.path.append(os.path.dirname(__file__) + "/utils")

import argparse
import numpy as np
import pandas as pd
from proto import fusion_od_pb2
from proto import localization_pb2
from proto import noa_debug_info_pb2
from datetime import datetime
import threading
from packreader.reader import Reader
from concurrent.futures import ThreadPoolExecutor

def single_pack_to_dataset(single_pack, planning_debug_data, planning_frames, args) -> None:
    scenario_prefix, ext = os.path.splitext(os.path.basename(single_pack))
    with open(single_pack, "rb") as f:
        reader = Reader(f)
        planning_frame_idx = 0
        for frame in reader:
            for data_des in frame.meta.data_list.data_descriptor:
                if data_des.message_name == "/apps/plan_control/plan_result_data_debug":
                    planning_frames.ParseFromString(data_des.proto[0].meta)
                    print("processing：" + str(planning_frame_idx))
                    planning_debug_data[str(planning_frame_idx)] = {
                        # "mission_debug_info": planning_frames.mission_debug_info.items(),
                        # "vehicle_debug_info": planning_frames.vehicle_debug_info_map.items(),
                        "planning_debug_info_map": planning_frames.planning_debug_info_map.items(),
                        # "final_trajs": planning_frames.final_trajs.items(),
                        # "choose_lane_debug_info": planning_frames.choose_lane_debug_info.items()
                    }
                    planning_frame_idx += 1        
        print("total frame:" + str(planning_frame_idx))    

def find_pack_files(directory):
    pack_path = []
    for root, dirs, files in os.walk(directory):
        for file in files:
            if file.endswith(".pack"):
                if "metrics" in os.path.join(root, file):
                    pack_path.append(os.path.join(root, file))
    print("all pack num is: " + str(len(pack_path)))
    return pack_path

def parse_config():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--input_path",
        type=str,
        default="/home/not0296/test",
        help="input path",
        required=False,
    )
    parser.add_argument(
        "--results_path",
        type=str,
        default="/home/not0296/test",
        required=False,
        help="the path of result",
    )
    parser.add_argument(
        "--results_category_path",
        type=str,
        default="/home/not0296/test",
        required=False,
        help="the path of result",
    )
    parser.add_argument(
        "--is_map_range",
        type=bool,
        default=True,
        required=False,
        help="whether to use map range",
    )
    parser.add_argument(
        "--map_dis_thr",
        type=int,
        default=300,
        required=False,
        help="map point distance of ego",
    )
    parser.add_argument(
        "--interval",
        type=int,
        default=10,
        required=False,
        help="interval of map frames",
    )
    parser.add_argument(
        "--num_threads",
        type=int,
        default=5,
        required=False,
        help="num of threads to multi process",
    )
    parser.add_argument(
        "--interp_arc",
        type=bool,
        default=True,
        required=False,
        help="whether to interp arc",
    )
    args = parser.parse_args()
    return args


if __name__ == "__main__":
    args = parse_config()
    planning_debug_data = {}
    planning_frames = noa_debug_info_pb2.NOADebugInfo()
    pack_files = find_pack_files(args.input_path)
    num_threads = args.num_threads
    with ThreadPoolExecutor(max_workers=num_threads) as executor:
        futures = {
            executor.submit(single_pack_to_dataset, file, planning_debug_data, planning_frames, args): file
            for file in pack_files
        }