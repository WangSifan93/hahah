import os, sys

sys.path.append(os.path.dirname(os.getcwd()))
sys.path.append(os.path.dirname(__file__) + "/proto")
sys.path.append(os.path.dirname(__file__) + "/utils")
import glob
import json
import argparse
import numpy as np
import pandas as pd
# import matplotlib.pyplot as plt
import copy
import math
import shutil
from mcts.proto import noa_debug_info_pb2
from datetime import datetime
import threading
# from utils import extraction_util
from mcts.packreader.reader import Reader

def process_mcts_debug(planning_debug_map, output_planning_debug_info):
    for key, value in planning_debug_map.items():
        if 'Selected' in key:
            for key_string, value_string in value.strings.items():
                if 'MCTS' in key_string:
                    output_planning_debug_info["mcts"] = value_string
                    output_planning_debug_info["mcts_json_name"] = str(key) + ".json"
                    return
            return

def process_pack_planning(data_des, planning_frames, output_mission_debug_info,
                        output_vehicle_debug_info, output_planning_debug_info, output_final_trajs,
                        output_choose_lane_debug_info):
    planning_frames.ParseFromString(data_des.proto[0].meta)
    process_mcts_debug(planning_frames.planning_debug_info_map, output_planning_debug_info)

def single_pack_to_dataset(single_pack) -> None:
    scenario_prefix, ext = os.path.splitext(os.path.basename(single_pack))
    planning_frames = noa_debug_info_pb2.NOADebugInfo()
    
    planning_debug_data = {}
    with open(single_pack, "rb") as f:
        reader = Reader(f)
        planning_frame_idx = 0
        for frame in reader:
            for data_des in frame.meta.data_list.data_descriptor:
                if data_des.message_name == "/apps/plan_control/plan_result_data_debug":
                    output_mission_debug_info = {}
                    output_vehicle_debug_info = {}
                    output_planning_debug_info_map = {}
                    output_final_trajs = {}
                    output_choose_lane_debug_info = {}
                    process_pack_planning(
                        data_des, planning_frames, output_mission_debug_info, output_vehicle_debug_info,
                        output_planning_debug_info_map, output_final_trajs, output_choose_lane_debug_info
                    )
                    planning_debug_data[str(planning_frame_idx)] = {
                        "mission_debug_info": output_mission_debug_info,
                        "vehicle_debug_info": output_vehicle_debug_info,
                        "planning_debug_info_map": output_planning_debug_info_map,
                        "final_trajs": output_final_trajs,
                        "choose_lane_debug_info": output_choose_lane_debug_info
                    }
                    planning_frame_idx += 1

        # 获取当前年月日
        today = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')

        # 获取当前脚本所在目录的上一级目录
        script_dir = os.path.dirname(os.path.abspath(__file__))
        parent_dir = os.path.dirname(script_dir) 
        # print(parent_dir)
        # 拼出存放 json 文件的目标目录
        output_dir = os.path.join(parent_dir, scenario_prefix + today)
        os.makedirs(output_dir, exist_ok=True)  # 如果不存在就新建
        for key, value in planning_debug_data.items():
            if "mcts" in value["planning_debug_info_map"]:
                data = json.loads(value["planning_debug_info_map"]["mcts"])
                filename = value["planning_debug_info_map"]["mcts_json_name"]
                json_path = os.path.join(output_dir, filename)
                with open(json_path, "w", encoding="utf-8") as f:
                    json.dump(data, f, ensure_ascii=False, indent=4)
    return output_dir    

def find_pack_files(directory):
    pack_path = []
    for root, dirs, files in os.walk(directory):
        for file in files:
            if file.endswith(".pack"):
                pack_path.append(os.path.join(root, file))
    print("all pack num is: " + str(len(pack_path)))
    return pack_path

def pack_extraction_to_json(input_path="/home/sifan/A_DZ/planning_debug_tool"):
    from concurrent.futures import ThreadPoolExecutor, as_completed
    pack_files = find_pack_files(input_path)
    num_threads = 5
    results = []
    with ThreadPoolExecutor(max_workers=num_threads) as executor:
        futures = {
            executor.submit(single_pack_to_dataset, file): file
            for file in pack_files
        }

        for future in as_completed(futures):
            file = futures[future]
            try:
                result = future.result()  # 获取 single_pack_to_dataset 的返回值
                results.append(result)  # 你可以根据需要只 append result
            except Exception as e:
                print(f"❌ 处理文件 {file} 时出错: {e}")

    return results

if __name__ == "__main__":
    pack_extraction_to_json()
