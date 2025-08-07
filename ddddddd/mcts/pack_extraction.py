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
# from tqdm import tqdm
# from proto import all_map_new_pb2
from proto import fusion_od_pb2
from proto import localization_pb2
from proto import noa_debug_info_pb2
#from proto import learning_scenario_pb2
#from proto import learning_map_pb2
from datetime import datetime
import threading
# from utils import extraction_util
from packreader.reader import Reader
# from av2.geometry.interpolate import interp_arc
from bokeh.io import curdoc
from bokeh.layouts import column, row
from bokeh.models import ColumnDataSource, Slider, CustomJS
from bokeh.plotting import figure, show


np.set_printoptions(precision=4)
np.set_printoptions(suppress=True)


def build_and_save_dataset(agents_df, maps_dic, dir, args):
    time_steps = []
    for time_step in agents_df["timestep"]:
        if time_step not in time_steps:
            time_steps.append(time_step)
    for i in range(0, len(time_steps), int(args.interval)):
        step = time_steps[i]
        step_start = step - 50  # history
        step_end = step + 59  # prediction

        agent_entries_clip = agents_df[
            (agents_df["timestep"] >= step_start) & (agents_df["timestep"] <= step_end)
        ].copy()
        if len(agent_entries_clip[(agent_entries_clip["track_id"] == "AV")]) < 105:
            continue
        ts_idx = agent_entries_clip[
            (agent_entries_clip["track_id"] == "AV")
            & (agent_entries_clip["timestep"] == step)
        ].index
        if len(agent_entries_clip.loc[ts_idx, "timestamp"].tolist()) > 0:
            prediction_ts = agent_entries_clip.loc[ts_idx, "timestamp"].tolist()[0]
            if prediction_ts in maps_dic:
                flag_found_agent = False
                for agent_id, _ in agent_entries_clip.groupby("track_id"):
                    if (
                        agent_id != "AV"
                        and len(
                            agent_entries_clip[(agent_entries_clip["track_id"] == agent_id)]
                        )
                        >= 100
                    ):
                        object_category_3_idx = agent_entries_clip[
                            (agent_entries_clip["track_id"] == agent_id)
                        ].index
                        agent_entries_clip.loc[object_category_3_idx, "object_category"] = 3
                        flag_found_agent = True
                # if flag_found_agent:
                # reset timestep to 0~109
                agent_entries_clip["timestep"] = agent_entries_clip["timestep"] - min(
                    agent_entries_clip["timestep"]
                )
                # agent_entries_clip.loc[:,'focal_track_id'] = int(focal_track_id)
                agent_entries_clip = agent_entries_clip.reset_index(drop=True)
                endwith = ""
                if flag_found_agent:
                    endwith = "_object_category_3"
                clip = str(prediction_ts) + endwith
                map_dic = maps_dic[prediction_ts]
                if endwith != "_object_category_3":
                    save_dir = os.path.join(str(args.results_path), clip)
                else:
                    save_dir = os.path.join(str(args.results_category_path), clip)
                os.makedirs(save_dir, exist_ok=True)
                agents_path = os.path.join(save_dir, f"scenario_{clip}.parquet")
                lines_path = os.path.join(save_dir, f"log_map_archive_{clip}.json")
                print(f"saving dataset to {save_dir}")
                agent_entries_clip.to_parquet(agents_path)
                with open(lines_path, "w") as json_file:
                    json.dump(map_dic, json_file)
                # extraction_util.save_vis(agent_entries_clip, maps_list, save_dir, args)


def l2_dis(point1, point2):
    return math.sqrt(sum([(p1 - p2) ** 2 for p1, p2 in zip(point1, point2)]))


def process_every_lane_points(points, ego_center, args):
    valid_line_points = []
    start_dis = l2_dis([points[0].x, points[0].y], ego_center)
    end_dis = l2_dis([points[-1].x, points[-1].y], ego_center)
    if args.is_map_range and (start_dis > args.map_dis_thr and end_dis > args.map_dis_thr):
        return []
    for idx, point in enumerate(points):
        x = point.x
        y = point.y
        z = 0
        valid_line_points.append({"x": x, "y": y, "z": z})
    return valid_line_points


def process_every_lane_boundary(map_frame, lane, ego_center, args):
    lane_boundary_dic = {}
    left_boundary_id = lane.left_boundary_id
    right_boundary_id = lane.right_boundary_id
    left_lane_mark_type, right_lane_mark_type = "UNKNOWN", "UNKNOWN"
    flag_find_left_boundary = False
    flag_find_right_boundary = False
    for lane_boundary in map_frame.map.hdmap.lane_boundary:
        if not flag_find_left_boundary and lane_boundary.id == left_boundary_id:
            left_lane_boundary = process_every_lane_points(
                lane_boundary.curve.point, ego_center, args
            )
            if len(left_lane_boundary) > 1 and len(lane_boundary.boundary_attr):
                left_lane_mark_type = (
                    extraction_util.transform_lane_boundary_type_with_color(
                        lane_boundary.boundary_attr[0].type,
                        lane_boundary.boundary_attr[0].color,
                    )
                )
                flag_find_left_boundary = True
        if not flag_find_right_boundary and lane_boundary.id == right_boundary_id:
            right_lane_boundary = process_every_lane_points(
                lane_boundary.curve.point, ego_center, args
            )
            if len(right_lane_boundary) > 1 and len(lane_boundary.boundary_attr):
                right_lane_mark_type = (
                    extraction_util.transform_lane_boundary_type_with_color(
                        lane_boundary.boundary_attr[0].type,
                        lane_boundary.boundary_attr[0].color,
                    )
                )
                flag_find_right_boundary = True
        if flag_find_left_boundary and flag_find_right_boundary:
            break
    if not flag_find_left_boundary:
        left_lane_boundary = []
    if not flag_find_right_boundary:
        right_lane_boundary = []
    lane_boundary_dic["left_lane_boundary"] = left_lane_boundary
    lane_boundary_dic["left_lane_mark_type"] = left_lane_mark_type
    lane_boundary_dic["right_lane_boundary"] = right_lane_boundary
    lane_boundary_dic["right_lane_mark_type"] = right_lane_mark_type
    return lane_boundary_dic


def points_interp_arc(points, lane_type):
    lane_type_dic = {"center_lane": 17, "boundary": 18}
    points_array = np.array([(point["x"], point["y"], point["z"]) for point in points])
    interp_arc_points = interp_arc(lane_type_dic[lane_type], points_array)
    interp_points = [{"x": x, "y": y, "z": z} for x, y, z in interp_arc_points]
    return interp_points


def every_lane_postprocess(line, line_type, args):
    lane_type_dic = {"center_lane": 51, "boundary": 51}
    line_list = []
    line_length = len(line)
    if line_length == 0:
        return line_list
    line_num = lane_type_dic[line_type]
    if line_length < line_num and args.interp_arc:
        # line = points_interp_arc(line, line_type)
        line_list.append(line)
    else:
        for i in range(1, line_length // line_num + 1):
            line_list.append(line[(i - 1) * line_num : i * line_num])
        if line_length % line_num > 1:
            # last_split_line = points_interp_arc(
            #     line[(line_length // line_num) * line_num :], line_type
            # )
            last_split_line = line[(line_length // line_num) * line_num :]
            line_list.append(last_split_line)
    return line_list


def process_pack_map_lane(map_frame, ego_center, args):
    lane_info, road_ids = process_pack_map_lane_centerline(map_frame, ego_center, args)
    lane_boundary_info = process_pack_map_lane_boundary(map_frame, ego_center, args)
    lane_info.update(lane_boundary_info)
    return lane_info, road_ids


def process_pack_map_lane_centerline(map_frame, ego_center, args):
    lane_info = {}
    road_ids = []
    for lane in map_frame.map.hdmap.lane:
        # center_line
        center_line = process_every_lane_points(
            lane.center_line.point, ego_center, args
        )
        # need more than two points
        if len(center_line) > 1:
            id = lane.id.id
            road_ids.append(lane.road_id.id)
            is_intersection = lane.in_junction
            # to transform
            lane_type = extraction_util.transform_lane_type(lane.type)

            left_neighbor_id, right_neighbor_id = None, None
            if len(lane.left_neighbor_forward_lane_id):
                left_neighbor_id = lane.left_neighbor_forward_lane_id[0].id
            if len(lane.right_neighbor_forward_lane_id):
                right_neighbor_id = lane.right_neighbor_forward_lane_id[0].id

            predecessor_id, successor_id = [], []
            for lane_predecessor_id in lane.predecessor_id:
                predecessor_id.append(lane_predecessor_id.id)
            for lane_successor_id in lane.successor_id:
                successor_id.append(lane_successor_id.id)

            center_line_list = every_lane_postprocess(center_line, "center_lane", args)
            for i in range(len(center_line_list)):
                fix_id = f"{id}_{i}_centerline"
                lane_info[fix_id] = {
                    "centerline": center_line_list[i],
                    "id": fix_id,
                    "is_intersection": is_intersection,
                    "lane_type": lane_type,
                    "left_lane_boundary": [],
                    "left_lane_mark_type": "UNKNOWN",
                    "left_neighbor_id": left_neighbor_id,
                    "predecessors": predecessor_id,
                    "right_lane_boundary": [],
                    "right_lane_mark_type": "UNKNOWN",
                    "right_neighbor_id": right_neighbor_id,
                    "successors": successor_id,
                }
    return lane_info, road_ids


def process_pack_map_lane_boundary(map_frame, ego_center, args):
    lane_info = {}
    lane_mark_type = "UNKNOWN"
    for boundary in map_frame.map.hdmap.lane_boundary:
        lane_boundary = process_every_lane_points(
            boundary.curve.point, ego_center, args
        )
        if len(lane_boundary) > 1 and len(boundary.boundary_attr):
            lane_mark_type = (
                extraction_util.transform_lane_boundary_type_with_color(
                    boundary.boundary_attr[0].type,
                    boundary.boundary_attr[0].color,
                )
            )
            id = boundary.id.id
            is_intersection = False
            lane_type = "UNKNOWN"

            left_neighbor_id, right_neighbor_id = None, None
            predecessor_id, successor_id = [], []

            boundary_line_list = every_lane_postprocess(lane_boundary, "boundary", args)
            for i in range(len(boundary_line_list)):
                fix_id = f"{id}_{i}_boundary"
                lane_info[fix_id] = {
                    "centerline": [],
                    "id": fix_id,
                    "is_intersection": is_intersection,
                    "lane_type": lane_type,
                    "left_lane_boundary": lane_boundary,
                    "left_lane_mark_type": lane_mark_type,
                    "left_neighbor_id": left_neighbor_id,
                    "predecessors": predecessor_id,
                    "right_lane_boundary": [],
                    "right_lane_mark_type": "UNKNOWN",
                    "right_neighbor_id": right_neighbor_id,
                    "successors": successor_id,
                }
    return lane_info


def process_pack_map_boundary(map_frame, ego_center, road_ids, args):
    drivable_areas = {}
    visited_road_ids = set()
    for boundary in map_frame.map.hdmap.road_boundary:
        boundary_road_id = boundary.road_id.id
        if boundary_road_id in road_ids and boundary_road_id not in visited_road_ids:
            visited_road_ids.add(boundary_road_id)
            boundary_points = process_every_lane_points(
                boundary.curve.point, ego_center, args
            )
            if len(boundary_points) > 1:
                id = boundary.id.id
                drivable_areas[str(id)] = {"area_boundary": boundary_points, "id": id}
    return drivable_areas


def process_pack_map_crosswalk(map_frame, ego_center, args):
    # cross_walk is empty temporarily
    crosswalk = {}
    for junction in map_frame.map.hdmap.junction:
        # cross walk
        pass
        if junction.Type == 2:
            pass
    return crosswalk


def process_pack_map(
    map_frame,
    data_des,
    fusion_timestamps_for_map,
    ego_pos_x_all,
    ego_pos_y_all,
    pre_idx,
    args,
):
    map_result = {}
    map_dic = {}
    map_frame.ParseFromString(data_des.proto[0].meta)
    map_ts = map_frame.header.timestamp_nano * 1.0 / 1e9  # second
    fusion_idx = extraction_util.find_valid_ts_idx(
        map_ts, fusion_timestamps_for_map, pre_idx, 0.25
    )
    if fusion_idx != -1:
        ego_center = [ego_pos_x_all[fusion_idx], ego_pos_y_all[fusion_idx]]
        # lane_segments
        lane_info, road_ids = process_pack_map_lane(map_frame, ego_center, args)
        if not len(road_ids):
            return map_result
        map_dic["lane_segments"] = lane_info
        # drivable_areas
        drivable_areas = process_pack_map_boundary(
            map_frame, ego_center, road_ids, args
        )
        if not len(drivable_areas):
            return map_result
        map_dic["drivable_areas"] = drivable_areas
        # pedestrian_crossings
        crosswalk = process_pack_map_crosswalk(map_frame, ego_center, args)
        map_dic["pedestrian_crossings"] = crosswalk

        map_result["fusion_idx"] = fusion_idx
        map_result["map_dic"] = map_dic

    return map_result

def process_marker(marker):
    """处理 Protobuf 定义的 Marker 消息，生成可绘制的图形数据"""
    # 基础校验
    if not marker.HasField("type"):
        print("Marker type is required")
    if not marker.points:
        print("Marker must have at least one point")

    # 提取基础属性
    mark_id = marker.id if marker.HasField("id") else None
    mark_type = marker.type
    color = marker.color if marker.HasField("color") else None
    scale = marker.scale if marker.HasField("scale") else 1.0
    is_closed = marker.is_closed if marker.HasField("is_closed") else False
    properties = marker.properties if marker.HasField("properties") else {}

    # 提取点坐标（假设 Point 消息有 x, y 字段）
    points = [(p.x, p.y) for p in marker.points]

    # 根据类型处理
    result = {
        "id": mark_id,
        "type": mark_type,
        "color": color,
        "scale": scale,
        "properties": properties,
        "is_closed": is_closed,
        "data": None
    }

    if mark_type == marker.Type.LINE:
        print("LINE")
        # # 处理线段：连接所有点
        # if len(points) < 2:
        #     raise ValueError("LINE requires at least 2 points")
        # result["data"] = {
        #     "path": _generate_line_path(points),
        #     "is_closed": False  # 线段默认不闭合
        # }

    elif mark_type == marker.Type.POINT:
        print("POINT")
        # # 处理点：取第一个点
        # result["data"] = {
        #     "coordinates": points[0],
        #     "radius": scale * 5  # 示例缩放逻辑
        # }

    elif mark_type == marker.Type.POLYGON:
        # 处理多边形
        result["data"] = {
            "xs": [p[0] for p in points],  # Extract x-coordinates
            "ys": [p[1] for p in points],  # Extract y-coordinates
        }

    else:
        raise NotImplementedError(f"Unsupported marker type: {mark_type}")

    # 应用颜色规则（假设 color 是 RGB 对象）
    # if color:
    #     result["color"] = (color.r, color.g, color.b, color.a if color.HasField("a") else 255)

    return result

def process_speed_finder(speed_finder_debug, output_speed_finder):
    if "pre_decision" in speed_finder_debug:
        pre_decision = speed_finder_debug.pre_decision
        for st_boundary in pre_decision.st_boundaries:
            output_speed_finder["pre_decision"] = process_marker(st_boundary)
                
def process_planning_debug(planning_debug_map, output_planning_debug_info):
    other_idx = 0
    output_speed_finder = {}
    for key, value in planning_debug_map.items():
        if 'speed_finder_debug' in value:
            process_speed_finder(value.speed_finder_debug, output_speed_finder)
            break
    output_planning_debug_info["speed_finder"] = output_speed_finder


def process_pack_planning(data_des, planning_frames, output_mission_debug_info,
                        output_vehicle_debug_info, output_planning_debug_info, output_final_trajs,
                        output_choose_lane_debug_info):
    planning_frames.ParseFromString(data_des.proto[0].meta)
    process_planning_debug(planning_frames.planning_debug_info_map, output_planning_debug_info)
    
        
def process_pack_agent(data_des, fusion_frames, fusion_frame_idx):
    fusion_frames.ParseFromString(data_des.proto[0].meta)
    fusion_timestamp = fusion_frames.header.timestamp
    agent_info_df = pd.DataFrame([])
    for obj_idx in range(fusion_frames.header.obj_num):
        object = fusion_frames.objects[obj_idx]
        # agent info
        track_id, position = object.obj_id, object.odom_center  # x, y, z
        length, width, height = object.size.x, object.size.y, object.size.z
        heading, velocity = (
            object.odom_orientation_angle,
            object.odom_abs_vel,
        )  # vx, vy, vz
        # object_type = extraction_util.transform_agent_type(object.class_type)
        agent_info = {
            "track_id": str(track_id),
            "position_x": position.x,
            "position_y": position.y,
            "position_z": position.z,
            "length": length,
            "width": width,
            "height": height,
            "heading": heading,
            "velocity_x": velocity.x,
            "velocity_y": velocity.y,
            # "object_type": object_type,
            "valid": 1.0,
            "timestamp": fusion_timestamp,
            "timestep": fusion_frame_idx,
        }

        agent_info_df = pd.concat(
            [agent_info_df, pd.DataFrame([agent_info])], ignore_index=True
        )
    return agent_info_df, fusion_timestamp


def process_pack_ego(data_des, ego_frame, fusion_timestamps, pre_idx):
    ego_result = {}
    ego_frame.ParseFromString(data_des.proto[0].meta)
    ego_ts = ego_frame.local_pose.timestamp
    fusion_idx = extraction_util.find_valid_ts_idx(
        ego_ts, fusion_timestamps, pre_idx, 0.05
    )
    if fusion_idx != -1:
        ego_position = ego_frame.local_pose.pose.t
        ego_heading = ego_frame.local_pose.yaw
        ego_velocity = ego_frame.local_pose.velocity
        ego_velocity_x = ego_velocity.x * math.cos(ego_heading)
        ego_velocity_y = ego_velocity.x * math.sin(ego_heading)
        ego_info = {
            "track_id": "AV",
            "position_x": ego_position.x,
            "position_y": ego_position.y,
            "position_z": 0.0,
            "length": 5.0,
            "width": 2.0,
            "height": 1.8,
            "heading": ego_heading,
            "velocity_x": ego_velocity_x,
            "velocity_y": ego_velocity_y,
            "object_type": "vehicle",
            "valid": 1.0,
            "timestamp": fusion_timestamps[fusion_idx],
            "timestep": fusion_idx,
        }
        ego_info_df = pd.DataFrame([ego_info])
        ego_result["ego_info_df"] = ego_info_df
        ego_result["ego_pos_x"] = ego_position.x
        ego_result["ego_pos_y"] = ego_position.y
        ego_result["fusion_idx"] = fusion_idx
    return ego_result


def single_pack_to_dataset(single_pack, args) -> None:
    scenario_prefix, ext = os.path.splitext(os.path.basename(single_pack))

    planning_frames = noa_debug_info_pb2.NOADebugInfo()
    # fusion_frames = fusion_od_pb2.ObjFrame()
    # map_frame = all_map_new_pb2.ZarkMap()
    # ego_frame = localization_pb2.LocalizationInfo()
    # agents_df = pd.DataFrame(
    #     columns=[
    #         "track_id",
    #         "position_x",
    #         "position_y",
    #         "position_z",
    #         "length",
    #         "width",
    #         "height",
    #         "heading",
    #         "velocity_x",
    #         "velocity_y",
    #         "object_type",
    #         "valid",
    #         "timestamp",
    #         "timestep",
    #         "num_timestamps",
    #     ]
    # )
    # maps_dic = {}

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
        init_frame = "0"
        # print(planning_debug_data[init_frame]["planning_debug_info_map"]["speed_finder"]["pre_decision"]['data'])
        source = ColumnDataSource(data=planning_debug_data[init_frame]["planning_debug_info_map"]["speed_finder"]["pre_decision"]['data'])
        
        p = figure(title="Planning", x_range=(0, 10), y_range=(0, 10),
           width=600, height=400)
        print(source.data)
        p.patches('xs', 'ys', source=source, fill_alpha=0.6, line_color="black")
        frame_slider = Slider(start=0, end=len(planning_debug_data)-1, value=0, step=1, title="Frame")
        # callback = CustomJS(args=dict(source=source,
        #                             frame_slider=frame_slider,
        #                             planning_debug_data=planning_debug_data),
        # code="""
        #     var frame = frame_slider.value.toString();
        #     var new_data = planning_debug_data[frame]["planning_debug_info_map"]["speed_finder"]["pre_decision"]['data']
        #     source.data = new_data;
        #     source.change.emit();
        # """)
        
        # # Attach the callback to both sliders.
        # frame_slider.js_on_change('value', callback)
        # Arrange the layout and add it to the current document.
        layout = column(p, row(frame_slider))
        show(layout)
        
    
        # fusion_frame_idx = 0
        # fusion_timestamps = []
        # ## process agent info every frame
        # ## 100 ms
        # for frame in reader:
        #     for data_des in frame.meta.data_list.data_descriptor:
        #         if data_des.message_name == "/apps/sensor_fusion/fusion":
        #             agent_info_df, fusion_timestamp = process_pack_agent(
        #                 data_des, fusion_frames, fusion_frame_idx
        #             )
        #             agents_df = pd.concat([agents_df, agent_info_df], ignore_index=True)
        #             fusion_timestamps.append(fusion_timestamp)
        #             fusion_frame_idx += 1
        # if not len(fusion_timestamps):
        #     return
        # ## process ego info
        # ## 10 ms -> 100ms
        # pre_idx = -1
        # fusion_timestamps_for_map = []
        # ego_pos_x_all = []
        # ego_pos_y_all = []
        # for frame in reader:
        #     for data_des in frame.meta.data_list.data_descriptor:
        #         if data_des.message_name == "/apps/localization/localization_data":
        #             ego_result = process_pack_ego(
        #                 data_des, ego_frame, fusion_timestamps, pre_idx
        #             )
        #             if len(ego_result):
        #                 agents_df = pd.concat(
        #                     [agents_df, ego_result["ego_info_df"]], ignore_index=True
        #                 )
        #                 ego_pos_x_all.append(ego_result["ego_pos_x"])
        #                 ego_pos_y_all.append(ego_result["ego_pos_y"])
        #                 pre_idx = ego_result["fusion_idx"]
        #                 fusion_timestamps_for_map.append(fusion_timestamps[pre_idx])
        # # interval
        # # fusion_timestamps_for_map = fusion_timestamps_for_map[:: args.interval]
        # if not len(fusion_timestamps_for_map):
        #     return

        # ## process hdmap message
        # ## around ego
        # ## 50ms~60ms -> 100ms
        # pre_idx = -1
        # for frame in reader:
        #     for data_des in frame.meta.data_list.data_descriptor:
        #         if data_des.message_name == "/apps/mapfusion/local_hdmap_data_new":
        #             map_result = process_pack_map(
        #                 map_frame,
        #                 data_des,
        #                 fusion_timestamps_for_map,
        #                 ego_pos_x_all,
        #                 ego_pos_y_all,
        #                 pre_idx,
        #                 args,
        #             )
        #             if len(map_result):
        #                 pre_idx = map_result["fusion_idx"]
        #                 maps_dic[
        #                     fusion_timestamps_for_map[map_result["fusion_idx"]]
        #                 ] = map_result["map_dic"]
        # if not len(maps_dic):
        #     return
        # # add scenario_id & city row
        # agents_df.loc[:, "scenario_id"] = "000"
        # if "wuhu" in single_pack.lower():
        #     agents_df.loc[:, "city"] = "wuhu"
        # else:
        #     agents_df.loc[:, "city"] = "shanghai"
        # # add object_category row as default
        # agents_df.loc[:, "object_category"] = 0

        # agents_df["num_timestamps"] = fusion_frame_idx
        # agents_df.sort_values(by=["timestep"], inplace=True)
        # agents_df = agents_df.reset_index(drop=True)

        # build_and_save_dataset(agents_df, maps_dic, scenario_prefix, args)

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
        default="/home/not0526/tools",
        help="input path",
        required=False,
    )
    parser.add_argument(
        "--results_path",
        type=str,
        default="/home/not0526/tools",
        required=False,
        help="the path of result",
    )
    parser.add_argument(
        "--results_category_path",
        type=str,
        default="/home/not0526/tools",
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
    from concurrent.futures import ThreadPoolExecutor
    args = parse_config()

    pack_files = find_pack_files(args.input_path)
    num_threads = args.num_threads
    with ThreadPoolExecutor(max_workers=num_threads) as executor:
        futures = {
            executor.submit(single_pack_to_dataset, file, args): file
            for file in pack_files
        }
