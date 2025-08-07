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
import random
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

np.set_printoptions(precision=4)
np.set_printoptions(suppress=True)

planning_frames = noa_debug_info_pb2.NOADebugInfo()
planning_debug_data = {}



def plot_speed_finder_data(speed_finder_debug):
    # 创建包含三个子图的画布
    fig, axes = plt.subplots(3, 1, figsize=(10, 12))
    axes[0].set_title('ST Diagram')
    axes[1].set_title('VT Diagram')
    axes[2].set_title('AT Diagram')
    
    # 为每个图表类型定义配置
    config = {
        'st': {'ax': axes[0], 'ylabel': 'S'},
        'vt': {'ax': axes[1], 'ylabel': 'V'},
        'at': {'ax': axes[2], 'ylabel': 'A'}
    }

    for graph_type in ['st', 'vt', 'at']:
        ax = config[graph_type]['ax']
        ax.set_xlabel('Time')
        ax.set_ylabel(config[graph_type]['ylabel'])
        
        # 获取对应图表的数据
        for element in speed_finder_debug.get(graph_type, []):
            if element.get('is_closed', False):
                continue
            
            # 解析颜色设置
            color_info = element.get('color', {})
            r = color_info.get('R', 0.0) / 255.0
            g = color_info.get('G', 0.0) / 255.0
            b = color_info.get('B', 0.0) / 255.0
            alpha = color_info.get('alpha', 1.0)
            color = (r, g, b, alpha)
            
            # 提取坐标点
            points = element.get('points', [])
            x = [p[0] for p in points]
            y = [p[1] for p in points]
            
            # 根据类型绘制图形
            if element.get('type') == 'POLYGON':
                polygon = Polygon(list(zip(x, y)), closed=True, 
                                fill=True, color=color)
                ax.add_patch(pgon)
                # 自动调整坐标轴范围
                ax.autoscale_view()
            elif element.get('type') == 'LINE':
                ax.plot(x, y, color=color)

    plt.tight_layout()
    plt.show()

def process_pack_planning(data_des, planning_frames):
    planning_frames.ParseFromString(data_des.proto[0].meta)

def single_pack_to_dataset() -> None:
    with open("/home/not0296/test/planning_debug_tool/metrics/dest2.pack", "rb") as f:
        reader = Reader(f)
        planning_frame_idx = 0
        for frame in reader:
            for data_des in frame.meta.data_list.data_descriptor:
                if data_des.message_name == "/apps/plan_control/plan_result_data_debug":
                    process_pack_planning(data_des, planning_frames)
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

single_pack_to_dataset()

def color_complete(color):
    if not color.R: color.R = 0
    else: 
            color.R=color.R/255
    if not color.G: color.G = 0
    else: 
            color.G=abs(color.G/255-random.random())
    if not color.B: color.B = 0
    else: 
            color.B=random.random()
    if not color.alpha: color.alpha = 1.0
    return (color.R,color.G,color.B,color.alpha)


def point_complete(point):
    if not point.x: point.x = 0
    if not point.y: point.y = 0



# for planning_frame_idx in range(len(planning_debug_data)):
#     print(str(planning_frame_idx))
#     plot_obj=[][]
#     plot_frame.append(plot_obj)
#     for key, value in planning_debug_data[str(planning_frame_idx)]["planning_debug_info_map"]:
#         # print("key:" + str(key))
#         # print(value.speed_finder_debug)
#         xt=[]
#         yt=[]
#         for st_graph in value.speed_finder_debug.pre_decision.st_boundaries: 
#                 color_complete(st_graph.color)
#                 print()
#                 print(st_graph.id)
#                 print(st_graph.type)
#                 print(st_graph.is_closed)
#                 print(st_graph.color)
#                 for point in st_graph.points:
#                     point_complete(point)
#                     xt.append(point.x)                      
#                     yt.append(point.y)      
plot_frames = []

for planning_frame_idx in range(len(planning_debug_data)):
    frame_obstacles = [] 
    for key, value in planning_debug_data[str(planning_frame_idx)]["planning_debug_info_map"]:
        for st_graph in value.speed_finder_debug.pre_decision.st_boundaries:
            obstacle = {
                'id': st_graph.id,
                'type': st_graph.type,
                'color': st_graph.color,
                'x': [],
                'y': []
            }
            for point in st_graph.points:
                point_complete(point)
                obstacle['x'].append(point.x)
                obstacle['y'].append(point.y)
            obstacle['x'].append(obstacle['x'][0])
            obstacle['y'].append(obstacle['y'][0])
            frame_obstacles.append(obstacle)
    plot_frames.append(frame_obstacles)


# plot_frames[frame_num][obj_num]['']


# # 绘制多边形
# plt.figure(figsize=(8, 6))
# plt.plot(plot_frames[0][0]['x'], plot_frames[0][0]['y'], 'r-', linewidth=2, label='Obstacle')  # 红色连线
# plt.xlabel('X')
# plt.ylabel('Y')
# plt.title('First Obstacle Polygon')
# plt.grid(True)
# plt.legend()
# plt.show()


# # 绘制一帧的多边形
# plt.figure(figsize=(8, 6))
# for obj_num in range(len(plot_frames[0])):
#     plt.plot(plot_frames[0][obj_num]['x'], plot_frames[0][obj_num]['y'], 'r-', linewidth=2, label='Obstacle')  # 红色连线
#     plt.xlabel('X')
#     plt.ylabel('Y')
#     plt.title('First Obstacle Polygon')
#     plt.grid(True)
# plt.show()

# # 绘制一帧的多边形
# plt.figure(figsize=(8, 6))
# frame_num = 0
# for obj_num in range(len(plot_frames[frame_num])):
#     plt.plot(plot_frames[frame_num][obj_num]['x'], plot_frames[frame_num][obj_num]['y'], color=color_complete(
# plot_frames[frame_num][obj_num]['color']), linewidth=2, label='Obstacle')  # 红色连线
#     plt.xlabel('X')
#     plt.ylabel('Y')
#     plt.title('First Obstacle Polygon')
#     plt.grid(True)
# plt.show()


# 绘制每帧的多边形
plt.figure(figsize=(8, 6))
for frame_num in range(len(plot_frames)):
    for obj_num in range(len(plot_frames[frame_num])):
        plt.plot(plot_frames[frame_num][obj_num]['x'], plot_frames[frame_num][obj_num]['y'], color=color_complete(
    plot_frames[frame_num][obj_num]['color']), linewidth=2, label='Obstacle')  # 红色连线
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title(str(frame_num))
        plt.grid(True)
    plt.show()