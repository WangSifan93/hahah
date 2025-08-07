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
from proto import fusion_od_pb2
from proto import localization_pb2
from proto import noa_debug_info_pb2
from datetime import datetime
import threading
from packreader.reader import Reader
from bokeh.io import curdoc
from bokeh.layouts import column, row
from bokeh.models import ColumnDataSource, Slider, CustomJS, HoverTool
from bokeh.plotting import figure, show
from concurrent.futures import ThreadPoolExecutor

np.set_printoptions(precision=4)
np.set_printoptions(suppress=True)



def point_complete(point):
    if not point.x: point.x = 0
    if not point.y: point.y = 0

def color_complete(color, mark):
    if not color.R: color.R = 0
    else: 
            color.R = 0 
    if not color.G: color.G = 0
    else: 
            color.G = abs(color.G)/255
    if not color.B: color.B = 0
    else: 
            color.B = abs(color.B)/255
    if not color.alpha: color.alpha = 1.0
    return (color.R,color.G,color.B,color.alpha)

def date_reshape_for_plot(planning_debug_data, name1, name2, name3):
    plot_frames=[]
    for planning_frame_idx in range(len(planning_debug_data)):
        frame_obstacles = [] 
        for key, value in planning_debug_data[str(planning_frame_idx)]["planning_debug_info_map"]:
            graphs = getattr(value, name1, [])
            graphs = getattr(graphs, name2, [])
            graphs = getattr(graphs, name3, [])
            try:
                for graph in graphs:
                    obstacle = {
                        'id': graph.id,
                        'type': graph.type,
                        'color': graph.color,
                        'is_closed': graph.is_closed,
                        'x': [],
                        'y': []
                    }
                    for point in graph.points:
                        point_complete(point)
                        obstacle['x'].append(point.x)
                        obstacle['y'].append(point.y)
                    obstacle['x'].append(obstacle['x'][0])
                    obstacle['y'].append(obstacle['y'][0])
                    frame_obstacles.append(obstacle)            
            except Exception as e:
                obstacle = {
                    'id': graphs.id,
                    'type': graphs.type,
                    'color': graphs.color,
                    'is_closed': graphs.is_closed,
                    'x': [],
                    'y': []
                }
                for point in graphs.points:
                    point_complete(point)
                    obstacle['x'].append(point.x)
                    obstacle['y'].append(point.y)
                # obstacle['x'].append(obstacle['x'][0])
                # obstacle['y'].append(obstacle['y'][0])
                frame_obstacles.append(obstacle)
        plot_frames.append(frame_obstacles)    
    frames_data = {}
    for frame_num in range(len(plot_frames)):
        frame_objs = {}
        for obj_num in range(len(plot_frames[frame_num])):
            frame_objs[str(obj_num)] = {
                "xs": [plot_frames[frame_num][obj_num]['x']], 
                "ys": [plot_frames[frame_num][obj_num]['y']],
                "id": [plot_frames[frame_num][obj_num]['id']],
                "type": [plot_frames[frame_num][obj_num]['type']],
                "color": [color_complete(plot_frames[frame_num][obj_num]['color'], obj_num)],
                "is_closed": [plot_frames[frame_num][obj_num]['is_closed']],
            }
        frames_data[str(frame_num)] = frame_objs
    return frames_data


def plot_graph_with_JS(source_data, graph_name):
    frames_data=source_data

    # Set initial frame and time tick.
    init_frame = "0"
    init_obj = "0" 
    
    # Set source frames data
    print(frames_data[init_frame][init_obj]['id'])
    source = ColumnDataSource(data=dict(
        xs=frames_data[init_frame][init_obj]['xs'],
        ys=frames_data[init_frame][init_obj]['ys'],
        id=frames_data[init_frame][init_obj]['id'],
        fill_color=frames_data[init_frame][init_obj]['color']
    ))

    p = figure(title=graph_name, tools="hover",
           tooltips=[("ID", "@id")], x_range=(0, 8),
            width=800, height=600)

    p.patches('xs', 'ys', source=source, fill_alpha=0.3, line_color="black")

    hover = p.select(dict(type=HoverTool))
    hover.tooltips = [("ID", "@id")]

    # Create the frame slider.
    frame_slider = Slider(start=0, end=len(frames_data)-1, value=0, step=1, title="Frame")

    # For the time tick slider:
    # Use -1 to represent "all" and 0..max_tick for each frame.
    # When obj_slider.value == -1, all st_graph in that frame are combined.
    # When obj_slider.value == id, top 10 id  in that frame are combined.
    obj_slider = Slider(start=-1, end=max(len(objs)-1 for objs in frames_data.values()), value=0, step=1, title="object (-1 means all)")

    # Create a CustomJS callback that updates theobj_slider polygon data.
    # When the frame slider changes, it recomputes the available time ticks (and sets the slider end).
    # When time_slider.value == -1, all ticks in that frame are combined.
    callback = CustomJS(args=dict(source=source,
                                frame_slider=frame_slider,
                                obj_slider=obj_slider,
                                frames_data=frames_data),
    code="""
        // Convert the frame value to a string to index our data.
        var frame = frame_slider.value.toString();
        var tickKeys = Object.keys(frames_data[frame]).map(Number);
        tickKeys.sort(function(a, b){ return a - b; });
        var maxTick = tickKeys[tickKeys.length - 1];

        // Update the obj_slider range. Always start at -1 for the "all" option.
        obj_slider.start = -1;
        obj_slider.end = maxTick;

        // If the current time tick slider value is out of bounds, reset it to -1.
        if (obj_slider.value < -1 || obj_slider.value > maxTick) {
            obj_slider.value = -1;
        }

        var new_data;
        if (obj_slider.value == -1) {
            // Combine polygon data from all time ticks for this frame.
            new_data = {xs: [], ys: []};
            for (var i = 0; i < tickKeys.length; i++) {
                var tick = tickKeys[i].toString();
                var tick_data = frames_data[frame][tick];
                new_data.xs = new_data.xs.concat(tick_data.xs);
                new_data.ys = new_data.ys.concat(tick_data.ys);
            }
        } else {
            // Show only the polygon data for the selected time tick.
            new_data = frames_data[frame][obj_slider.value.toString()];
        }

        // Update the data source and trigger a change.
        source.data = new_data;
        source.change.emit();
    """)

    # Attach the callback to both sliders.
    frame_slider.js_on_change('value', callback)
    obj_slider.js_on_change('value', callback)

    # Arrange the layout and add it to the current document.
    layout = column(p, row(frame_slider, obj_slider))
    show(layout)
    return 


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
    
    frames_st_boundaries = date_reshape_for_plot(planning_debug_data, 'speed_finder_debug', 'pre_decision','st_boundaries')
    # frames_raw_st_boundaries = date_reshape_for_plot(planning_debug_data, 'speed_finder_debug', 'pre_decision','raw_st_boundaries')
    # frames_st_softbound = date_reshape_for_plot(planning_debug_data, 'speed_finder_debug', 'pre_decision','st_softbound')
    # frames_speed_limit = date_reshape_for_plot(planning_debug_data, 'speed_finder_debug', 'pre_decision','speed_limit')
    # frames_end_of_path = date_reshape_for_plot(planning_debug_data, 'speed_finder_debug', 'pre_decision','end_of_path')


    # frames_preliminary_speed_st = date_reshape_for_plot(planning_debug_data, 'speed_finder_debug', 'interactive_speed_decision','preliminary_speed_st')
    # frames_preliminary_speed_vt = date_reshape_for_plot(planning_debug_data, 'speed_finder_debug', 'interactive_speed_decision','preliminary_speed_vt')
    # frames_preliminary_speed_at = date_reshape_for_plot(planning_debug_data, 'speed_finder_debug', 'interactive_speed_decision','preliminary_speed_at')
    # frames_preliminary_speed_jt = date_reshape_for_plot(planning_debug_data, 'speed_finder_debug', 'interactive_speed_decision','preliminary_speed_jt')

    # frames_soft_s_upper_bound = date_reshape_for_plot(planning_debug_data, 'speed_finder_debug', 'optimization','soft_s_upper_bound')
    # frames_optimized_speed_st = date_reshape_for_plot(planning_debug_data, 'speed_finder_debug', 'optimization','optimized_speed_st')
    # frames_optimized_speed_vt = date_reshape_for_plot(planning_debug_data, 'speed_finder_debug', 'optimization','optimized_speed_vt')
    # frames_optimized_speed_at = date_reshape_for_plot(planning_debug_data, 'speed_finder_debug', 'optimization','optimized_speed_at')
    # frames_optimized_speed_jt = date_reshape_for_plot(planning_debug_data, 'speed_finder_debug', 'optimization','optimized_speed_jt')
    # frames_comfortable_brake_speed = date_reshape_for_plot(planning_debug_data, 'speed_finder_debug', 'optimization','comfortable_brake_speed')
    # frames_max_brake_speed = date_reshape_for_plot(planning_debug_data, 'speed_finder_debug', 'optimization','max_brake_speed')
    # frames_ref_speed_vt = date_reshape_for_plot(planning_debug_data, 'speed_finder_debug', 'optimization','ref_speed_vt')

    
    plot_graph_with_JS(frames_st_boundaries, "st") 