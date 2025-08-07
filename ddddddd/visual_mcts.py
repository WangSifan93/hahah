from typing import Dict, List
import sys
import numpy as np
import json
import random
from bokeh.plotting.glyph_api import GlyphAPI
from bokeh.plotting import figure, from_networkx, ColumnDataSource
from bokeh.layouts import layout,column,row
from bokeh.palettes import Sunset8
import random
from bokeh.models import (
    HoverTool,
    Circle,
    LabelSet,
    Range1d,
    Legend,
    CustomJS,
    CheckboxGroup,
)
import networkx as nx

import math
from bokeh.models import Arrow, NormalHead, VeeHead, BoxAnnotation
from bokeh.transform import cumsum

# def format_vru(vru_data):
#     """格式化VRU数据为分行字符串，保留3位小数"""
#     if not vru_data:
#         return ""
    
#     formatted = []
#     for i, vru in enumerate(vru_data):
#         formatted.append(f"VRU {i+1}:")
#         for key, value in vru.items():
#             # 对数值类型保留3位小数
#             if isinstance(value, (int, float)):
#                 value = f"{value:.3g}"
#             formatted.append(f"  {key}: {value}")
#         formatted.append("")  # 添加空行分隔
    
#     return "\n".join(formatted)

def format_vru(vru_data):
    """格式化VRU数据为9个独立的分支字符串，保留3位小数"""
    if not vru_data:
        return {f"spt_vru{i}_cost": "" for i in range(9)}
    
    formatted = {}
    for i in range(9):  # 确保处理9个分支
        if i < len(vru_data):
            vru = vru_data[i]
            lines = []
            for key, value in vru.items():
                # 对数值类型保留3位小数
                if isinstance(value, (int, float)):
                    value = f"{value:.3g}"
                lines.append(f"{key}: {value}")
            formatted[f"spt_vru{i}_cost"] = "\n".join(lines)
        else:
            formatted[f"spt_vru{i}_cost"] = ""
    
    return formatted

def build_graph(G, node, data, node_details, node_leader_state, node_follower_state, node_step_reward, node_reward, node_visits, node_lon_debug_reward, node_lon_debug_cost, node_spt_debug_reward, node_spt_debug_vru):
    if node is None or 'id' not in node:
        return
    node_id = node['id']
    G.add_node(node_id)
    node_details[node_id] = "is_leaf: "+str(data[str(node_id)]['is_leaf']) +", parent_id:"+ str(data[str(node_id)]['parent_id'])+ ", current_time:"+ str(data[str(node_id)]['current_time'])
    node_leader_state[node_id] = data[str(node_id)]["state"]["leader_status"]
    node_follower_state[node_id] = data[str(node_id)]["state"]["follower_status"]
    node_step_reward[node_id] = data[str(node_id)]["step_reward"]
    node_reward[node_id] = data[str(node_id)]["reward"]
    node_visits[node_id] = data[str(node_id)]['visits']
    if 'lon_debug' in data[str(node_id)]:
        node_lon_debug_reward[node_id] = data[str(node_id)]['lon_debug']['reward']
        node_lon_debug_cost[node_id] = {}
        if '-2.000000' in data[str(node_id)]['lon_debug']['cost']: 
            node_lon_debug_cost[node_id]['f2'] = data[str(node_id)]['lon_debug']['cost']['-2.000000']
        if '-1.000000' in data[str(node_id)]['lon_debug']['cost']:
            node_lon_debug_cost[node_id]['f1'] = data[str(node_id)]['lon_debug']['cost']['-1.000000']
        if '0.000000' in data[str(node_id)]['lon_debug']['cost']:
            node_lon_debug_cost[node_id]['0'] = data[str(node_id)]['lon_debug']['cost']['0.000000']
        if '1.000000' in data[str(node_id)]['lon_debug']['cost']:
            node_lon_debug_cost[node_id]['1'] = data[str(node_id)]['lon_debug']['cost']['1.000000']
        if '2.000000' in data[str(node_id)]['lon_debug']['cost']:
            node_lon_debug_cost[node_id]['2'] = data[str(node_id)]['lon_debug']['cost']['2.000000']
    if 'spt_debug' in data[str(node_id)]:
        node_spt_debug_reward[node_id] = data[str(node_id)]['spt_debug']['reward']
        # node_spt_debug_vru[node_id] = data[str(node_id)]['spt_debug']['vru']
        # 使用格式化函数处理VRU数据
        vru_formatted = format_vru(data[str(node_id)]['spt_debug']['vru'])
        for i in range(9):
            node_spt_debug_vru[f"spt_vru{i}_cost"][node_id] = vru_formatted[f"spt_vru{i}_cost"]
    for child in node.get('children', []):
        child_id = child['id']
        if data[str(child_id)]['visits'] == 0:
            continue
        G.add_edge(node_id, child_id)
        build_graph(G, child, data, node_details, node_leader_state, node_follower_state, node_step_reward, node_reward, node_visits,
                    node_lon_debug_reward, node_lon_debug_cost, node_spt_debug_reward, node_spt_debug_vru)

def visual_mcts_tree(data, fig_mcts_tree):
    G = nx.DiGraph()
    node_details = {}
    node_leader_state = {}
    node_follower_state = {}
    node_step_reward = {}
    node_reward = {}
    node_visits = {}
    lon_debug_reward = {}
    lon_debug_cost = {}
    spt_debug_reward = {}
    spt_debug_vru = {f"spt_vru{i}_cost": {} for i in range(9)}  # 初始化9个VRU分支的存储
    if 'tree' not in data or data['tree'] is None:
        print("No tree data found in the input.")
        return None
    build_graph(G, data['tree'], data, node_details, node_leader_state, node_follower_state, node_step_reward, node_reward, node_visits,
                lon_debug_reward, lon_debug_cost, spt_debug_reward, spt_debug_vru)
    try:
        pos = nx.drawing.nx_agraph.graphviz_layout(G, prog='dot')
    except ImportError:
        print("请安装 pygraphviz 库以使用 graphviz_layout 布局算法。")
        pos = nx.spring_layout(G)
    # 用 Bokeh 从 NetworkX 图创建图形渲染器
    graph_renderer = from_networkx(G, pos, scale=1, center=(0, 0))

    # 定义圆圈大小的最大值和最小值
    circle_size_min = 20
    circle_size_max = 50
    step_reward_max = 6.3

    # 创建数据源，用于存储节点信息
    node_info = {
                'index': list(G.nodes()),
                'details': [json.dumps(node_details[node], indent=2) for node in G.nodes()],
                'node_leader_state': [json.dumps(node_leader_state[node], indent=2) for node in G.nodes()],
                'node_follower_state': [json.dumps(node_follower_state[node], indent=2) for node in G.nodes()],
                'node_step_reward': [json.dumps(node_step_reward[node], indent=2) for node in G.nodes()],
                'node_reward': [json.dumps(node_reward[node], indent=2) for node in G.nodes()],
                'node_visits': [node_visits[node] for node in G.nodes()],
                'x': [pos[node][0] for node in G.nodes()],
                'y': [pos[node][1] for node in G.nodes()],
                'circle_size': [ 10 if node_visits[node] == 0 else 35 if node in data["best_path"] 
                else circle_size_min + ((node_visits[node]) / max(node_visits.values())) * (circle_size_max - circle_size_min) for node in G.nodes()],
                'circle_color':
                ["rgba(255, 0, 0, 1)" 
                if node in data["best_path"] 
                else "rgba(255, 255, 100, 0.8)" 
                if node_step_reward[node] >= np.quantile(list(node_step_reward.values()), 0.95) 
                else f"rgba(100, {50+min(1, (float(node_step_reward[node]) / step_reward_max))*205}, 100, {max(min(1, (float(node_step_reward[node]) / step_reward_max)+0.2), 0.5)})"
                for node in G.nodes()],
        }
    if len(lon_debug_reward) >= 1:
        node_info_lon = {
            'lon_debug_reward': [json.dumps(lon_debug_reward[node], indent=2) if node in lon_debug_reward else "" for node in G.nodes()],
            'lon_cost_f2': [json.dumps(lon_debug_cost[node]['f2']) if node in lon_debug_reward and 'f2' in lon_debug_cost[node] else "" for node in G.nodes()],
            'lon_cost_f1': [json.dumps(lon_debug_cost[node]['f1']) if node in lon_debug_reward and 'f1' in lon_debug_cost[node] else "" for node in G.nodes()],
            'lon_cost_0': [json.dumps(lon_debug_cost[node]['0']) if node in lon_debug_reward and '0' in lon_debug_cost[node] else "" for node in G.nodes()],
            'lon_cost_1': [json.dumps(lon_debug_cost[node]['1']) if node in lon_debug_reward and '1' in lon_debug_cost[node] else "" for node in G.nodes()],
            'lon_cost_2': [json.dumps(lon_debug_cost[node]['2']) if node in lon_debug_reward and '2' in lon_debug_cost[node] else "" for node in G.nodes()],
        }
        node_info.update(node_info_lon)
    elif len(spt_debug_reward) >= 1:
        node_info_spt = {
            'spt_debug_reward': [json.dumps({k: round(v, 3) for k, v in spt_debug_reward[node].items()},indent=2) if node in spt_debug_reward else "" for node in G.nodes()],
            # 'spt_debug_vru': [json.dumps(spt_debug_vru[node], indent=2)
            #           if node in spt_debug_reward else "" for node in G.nodes()],
            # 'spt_debug_vru': [format_vru(spt_debug_vru[node])  # 使用格式化函数
            #           if node in spt_debug_reward else "" for node in G.nodes()],
        }
        # 添加9个VRU分支字段
        for i in range(9):
            node_info_spt[f'spt_vru{i}_cost'] = [
                spt_debug_vru[f"spt_vru{i}_cost"].get(node, "") 
                for node in G.nodes()
                ]
        node_info.update(node_info_spt)

    node_source = ColumnDataSource(data=node_info)

    # 为图形渲染器设置节点数据源
    graph_renderer.node_renderer.data_source = node_source

    # 将节点形状改为圆形
    graph_renderer.node_renderer.glyph = Circle(x="x", y="y", radius="circle_size", fill_color="circle_color", line_color=None)

    # 创建树形图
    fig_mcts_tree.title.text +=', 总节点数:'+str(max( list(G.nodes())))
    # 计算 x 和 y 的范围
    x_min = min([pos[node][0] for node in pos]) - 100
    x_max = max([pos[node][0] for node in pos]) + 100
    y_min = min([pos[node][1] for node in pos]) - 100
    y_max = max([pos[node][1] for node in pos]) + 100

    # 使用 Range1d 设置范围
    fig_mcts_tree.x_range = Range1d(start=x_min, end=x_max)
    fig_mcts_tree.y_range = Range1d(start=y_min, end=y_max)


    # 添加图形渲染器到绘图
    fig_mcts_tree.renderers.append(graph_renderer)

    # 定义悬停工具，使悬停显示节点详细信息
    # hover = HoverTool(
    #     tooltips=[("Node ID", "@index"), ("Details", "@details{safe}"), ("Leader_state", "@node_leader_state{safe}"),
    #             ("Follower_state", "@node_follower_state{safe}"), ("Step_reward", "@node_step_reward{safe}"),
    #             ("Node_reward", "@node_reward{safe}"), ("Visits", "@node_visits")],
    #     renderers=[graph_renderer.node_renderer]
    # )
    # fig_mcts_tree.add_tools(hover)
    if len(lon_debug_reward) >= 1:
        hover_lon_debug = HoverTool(
            tooltips=[("Node ID", "@index"), ("Details", "@details{safe}"), ("Leader_state", "@node_leader_state{safe}"),
                    ("Follower_state", "@node_follower_state{safe}"), ("Step_reward", "@node_step_reward{safe}"),
                    ("Node_reward", "@node_reward{safe}"), ("Visits", "@node_visits"), ('lon_reward', '@lon_debug_reward{safe}'),
                    ('lon_cost_f2', '@lon_cost_f2{safe}'), 
                    ('lon_cost_f1', '@lon_cost_f1{safe}'), 
                    ('lon_cost_0', '@lon_cost_0{safe}'), 
                    ('lon_cost_1', '@lon_cost_1{safe}'), 
                    ('lon_cost_2', '@lon_cost_2{safe}')],
            renderers=[graph_renderer.node_renderer]
        )
        fig_mcts_tree.add_tools(hover_lon_debug)
    else:
        tooltips=[
            ("Node ID", "@index"),
            ("Details", "@details{safe}"),
            ("Leader_state", "@node_leader_state{safe}"),
            ("Follower_state", "@node_follower_state{safe}"),
            ("Step_reward", "@node_step_reward{safe}"),
            ("Node_reward", "@node_reward{safe}"),
            ("Visits", "@node_visits"),
            ('spt_reward', '@spt_debug_reward{safe}')
            ]
            # 添加9个VRU分支的悬停项
        for i in range(9):
            tooltips.append((f'spt_vru{i}_cost', f'@spt_vru{i}_cost{{safe}}'))
        hover_spt_debug = HoverTool(
            tooltips=tooltips,
            renderers=[graph_renderer.node_renderer]
        )
        fig_mcts_tree.add_tools(hover_spt_debug)

    # 添加标签集，始终显示节点编号
    labels = LabelSet(x='x', y='y', text='index', x_offset=-10, y_offset=3, source=node_source)
    fig_mcts_tree.add_layout(labels)

def visual_mcts_figure(data, fig_mcts_reward, fig_mcts_visit):
     # 初始化存储数据的列表
    reward_data = []
    visits_data = []

    child_count = len(data['search_debug'])
    for i in range(1, child_count + 1):
        reward_data.append(data['search_debug'][f'{i}']['reward_debug'])
        visits_data.append(data['search_debug'][f'{i}']['visits_debug'])

    # 生成横坐标数据（使用第一个子节点的长度作为基准）
    if reward_data:
        x = list(range(1, len(reward_data[0]) + 1))
    else:
        x = []    
        
    # 创建颜色列表
    colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', 
            '#8c564b', '#e377c2', '#7f7f7f', '#bcbd22', '#17becf']
    
    max_reward = max([max(sub_list) for sub_list in reward_data])

    # 使用 Range1d 设置范围
    fig_mcts_reward.x_range = Range1d(start=0, end=len(x))
    fig_mcts_reward.y_range = Range1d(start=0, end=max_reward)

    # 为每个子节点添加奖励值
    reward_renderers = []
    for i in range(child_count):
        color = colors[i % len(colors)]  # 循环使用颜色
        source = ColumnDataSource(data=dict(
            x=x,
            y=reward_data[i],
            line_color=[color] * len(x),
            legend_label=[f'{i+1}'] * len(x)
        ))
        renderer = fig_mcts_reward.line('x', 'y', source=source, line_color=color, line_width=2)
        reward_renderers.append(renderer)

    # 添加图例
    legend = Legend(items=[(f'{i+1}', [reward_renderers[i]]) for i in range(child_count)], location='top_left')
    fig_mcts_reward.add_layout(legend, 'right')

    # 使用 Range1d 设置范围
    fig_mcts_visit.x_range = Range1d(start=0, end=len(x))
    fig_mcts_visit.y_range = Range1d(start=0, end=max([max(sub_list) for sub_list in visits_data]))

    # 为每个x添加访问次数曲线
    visits_renderers = []
    for i in range(child_count):
        color = colors[i % len(colors)]  # 循环使用颜色
        source = ColumnDataSource(data=dict(
            x=x,
            y=visits_data[i],
            line_color=[color] * len(x),
            legend_label=[f'{i+1}'] * len(x)
        ))
        renderer = fig_mcts_visit.line('x', 'y', source=source, line_color=color, line_width=2)
        visits_renderers.append(renderer)

    # 添加图例
    legend = Legend(items=[(f'{i+1}', [visits_renderers[i]]) for i in range(child_count)], location='top_left')
    fig_mcts_visit.add_layout(legend, 'right')

def visual_mcts_st(data, fig_mcts_st):
    frames_data = data
    
    # 设置初始帧和对象ID
    init_frame = "0"
    init_obj = "0"
    
    # 确保初始帧存在
    if init_frame not in frames_data:
        print(f"初始帧 {init_frame} 不存在")
        return
    
    # 确保初始对象存在
    if init_obj not in frames_data[init_frame]:
        print(f"初始对象 {init_obj} 在帧 {init_frame} 中不存在")
        return
    
    # 初始化数据源
    source = ColumnDataSource(data=dict(
        xs=frames_data[init_frame][init_obj]['xs'],
        ys=frames_data[init_frame][init_obj]['ys'],
        id=frames_data[init_frame][init_obj]['id'],
        fill_color=frames_data[init_frame][init_obj]['color']
    ))
    
    # 创建图形
    p = figure(
        title="ST Graph Visualization", 
        tools="pan,box_zoom,wheel_zoom,reset,save,hover",
        tooltips=[("ID", "@id")], 
        x_range=(0, 8),
        width=800, 
        height=600
    )
    
    # 添加多边形
    p.patches('xs', 'ys', source=source, fill_alpha=0.3, line_color="black")
    
    # 配置悬停工具
    hover = p.select_one(HoverTool)
    hover.tooltips = [("ID", "@id")]
    
    # 创建帧滑块
    max_frame = max(int(frame) for frame in frames_data.keys())
    frame_slider = Slider(start=0, end=max_frame, value=0, step=1, title="Frame")
    
    # 创建对象滑块
    # -1 表示显示所有对象，0到最大ID表示显示特定对象
    max_obj_id = max(max(int(obj_id) for obj_id in frame_data.keys()) 
                    for frame_data in frames_data.values())
    obj_slider = Slider(start=-1, end=max_obj_id, value=0, step=1, title="Object (-1 for all)")
    
    # 创建JavaScript回调函数
    callback = CustomJS(args=dict(
        source=source,
        frame_slider=frame_slider,
        obj_slider=obj_slider,
        frames_data=frames_data
    ), code="""
        // 获取当前帧
        var frame = frame_slider.value.toString();
        if (!(frame in frames_data)) {
            console.log("Frame " + frame + " not found");
            return;
        }
        
        // 获取当前帧的所有对象ID
        var objKeys = Object.keys(frames_data[frame]).map(Number);
        objKeys.sort(function(a, b){ return a - b; });
        var maxObj = objKeys[objKeys.length - 1];
        
        // 更新对象滑块范围
        obj_slider.start = -1;
        obj_slider.end = maxObj;
        
        // 如果当前对象ID超出范围，重置为-1（全部）
        if (obj_slider.value < -1 || obj_slider.value > maxObj) {
            obj_slider.value = -1;
        }
        
        // 更新数据源
        var new_data = {xs: [], ys: [], id: [], fill_color: []};
        
        if (obj_slider.value == -1) {
            // 显示所有对象
            for (var i = 0; i < objKeys.length; i++) {
                var obj = objKeys[i].toString();
                if (obj in frames_data[frame]) {
                    var obj_data = frames_data[frame][obj];
                    new_data.xs = new_data.xs.concat(obj_data.xs);
                    new_data.ys = new_data.ys.concat(obj_data.ys);
                    new_data.id = new_data.id.concat(obj_data.id);
                    new_data.fill_color = new_data.fill_color.concat(obj_data.color);
                }
            }
        } else {
            // 显示特定对象
            var obj = obj_slider.value.toString();
            if (obj in frames_data[frame]) {
                new_data = frames_data[frame][obj];
            } else {
                console.log("Object " + obj + " not found in frame " + frame);
            }
        }
        
        // 更新数据源并触发更新
        source.data = new_data;
        source.change.emit();
    """)
    
    # 绑定回调函数
    frame_slider.js_on_change('value', callback)
    obj_slider.js_on_change('value', callback)
    
    # 设置图形范围
    p.x_range = Range1d(start=0, end=8)
    p.y_range = Range1d(start=0, end=10)  # 假设时间范围为0-10
    
    # 添加图形和控件到布局
    layout = column(p, row(frame_slider, obj_slider))
    fig_mcts_st.add_layout(layout)

def visual_single_obs(json_data, fig_mcts_st):
    if "original_st" not in json_data or "raw_st" not in json_data["original_st"] or "st" not in json_data["original_st"]:
        print("No original_st data found in the input.")
        return None
    all_xs = []
    all_ys = []
    all_ids = []
    
    # 收集raw_st数据
    raw_st = json_data["original_st"]["raw_st"]
    all_xs.extend(raw_st["xs"])
    all_ys.extend(raw_st["ys"])
    all_ids.extend(raw_st["id"])
    
    # 收集st数据
    st = json_data["original_st"]["st"]
    all_xs.extend(st["xs"])
    all_ys.extend(st["ys"])
    all_ids.extend(st["id"])

    modify_raw_st = []
    modify_st = []
    if "modify_st" in json_data:
        modify_raw_st = json_data["modify_st"]["raw_st"]
        modify_st = json_data["modify_st"]["st"]
        all_xs.extend(modify_raw_st["xs"])
        all_ys.extend(modify_raw_st["ys"])
        all_ids.extend(modify_raw_st["id"])
        all_xs.extend(modify_st["xs"])
        all_ys.extend(modify_st["ys"])
        all_ids.extend(modify_st["id"])
    
    # 计算坐标范围，添加边距以确保图形完整显示
    x_margin = (max(all_xs) - min(all_xs)) * 0.1
    y_margin = (max(all_ys) - min(all_ys)) * 0.1
    
    fig_mcts_st.x_range = Range1d(min(0, min(all_xs) - x_margin), max(max(all_xs) + x_margin,8))
    fig_mcts_st.y_range = Range1d(min(0, min(all_ys) - y_margin), max(all_ys) + y_margin)
    
    raw_st_source = ColumnDataSource(data=dict(
        xs=[raw_st["xs"]],
        ys=[raw_st["ys"]],
        id=[raw_st["id"]],
    ))
    
    raw_st_glyph = fig_mcts_st.patches('xs', 'ys', source=raw_st_source, 
              fill_color='blue', fill_alpha=0.3, 
              line_color='blue', line_width=1)
    
    # 绘制st区域（红色半透明）
    st_source = ColumnDataSource(data=dict(
        xs=[st["xs"]],
        ys=[st["ys"]],
        id=[st["id"]],
    ))
    
    st_glyph = fig_mcts_st.patches('xs', 'ys', source=st_source, 
              fill_color='red', fill_alpha=0.3, 
              line_color='red', line_width=1)
    
    legend_items = [
        (raw_st["id"], [raw_st_glyph]),
        (st["id"], [st_glyph])
    ]

    if "modify_st" in json_data:
        modify_raw_st_source = ColumnDataSource(data=dict(
            xs=[modify_raw_st["xs"]],
            ys=[modify_raw_st["ys"]],
            id=[modify_raw_st["id"]],
        ))
        
        modify_raw_st_glyph = fig_mcts_st.patches('xs', 'ys', source=modify_raw_st_source, 
                fill_color='green', fill_alpha=0.8, 
                line_color='green', line_width=2)
        
        modify_st_source = ColumnDataSource(data=dict(
            xs=[modify_st["xs"]],
            ys=[modify_st["ys"]],
            id=[modify_st["id"]],
        ))
        
        modify_st_glyph = fig_mcts_st.patches('xs', 'ys', source=modify_st_source, 
                fill_color='orange', fill_alpha=0.8, 
                line_color='orange', line_width=2)
        legend_items.extend([
            (modify_raw_st["id"], [modify_raw_st_glyph]),
            (modify_st["id"], [modify_st_glyph])
        ])
    legend = Legend(items=legend_items, location="top_right")
    
    # 将图例添加到图表
    fig_mcts_st.add_layout(legend, 'right')

    # 设置悬停工具提示
    hover = fig_mcts_st.select_one(HoverTool)
    hover.tooltips = [
        ("ID", "@id"),
        ("Point", "($x, $y)")
    ]

    # 设置图例可点击，用于切换显示
    fig_mcts_st.legend.click_policy = "hide"

def draw_vehicle(fig, x, y, heading, v, color, label, length, width):
    """在图上绘制车辆（矩形）和速度箭头"""
    # 计算车辆矩形的四个角点
    half_length = 0.5 * length
    half_width = 0.5 * width
    cos_h = math.cos(heading)
    sin_h = math.sin(heading)
    
    corners = [
        (x + half_length*cos_h - half_width*sin_h, y + half_length*sin_h + half_width*cos_h),
        (x - half_length*cos_h - half_width*sin_h, y - half_length*sin_h + half_width*cos_h),
        (x - half_length*cos_h + half_width*sin_h, y - half_length*sin_h - half_width*cos_h),
        (x + half_length*cos_h + half_width*sin_h, y + half_length*sin_h - half_width*cos_h)
    ]
    
    # 绘制车辆矩形
    xs = [c[0] for c in corners]
    ys = [c[1] for c in corners]
    fig.patch(xs, ys, fill_color=color, fill_alpha=0.6, line_color="black")
    
    # 绘制速度箭头（长度与速度成正比）
    vx = x + v * cos_h * 0.5
    vy = y + v * sin_h * 0.5
    fig.add_layout(Arrow(end=VeeHead(size=8, fill_color="red"), 
                        x_start=x, y_start=y, x_end=vx, y_end=vy,
                        line_color="red", line_width=2))
    
    # 添加标签
    fig.text(x, y, text=[label], text_font_size="10pt", text_color="black", 
             text_align="center", text_baseline="middle")

def visual_best_path_xy(data, fig_xy):
    """绘制最佳路径的x-y坐标系示意图"""
    if "best_path" not in data or not data["best_path"]:
        print("No best_path found in the input.")
        return
    
    # 提取最佳路径节点
    best_path = data["best_path"]
    leader_positions = []
    follower_positions = []
    
    # 收集路径上的所有位置
    for node_id in best_path:
        node = data.get(str(node_id))
        if not node:
            continue
        if "x" not in node["state"]["leader_status"] or "heading" not in node["state"]["leader_status"]:
            return
        leader = node["state"]["leader_status"]
        follower = node["state"]["follower_status"]
        
        leader_positions.append((leader["x"], leader["y"], leader["heading"], leader["v"]))
        follower_positions.append((follower["x"], follower["y"], follower["heading"], follower["v"]))
    
    if not leader_positions or not follower_positions:
        return
    
    # 设置图形范围
    all_x = [pos[0] for pos in leader_positions + follower_positions]
    all_y = [pos[1] for pos in leader_positions + follower_positions]
    
    x_min, x_max = min(all_x) - 5, max(all_x) + 5
    y_min, y_max = min(all_y) - 5, max(all_y) + 5
    
    fig_xy.x_range = Range1d(x_min, x_max)
    fig_xy.y_range = Range1d(y_min, y_max)
    
    # 绘制起点和终点标记
    fig_xy.circle([leader_positions[0][0]], [leader_positions[0][1]], 
                 size=10, color="green", alpha=0.7, legend_label="Start (Leader)")
    fig_xy.circle([follower_positions[0][0]], [follower_positions[0][1]], 
                 size=10, color="blue", alpha=0.7, legend_label="Start (Follower)")
    
    fig_xy.circle([leader_positions[-1][0]], [leader_positions[-1][1]], 
                 size=10, color="red", alpha=0.7, legend_label="End (Leader)")
    fig_xy.circle([follower_positions[-1][0]], [follower_positions[-1][1]], 
                 size=10, color="orange", alpha=0.7, legend_label="End (Follower)")
    
    # 绘制路径线
    leader_x, leader_y = zip(*[(pos[0], pos[1]) for pos in leader_positions])
    follower_x, follower_y = zip(*[(pos[0], pos[1]) for pos in follower_positions])
    
    fig_xy.line(leader_x, leader_y, line_width=2, color="green", 
               legend_label="Leader Path", line_dash="dashed")
    fig_xy.line(follower_x, follower_y, line_width=2, color="blue", 
               legend_label="Follower Path", line_dash="dashed")
    
    # 绘制每个节点的车辆位置和速度
    for i, (lx, ly, lh, lv) in enumerate(leader_positions):
        draw_vehicle(fig_xy, lx, ly, lh, lv, "green", f"L{i}", 4.8, 2.0)
    
    for i, (fx, fy, fh, fv) in enumerate(follower_positions):
        draw_vehicle(fig_xy, fx, fy, fh, fv, "blue", f"F{i}", 0.6, 0.5)
    
    # 添加图例和标题
    fig_xy.title.text = "Best Path Visualization"
    fig_xy.legend.location = "top_right"
    fig_xy.legend.click_policy = "hide"
    
    # 添加网格和坐标轴标签
    fig_xy.grid.grid_line_color = "gray"
    fig_xy.grid.grid_line_alpha = 0.2
    fig_xy.xaxis.axis_label = "X Position"
    fig_xy.yaxis.axis_label = "Y Position"

def visual_mcts(data, figs_mcts_tree, figs_mcts_visit, figs_mcts_reward, figs_mcts_st, figs_xy, id_selector):
    id_selector.options = [(str(i), str(i)) for i in data['traj_ids']]
    id_selector.value = [str(i) for i in data['traj_ids']]
    for value in data['traj_ids']:
        if value in data:
            if data[value]['is_valid'] == False:
                figs_mcts_tree[value].background_fill_color="#ffe6e6"
                figs_xy[value].background_fill_color="#ffe6e6"
            figs_mcts_tree[value].title.text = 'frame id: ' + str(data['frame_id'])
            figs_mcts_tree[value].title.text += ', 障碍物id: '+ str(value) 
            figs_mcts_tree[value].title.text += ', 场景: '+ data[value]['scenario_info']
            visual_mcts_tree(data[value], figs_mcts_tree[value])
            visual_single_obs(data[value], figs_mcts_st[value])
            visual_best_path_xy(data[value], figs_xy[value])  # 添加最佳路径可视化
            
            if data[value]['search_debug'] is not None:
                visual_mcts_figure(data[value], figs_mcts_reward[value], figs_mcts_visit[value])
    update_figure_js = CustomJS(args=dict(
        figs_mcts_tree=figs_mcts_tree,
        figs_mcts_visit = figs_mcts_visit,
        figs_mcts_reward = figs_mcts_reward,
        figs_mcts_st = figs_mcts_st,
        figs_xy=figs_xy,
        select_fig=id_selector
        ), code="""
    const selected_values = select_fig.value;
    
    for (const [group_name, fig_mcts_tree] of Object.entries(figs_mcts_tree)) {
        fig_mcts_tree.visible = selected_values.includes(group_name);
        
        if (figs_mcts_visit[group_name]) {
            figs_mcts_visit[group_name].visible = fig_mcts_tree.visible;
        }
        if (figs_mcts_reward[group_name]) {
            figs_mcts_reward[group_name].visible = fig_mcts_tree.visible;
        }
        if (figs_mcts_st[group_name]) {
            figs_mcts_st[group_name].visible = fig_mcts_tree.visible;
        }
        if (figs_xy[group_name]) {
            figs_xy[group_name].visible = fig_mcts_tree.visible;
        }
    }
""")
    id_selector.js_on_change('value', update_figure_js)