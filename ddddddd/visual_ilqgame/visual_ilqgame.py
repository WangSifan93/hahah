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
import networkx as nx
import matplotlib.pyplot as plt
import math
from bokeh.models import Arrow, NormalHead, VeeHead, BoxAnnotation,OpenHead
from bokeh.transform import cumsum
def visual_xy(fig, vehicle_id, traj_data, ref_line, length,width,color):
    """可视化XY位置和参考线"""
    # 按时间排序
    sorted_points = sorted(traj_data.items(), key=lambda x: float(x   [0]))
    x = [point['x'] for _, point in sorted_points]
    y = [point['y'] for _, point in sorted_points]
    fig.line(x, y, line_width=2.5, color=color, legend_label=f"{vehicle_id} XY")
    
    # 绘制参考线
    if ref_line: 
        # 提取参考线的x和y坐标
        ref_x = [point[0] for point in ref_line]
        ref_y = [point[1] for point in ref_line]
        # 使用虚线绘制参考线
        fig.line(ref_x, ref_y, line_width=2, color='red',line_dash="dashed", alpha=0.8,
                 legend_label=f"Reference Line")
        
    points = [point for _, point in sorted_points]
    # 每隔20个点绘制方向箭头
    arrow_size = 12  # 箭头大小
    for i in range(0, len(points) - 1, 20):
        # 计算方向向量
        dx = points[i+1]['x'] - points[i]['x']
        dy = points[i+1]['y'] - points[i]['y']        
        fig.add_layout(Arrow(
            end=NormalHead(fill_color=color, size=arrow_size),
            x_start=points[i]['x'],
            y_start=points[i]['y'],
            x_end=points[i]['x'] + dx * 0.5,  # 指向下一个点的一半距离
            y_end=points[i]['y'] + dy * 0.5,
            line_color=color,
            line_alpha=0.8
        ))

        #绘制车辆矩形
    for i in range(0, len(points) - 1, 5):
        dx = points[i+1]['x'] - points[i]['x']
        dy = points[i+1]['y'] - points[i]['y']
        angle = np.arctan2(dy, dx)
        
        # 绘制车辆矩形
        fig.rect(
            x=points[i]['x'],
            y=points[i]['y'],
            width=length,
            height=width,
            angle=angle,
            angle_units="rad",
            fill_color=None,
            line_color=color,
            line_width =2
        )
        
    fig.xaxis.axis_label = "X Position"
    fig.yaxis.axis_label = "Y Position"


def visual_heading(fig, vehicle_id, traj_data, color):
    """可视化航向角（按时间顺序）"""
    # 按时间排序
    sorted_points = sorted(traj_data.items(), key=lambda x: float(x  [0]))
    times = [point['time'] for _, point in sorted_points]
    headings = [point['heading'] for _, point in sorted_points]
    fig.line(times, headings, line_width=2, color=color, legend_label=f"{vehicle_id} Heading")
    fig.xaxis.axis_label = "Time (s)"
    fig.yaxis.axis_label = "Heading (rad)"

def visual_velocity(fig, vehicle_id, traj_data, desired_v,color):
    """可视化速度（按时间顺序）"""
    # 按时间排序
    sorted_points = sorted(traj_data.items(), key=lambda x: float(x  [0]))
    times = [point['time'] for _, point in sorted_points]
    velocities = [point['v'] for _, point in sorted_points]
    fig.line(times, velocities, line_width=2, color=color, legend_label=f"{vehicle_id} Velocity")
    # 绘制期望速度（水平线）
    if desired_v is not None:
        # 获取时间范围
        min_time = min(times)
        max_time = max(times)
        
        # 绘制水平线表示期望速度
        fig.line([min_time, max_time], [desired_v, desired_v], 
                 line_width=2, color=color, line_dash="dashed", 
                 legend_label=f"{vehicle_id} Desired Velocity")
    
    fig.xaxis.axis_label = "Time (s)"
    fig.yaxis.axis_label = "Velocity (m/s)"

def visual_steer(fig, vehicle_id, traj_data, color):
    """可视化方向盘转角（按时间顺序）"""
    # 按时间排序
    sorted_points = sorted(traj_data.items(), key=lambda x: float(x  [0]))
    times = [point['time'] for _, point in sorted_points]
    wheel_angles = [point['wheel_angle'] for _, point in sorted_points]
    fig.line(times, wheel_angles, line_width=2, color=color, legend_label=f"{vehicle_id} Wheel Angle")
    fig.xaxis.axis_label = "Time (s)"
    fig.yaxis.axis_label = "Wheel Angle (rad)"

def visual_acc(fig, vehicle_id, traj_data, color):
    """可视化加速度（按时间顺序）"""
    # 按时间排序
    sorted_points = sorted(traj_data.items(), key=lambda x: float(x  [0]))
    times = [point['time'] for _, point in sorted_points]
    accelerations = [point['a'] for _, point in sorted_points]
    fig.line(times, accelerations, line_width=2, color=color, legend_label=f"{vehicle_id} Acceleration")
    fig.xaxis.axis_label = "Time (s)"
    fig.yaxis.axis_label = "Acceleration (m/s²)"

def visual_ilqgame(data,xy_fig, heading_fig, velocity_fig, wheel_angle_fig, acceleration_fig):
    """主可视化函数"""
    
    # 为不同车辆分配颜色
    colors = plt.rcParams['axes.prop_cycle'].by_key()['color']
    n=0
    # 遍历所有车辆
    for vehicle_id, vehicle_data in data["vehicles"].items():

        # 获取轨迹数据和参考线
        traj_data = vehicle_data["traj"]
        ref_line = vehicle_data.get("ref_line", [])  
        desired_v = vehicle_data.get("desired_v", None) 
        length = vehicle_data.get("length", 4.5) 
        width = vehicle_data.get("width", 2) 
        color = colors[n % len(colors)]
        # 调用五个可视化函数，注意visual_xy增加了ref_line参数
        visual_xy(xy_fig, vehicle_id, traj_data, ref_line, length,width,color)
        visual_heading(heading_fig, vehicle_id, traj_data, color)
        visual_velocity(velocity_fig, vehicle_id, traj_data, desired_v,color)
        visual_steer(wheel_angle_fig, vehicle_id, traj_data, color)
        visual_acc(acceleration_fig, vehicle_id, traj_data, color)
        n+=1
    # 统一调整所有图形的图例设置
    for fig in [xy_fig, heading_fig, velocity_fig, wheel_angle_fig, acceleration_fig]:
        fig.legend.location = "top_right"
        fig.legend.label_text_font_size = "10pt"
        fig.legend.background_fill_alpha = 0.5
        fig.legend.click_policy = "hide"  # 允许点击隐藏图例