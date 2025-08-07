from typing import Dict, List
import sys
import numpy as np
import random
from bokeh.plotting.glyph_api import GlyphAPI
from bokeh.plotting import figure, from_networkx, ColumnDataSource
from bokeh.layouts import layout,column,row
from bokeh.palettes import Sunset8
import random
from bokeh.models import (
    WheelZoomTool,
    MultiSelect,
)

def create_xy_figure():
    wzt_ilqgame_xy = WheelZoomTool()
    fig_ilqgame_xy = figure(
        title=f"XY_position",
        tools=["reset", "box_zoom", "pan", "hover", wzt_ilqgame_xy, "save"],
        width=1000,
        height=300
        )
    return fig_ilqgame_xy

def create_heading_figure():
    wzt_ilqgame_heading = WheelZoomTool()
    fig_ilqgame_heading = figure(
        title=f"Heading",
        tools=["reset", "box_zoom", "pan", "hover", wzt_ilqgame_heading, "save"],
        width=500,
        height=300
        )
    return fig_ilqgame_heading

def create_velocity_figure():
    wzt_ilqgame_velocity = WheelZoomTool()
    fig_ilqgame_velocity = figure(
        title=f"Velocity",
        tools=["reset", "box_zoom", "pan", "hover", wzt_ilqgame_velocity, "save"],
        width=500,
        height=300
        )
    return fig_ilqgame_velocity

def create_steer_figure():
    wzt_ilqgame_steer = WheelZoomTool()
    fig_ilqgame_steer = figure(
        title=f"Wheel_angle",
        tools=["reset", "box_zoom", "pan", "hover", wzt_ilqgame_steer, "save"],
        width=500,
        height=300
        )
    return fig_ilqgame_steer

def create_acc_figure():
    wzt_ilqgame_acc = WheelZoomTool()
    fig_ilqgame_acc = figure(
        title=f"Acceleration",
        tools=["reset", "box_zoom", "pan", "hover", wzt_ilqgame_acc, "save"],
        width=500,
        height=300
        )
    return fig_ilqgame_acc

def create_fig_default(data):

    all_layouts = []
    
    # 创建五个空图表（不包含数据）
    xy_fig = create_xy_figure()
    heading_fig = create_heading_figure()
    velocity_fig = create_velocity_figure()
    wheel_angle_fig = create_steer_figure()
    acceleration_fig = create_acc_figure()
        
    # 创建当前车辆的布局
    vehicle_layout = column(
        xy_fig,
        row(heading_fig,velocity_fig),
        row(wheel_angle_fig,acceleration_fig)
    )

    all_layouts.append(vehicle_layout)
    
    # 创建最终布局
    final_layout = row(
        column(*all_layouts, sizing_mode="scale_width")
    )
    
    return xy_fig,heading_fig,velocity_fig,wheel_angle_fig,acceleration_fig,final_layout
    