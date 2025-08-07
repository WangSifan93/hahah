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

def create_mcts_tree_figure():
    wzt_mcts_tree = WheelZoomTool()
    fig_mcts_tree = figure(
        name="mcts_tree",
        width=1600,
        height=400,
        match_aspect=True,
        output_backend="svg",
        x_range=(-20, 120),
        y_range=(-25, 25),
        tools=["reset", "box_zoom", "pan", wzt_mcts_tree, "save"],
        toolbar_location='above',
    )
    fig_mcts_tree.toolbar.active_scroll = wzt_mcts_tree
    return fig_mcts_tree

def create_reward_visit_figure():
    wzt_mcts_visit = WheelZoomTool()
    TOOLTIPS = [
        ("index", "$index"),
        ("(x,y)", "($x, $y)"),
    ]
    fig_mcts_visit = figure(
        name="mcts_visit",
        width=350,
        height=350,
        title='第一层子结点访问次数变化趋势',
        # match_aspect=True,
        tooltips=TOOLTIPS,
        output_backend="svg",
        x_range=(0, 100),
        y_range=(0, 10),
        tools=["reset", "box_zoom", "pan", "hover", "tap", wzt_mcts_visit, "save"],
        toolbar_location='above',
    )
    fig_mcts_visit.toolbar.active_scroll = wzt_mcts_visit
    return fig_mcts_visit

def create_reward_figure():
    wzt_mcts_reward = WheelZoomTool()
    TOOLTIPS = [
        ("index", "$index"),
        ("(x,y)", "($x, $y)"),
    ]
    fig_mcts_reward = figure(
        name="mcts_reward",
        width=350,
        height=350,
        title='第一层子结点奖励值变化趋势',
        # match_aspect=True,
        tooltips=TOOLTIPS,
        output_backend="svg",
        x_range=(0, 100),
        y_range=(0, 10),
        tools=["reset", "box_zoom", "pan", "hover", "tap", wzt_mcts_reward, "save"],
        toolbar_location='above',
    )
    fig_mcts_reward.toolbar.active_scroll = wzt_mcts_reward
    return fig_mcts_reward

def create_st_figure():
    wzt_mcts_st = WheelZoomTool()
    fig_mcts_st = figure(
        name="mcts_st",
        width=450,
        height=350,
        title="ST Graph Visualization",
        tooltips=[("ID", "@id")],
        output_backend="svg",
        x_range=(0, 9),
        y_range=(0, 10),
        tools=["reset", "box_zoom", "pan", "hover", wzt_mcts_st, "save"],
        toolbar_location='above',
    )
    fig_mcts_st.toolbar.active_scroll = wzt_mcts_st
    return fig_mcts_st

def create_xy_figure():
    wzt_mcts_xy = WheelZoomTool()
    fig_mcts_xy = figure(
        title=f"mcts_xy",
        tools=["reset", "box_zoom", "pan", "hover", wzt_mcts_xy, "save"],
        width=550,
        height=350
        )
    return fig_mcts_xy

def create_fig_default(data):
    tree_figures_map = {}
    visits_figures_map = {}
    reward_figures_map = {}
    st_figures_map = {}
    xy_figures_map = {}
    all_layouts = []
    for value in data['traj_ids']:
        tree_figures_map[value] = create_mcts_tree_figure()
        visits_figures_map[value] = create_reward_visit_figure()
        reward_figures_map[value] = create_reward_figure()
        st_figures_map[value] = create_st_figure()
        xy_figures_map[value] = create_xy_figure()
        all_layouts.append(column(tree_figures_map[value],
            row(xy_figures_map[value],
            st_figures_map[value],
            visits_figures_map[value],
            reward_figures_map[value]
            ),
            ))

    id_selector = MultiSelect(
        title="轨迹ID:",
        width=80,
        height=200,
    )

    # top_row = row(id_selector)
    final_layout = row(column(*all_layouts), id_selector)
    return tree_figures_map, visits_figures_map, reward_figures_map, st_figures_map, xy_figures_map, id_selector, final_layout