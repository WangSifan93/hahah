import re
from bokeh.plotting import figure, show, output_file
from bokeh.models import HoverTool, ColumnDataSource, Slider
from bokeh.layouts import column
from bokeh.io import curdoc
from itertools import cycle

# 读取文件内容
with open('e2e_pnc.log', 'r') as file:
    lines = file.readlines()

# 解析文件内容
frames = []
current_frame = None
current_lane_id = None
current_lane_path_idx = None
navi_priority_lane_id = None

for line in lines:
    seq_match = re.match(r'seq: (\d+)', line)
    if seq_match:
        if current_frame:
            frames.append(current_frame)
        current_frame = {
            'seq': seq_match.group(1),
            'ego_pos': None,
            'near_seg_points': [],
            'lane_data': {},  # 使用字典存储每个 Lane ID 的点
            'lane_paths': {},  # 使用字典存储每个 Lane Path idx 的点
            'out_put_traj': [],
            'navi_priority_lane_id': []
        }
        current_lane_id = None
        current_lane_path_idx = None
        navi_priority_lane_id = None
    if current_frame:
        # 解析 ego_pos
        ego_pos_match = re.match(r'ego_pos: ([-\d.]+), ([-\d.]+), ([-\d.]+)', line)
        if ego_pos_match:
            current_frame['ego_pos'] = (
                float(ego_pos_match.group(1)),
                float(ego_pos_match.group(2)),
                float(ego_pos_match.group(3))
            )
        # 解析 near_seg_points
        near_seg_match = re.match(r'\(([-\d.]+), ([-\d.]+)\)', line)
        if near_seg_match:
            x, y = map(float, near_seg_match.groups())
            current_frame['near_seg_points'].append((x, y))
        # 解析 Lane ID
        lane_id_match = re.match(r'Lane ID: (\d+)', line)
        if lane_id_match:
            current_lane_id = lane_id_match.group(1)
            if current_lane_id not in current_frame['lane_data']:
                current_frame['lane_data'][current_lane_id] = {'x': [], 'y': []}
        # 解析 Navi Priority Lane ID
        navi_prio_id_match = re.match(r'navi_priority_lane_id: (\d+)', line)
        if navi_prio_id_match:
            current_lane_id = navi_prio_id_match.group(1)
            current_frame['navi_priority_lane_id'].append(current_lane_id)
        # 解析 Lane ID 下的 Point 数据
        point_match = re.match(r'Point: \(([-\d.]+), ([-\d.]+)\)', line)
        if point_match and current_lane_id:
            x, y = map(float, point_match.groups())
            current_frame['lane_data'][current_lane_id]['x'].append(x)
            current_frame['lane_data'][current_lane_id]['y'].append(y)
        # 解析 Lane Path idx
        lane_path_idx_match = re.match(r'Lane path idx: (\d+)', line)
        if lane_path_idx_match:
            current_lane_path_idx = lane_path_idx_match.group(1)
            if current_lane_path_idx not in current_frame['lane_paths']:
                current_frame['lane_paths'][current_lane_path_idx] = {'x': [], 'y': []}
        # 解析 Lane Path Point 数据
        lane_path_point_match = re.match(r'Lane path point: \(([-\d.]+), ([-\d.]+)\)', line)
        if lane_path_point_match and current_lane_path_idx:
            x, y = map(float, lane_path_point_match.groups())
            current_frame['lane_paths'][current_lane_path_idx]['x'].append(x)
            current_frame['lane_paths'][current_lane_path_idx]['y'].append(y)
        # 解析 out_put_traj
        out_put_traj_match = re.match(r'x: ([-\d.]+) , y: ([-\d.]+)', line)
        if out_put_traj_match:
            x, y = map(float, out_put_traj_match.groups())
            current_frame['out_put_traj'].append((x, y))

if current_frame:
    frames.append(current_frame)

# 设置输出文件
output_file("section_points_with_ego_pose.html")

# 创建绘图对象
p = figure(
    title="Frame 1",
    x_axis_label='X',
    y_axis_label='Y',
    tools="pan,wheel_zoom,box_zoom,reset,save",
    width=1200,
    height=800
)
p.add_tools(HoverTool(tooltips=[("(x,y)", "(@x{0.000}, @y{0.000})")]))

# 创建数据源
ego_source = ColumnDataSource(data=dict(x=[], y=[], width=[], height=[], angle=[]))
near_seg_source = ColumnDataSource(data=dict(x=[], y=[]))
lane_source = ColumnDataSource(data=dict(xs=[], ys=[]))  # 使用 multi_line
navi_lane_source = ColumnDataSource(data=dict(xs=[], ys=[]))
lane_path_sources = {}  # 存储每个 Lane Path idx 的数据源
out_put_traj_source = ColumnDataSource(data=dict(x=[], y=[]))

# 绘制初始图形
p.rect('x', 'y', width='width', height='height', angle='angle', source=ego_source, color="red", legend_label="Ego Pose")
p.circle('x', 'y', source=near_seg_source, size=5, color="blue", legend_label="Near Seg Points")
p.multi_line('xs', 'ys', source=lane_source, line_width=2, color="green", legend_label="Lane Points")
p.multi_line('xs', 'ys', source=navi_lane_source, line_width=4, color="purple", legend_label="Navi Priority Lanes")
p.line('x', 'y', source=out_put_traj_source, line_width=2, color="black", legend_label="OutPut Traj")

# 使用不同颜色绘制 Lane Path idx
colors = cycle(["blue", "orange", "purple", "brown", "pink", "cyan", "magenta", "yellow"])
for idx in range(10):  # 假设最多有 10 个 Lane Path idx
    lane_path_sources[str(idx)] = ColumnDataSource(data=dict(x=[], y=[]))
    p.line('x', 'y', source=lane_path_sources[str(idx)], line_width=2, color=next(colors), legend_label=f"Lane Path {idx}")

# 创建滑块
slider = Slider(start=1, end=len(frames), value=1, step=1, title="Frame")

# 更新函数
def update(attr, old, new):
    frame = frames[slider.value - 1]
    p.title.text = f"Frame {frame['seq']}"
    if frame['ego_pos']:
        ego_x, ego_y, ego_yaw = frame['ego_pos']
        ego_source.data = dict(x=[ego_x], y=[ego_y], width=[1.4], height=[0.7], angle=[ego_yaw])
    else:
        ego_source.data = dict(x=[], y=[], width=[], height=[], angle=[])
    if frame['near_seg_points']:
        near_seg_x, near_seg_y = zip(*frame['near_seg_points'])
        near_seg_source.data = dict(x=list(near_seg_x), y=list(near_seg_y))
    else:
        near_seg_source.data = dict(x=[], y=[])
    # 更新 Lane 数据
    xs = []
    ys = []
    navi_xs = []
    navi_ys = []
    navi_priority_lane_ids = frame['navi_priority_lane_id']
    for lane_id, points in frame['lane_data'].items():
        if lane_id in navi_priority_lane_ids:
            navi_xs.append(points['x'])
            navi_ys.append(points['y'])
        else:
            xs.append(points['x'])
            ys.append(points['y'])
    lane_source.data = dict(xs=xs, ys=ys)
    navi_lane_source.data = dict(xs=navi_xs, ys=navi_ys)
    # 更新 Lane Path 数据
    for idx, source in lane_path_sources.items():
        if idx in frame['lane_paths']:
            source.data = frame['lane_paths'][idx]
        else:
            source.data = dict(x=[], y=[])
    # 更新输出轨迹
    if frame['out_put_traj']:
        out_put_traj_x, out_put_traj_y = zip(*frame['out_put_traj'])
        out_put_traj_source.data = dict(x=list(out_put_traj_x), y=list(out_put_traj_y))
    else:
        out_put_traj_source.data = dict(x=[], y=[])

slider.on_change('value', update)

# 初始更新
update(None, None, 1)

# 创建布局
layout = column(slider, p, sizing_mode='scale_width')

# 显示绘图
curdoc().add_root(layout)
show(layout)
