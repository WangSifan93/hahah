import json
from bokeh.io import show, output_file, output_notebook
from bokeh.models import ColumnDataSource, HoverTool, LabelSet, Circle, Legend
from bokeh.plotting import figure, from_networkx
from bokeh.layouts import row, column
import networkx as nx
import numpy as np

# 加载 JSON 数据
# with open('metrics/mcts_tree_1.json', 'r') as file:
#     data = json.load(file)
with open('/home/not0296/work/interactive_plan/MCTS/build/mcts_tree_1.json', 'r') as file:
    data = json.load(file)

# 创建 NetworkX 图
G = nx.DiGraph()

# 用于存储每个节点的详细信息
node_details = {}
node_leader_state = {}
node_follower_state = {}
node_step_reward = {}
node_reward = {}
node_visits = {}

def build_graph(node):
    node_id = node['id']
    G.add_node(node_id)
    node_details[node_id] = "is_leaf:"+str(data[str(node_id)]['is_leaf']) +", parent_id:"+ str(data[str(node_id)]['parent_id'])+ ", current_time:"+ str(data[str(node_id)]['current_time'])
    node_leader_state[node_id] = data[str(node_id)]["state"]["leader_status"]
    node_follower_state[node_id] = data[str(node_id)]["state"]["follower_status"]
    node_step_reward[node_id] = data[str(node_id)]["step_reward"]
    node_reward[node_id] = data[str(node_id)]["reward"]
    node_visits[node_id] = data[str(node_id)]['visits']
    for child in node.get('children', []):
        G.add_edge(node_id, child['id'])
        build_graph(child)

build_graph(data['tree'])

# 使用 graphviz_layout 进行层级布局
try:
    pos = nx.drawing.nx_agraph.graphviz_layout(G, prog='dot')
except ImportError:
    print("请安装 pygraphviz 库以使用 graphviz_layout 布局算法。")
    pos = nx.spring_layout(G)

# 用 Bokeh 从 NetworkX 图创建图形渲染器
graph_renderer = from_networkx(G, pos, scale=1, center=(0, 0))

# 定义圆圈大小的最大值和最小值
circle_size_min = 5
circle_size_max = 100
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
    'circle_size': [circle_size_min + (float(node_step_reward[node]) / step_reward_max) * (circle_size_max - circle_size_min) for node in G.nodes()],
    'circle_color':
    ["rgba(255, 0, 0, 1)" 
    if node in data["best_path"] 
    else "rgba(255, 255, 100, 0.8)" 
    if node_step_reward[node] >= np.quantile(list(node_step_reward.values()), 0.9) 
    else f"rgba(100, {100+min(1, (node_visits[node] / max(node_visits.values()) + 0.5))*155}, 100, {max(min(1, (node_visits[node] / max(node_visits.values()) + 0.5)), 0.5)})"
    for node in G.nodes()]
}

node_source = ColumnDataSource(data=node_info)

# 为图形渲染器设置节点数据源
graph_renderer.node_renderer.data_source = node_source

# 将节点形状改为圆形
graph_renderer.node_renderer.glyph = Circle(x="x", y="y", radius="circle_size", fill_color="circle_color", line_color=None)

# 创建树形图
tree_plot = figure(width=1200, height=900, title='Tree Visualization '+'总节点数:'+str(max( list(G.nodes()))), tools='pan,wheel_zoom,save,reset',
           x_range=(min([x for x, y in pos.values()]) - 30, max([x for x, y in pos.values()]) + 30),
           y_range=(min([y for x, y in pos.values()]) - 30, max([y for x, y in pos.values()]) + 30))

# 添加图形渲染器到绘图
tree_plot.renderers.append(graph_renderer)

# 定义悬停工具，使悬停显示节点详细信息
hover = HoverTool(
    tooltips=[("Node ID", "@index"), ("Details", "@details{safe}"), ("Leader_state", "@node_leader_state{safe}"),
              ("Follower_state", "@node_follower_state{safe}"), ("Step_reward", "@node_step_reward{safe}"),
              ("Node_reward", "@node_reward{safe}"), ("Visits", "@node_visits")],
    renderers=[graph_renderer.node_renderer]
)
tree_plot.add_tools(hover)

# 添加标签集，始终显示节点编号
labels = LabelSet(x='x', y='y', text='index', x_offset=-10, y_offset=3, source=node_source)
tree_plot.add_layout(labels)

# 获取实际存在的子节点数量
child_count = len(data['search_debug'])

# 初始化存储数据的列表
reward_data = []
visits_data = []
for i in range(1, child_count + 1):
    reward_data.append(data['search_debug'][f'{i}']['reward_debug'])
    visits_data.append(data['search_debug'][f'{i}']['visits_debug'])

# 生成横坐标数据（使用第一个子节点的长度作为基准）
if reward_data:
    x = list(range(1, len(reward_data[0]) + 1))
else:
    x = []

# 创建颜色列表（扩展到支持最多10个子节点，可根据需要增加）
colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', 
          '#8c564b', '#e377c2', '#7f7f7f', '#bcbd22', '#17becf']

# 创建奖励值图
reward_plot = figure(width=400, height=300, title='第一层子结点奖励值变化趋势', 
                     x_axis_label='索引', y_axis_label='奖励值',
                     tools='pan,wheel_zoom,box_zoom,reset,save')

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
    renderer = reward_plot.line('x', 'y', source=source, line_color=color, line_width=2)
    reward_renderers.append(renderer)

# 添加图例
legend = Legend(items=[(f'{i+1}', [reward_renderers[i]]) for i in range(child_count)], location='top_left')
reward_plot.add_layout(legend, 'right')

# 创建访问次数图
visits_plot = figure(width=400, height=300, title='第一层子结点访问次数变化趋势', 
                     x_axis_label='索引', y_axis_label='访问次数',
                     tools='pan,wheel_zoom,box_zoom,reset,save')

# 为每个子节点添加访问次数曲线
visits_renderers = []
for i in range(child_count):
    color = colors[i % len(colors)]  # 循环使用颜色
    source = ColumnDataSource(data=dict(
        x=x,
        y=visits_data[i],
        line_color=[color] * len(x),
        legend_label=[f'{i+1}'] * len(x)
    ))
    renderer = visits_plot.line('x', 'y', source=source, line_color=color, line_width=2)
    visits_renderers.append(renderer)

# 添加图例
legend = Legend(items=[(f'{i+1}', [visits_renderers[i]]) for i in range(child_count)], location='top_left')
visits_plot.add_layout(legend, 'right')

# 创建布局
left_layout = tree_plot  # 假设tree_plot已在其他地方定义
right_layout = column(reward_plot, visits_plot)
main_layout = row(left_layout, right_layout, sizing_mode='scale_width')

# 显示布局
show(main_layout)
