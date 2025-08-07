from bokeh.plotting import figure, show, output_file
from bokeh.models import ColumnDataSource, HoverTool

# 定义多边形的属性
ids = ['1', '2', '3']
# 这里每个多边形用一组 (x, y) 坐标来表示
xs = [[0, 1, 1, 0], [1, 2, 2, 1], [2, 3, 3, 2]]
ys = [[0, 0, 1, 1], [0, 0, 1, 1], [0, 0, 1, 1]]

# 为每个多边形定义不同的颜色
fill_colors = ['red', 'green', 'blue']

# 创建数据源
source = ColumnDataSource(data=dict(
    xs=xs,
    ys=ys,
    id=ids,
    fill_color=fill_colors
))

# 创建图形
p = figure(title="Hoverable Polygons", tools="hover",
           tooltips=[("ID", "@id")])

# 绘制多边形
p.patches('xs', 'ys', source=source, fill_color='fill_color', line_color='black')

# 配置悬停工具
hover = p.select(type=HoverTool)
hover.tooltips = [("ID", "@id")]

# 输出文件
output_file("hoverable_polygons.html")

# 显示图形
show(p)
