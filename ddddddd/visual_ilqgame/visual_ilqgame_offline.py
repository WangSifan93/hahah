import argparse
import json
from bokeh.plotting import show, ColumnDataSource
import numpy as np
from bokeh.embed import json_item
import os
from visual_ilqgamne_layout import create_fig_default
from visual_ilqgame import visual_ilqgame
from visual_files import get_files

parser = argparse.ArgumentParser("visual_pnc_offline")
parser.add_argument("file")

def file_visual(file: str):
    file_data = ""
    with open(file, "r") as f:
        file_data = f.read()
    visual_data = json.loads(file_data)
    xy_fig, heading_fig, velocity_fig, wheel_angle_fig, acceleration_fig,layout_fig = create_fig_default(visual_data)
    visual_ilqgame(visual_data, xy_fig, heading_fig, velocity_fig, wheel_angle_fig, acceleration_fig)
    return layout_fig
    
def file_show(file: str):
    layout_fig = file_visual(file)
    show(layout_fig)

def dir_show(folder: str):
    def visual_cb(single_file):
        layout_fig = file_visual(single_file)
        return json.dumps(json_item(layout_fig))

    app = get_files(folder, visual_cb)
    app.run(debug=True, port=10001)

if __name__ == "__main__":
    args = parser.parse_args()
    filename = args.file
    if os.path.isdir(filename):
        dir_show(filename)
    else:
        file_show(filename)