# app.py
import json
from flask import Flask, jsonify, request, render_template
import os
import glob
import re
from bokeh.resources import CDN


def get_sorted_files(folder_path: str):
    if not os.path.isdir(folder_path):
        raise FileNotFoundError(f"Folder not found: {folder_path}")
    
    json_files = glob.glob(os.path.join(folder_path, "*.json"))
    files_list = [os.path.basename(file) for file in json_files]
    
    def sort_key(x):
        name_part = x.split('.')[0]
        match = re.search(r'\d+', name_part)
        return int(match.group()) if match else 0
    
    files_list.sort(key=sort_key)
    return files_list


def get_files(path: str, visual_callback):
    """创建Flask应用提供文件浏览服务"""
    app = Flask(__name__, static_folder="frontend")
    app.jinja_options["variable_start_string"] = "[["
    app.jinja_options["variable_end_string"] = "]]"

    @app.route("/")
    def index():
        files_json = json.dumps(get_sorted_files(path))
        return render_template(
            "files_viewer.html",
            resources=CDN.render(),
            inputFolder=path,
            files=files_json,
        )
    @app.route("/api/file", methods=["POST"])
    def get_file_msg():
        file_name = request.form.get("file_name", "")
        if not os.path.isfile(file_name):
            return jsonify({"error": "File not found"}), 404
        return visual_callback(file_name)
    return app