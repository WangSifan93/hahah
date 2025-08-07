# planning_debug_tool

使用下方代码安装环境
```
pip install -r requirements.txt
```

在运行可视化代码前，先修改数据路径，需要可视化数据默认保存在metrics

## mcts可视化代码说明：
mcts_st_show : 解包pack文件，可视化每一帧的st图
mcts_tree_show : 解析mcts_debug.json文件，绘制单帧推理的mcts树及第一层子结点的reward和visits随探索次数的变化

