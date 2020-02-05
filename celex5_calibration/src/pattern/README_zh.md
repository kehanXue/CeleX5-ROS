*[English Version](README.md)*

# Calibration Pattern Generator

> ROS node for generating **Blinking-Chessboard** calibration pattern for Event-based Camera(Dynamic Vision Sensor).

- [概述](#概述)
- [编译与运行](#编译与运行)
- [CeleX5-MIPI的内参标定样例](#celex5-mipi的内参标定样例)

## 概述

提供了一个ROS节点，用于生成标定相机的闪烁的棋盘格。提供了`rqt_reconfigure`的支持以来控制棋盘格的大小、闪烁的频率。并提供将其保存成.avi视频格式。

<img src="assets/Screenshot from 2020-02-04 22-09-31.png" height="400" alt="Chessboard" />

<img src="assets/Screenshot from 2020-02-04 22-34-03.png" style="zoom:80%;" />

## 编译与运行

1. 编译

   如果还未编译整个仓库，请克隆整个仓库并进行编译（如果已经进行过，请略过这一步）。

   ```bash
   mkdir -p ~/celex_ws/src
   cd ~/celex_ws/src
   git clone git@github.com:kehanXue/CeleX5-ROS.git
   # Or with http: `git clone https://github.com/kehanXue/CeleX5-ROS.git`
   cd ..
   rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
   catkin_make # Or use `catkin build`
   ```

2. 运行

   ```bash
   source ~/celex_ws/devel/setup.bash # Or source setup.zsh when you use zsh
   rosrun celex5_ros pattern_generator_node
   # In a new Terminal
   rosrun rqt_reconfigure rqt_reconfigure #  Open rqt_reconfigure to config
   ```
   
   无报错即正常运行。你可以看到会弹出一个窗口显示正在闪烁的棋盘格标定板。此时，你可以通过`rqt_reconfigure`面板来调整标定板的参数：

   - `square_size`：棋盘格中每个格子方块的边长。单位为像素。
   
   - `row_corners_num`和`col_corners_num`：生成的棋盘格的行和列的内点数目。
   
   - `blinking_fps`：棋盘格闪烁的频率/帧率。一次闪烁与下次闪烁开始前为一个周期。
   
   - `static_board`：控制棋盘格是否闪烁。
   
   - `record_video`：点击开始生成闪烁棋盘格的视频，再次点击（不勾选）停止录制视频。视频中棋盘格闪烁的频率与当前`blinking_fps`参数一致。注意，在生成视频过程中调整`blinking_fps`对生成的视频是无效的，且视频生成过程中不要调整棋盘格的大小！
   
      视频会生成在当前运行的路径下：
   
      <img src="assets/Screenshot from 2020-02-04 22-11-21.png" style="zoom:80%;" />
   
     

## CeleX5-MIPI的内参标定样例

[CeleX5-MIPI的内参标定样例](../intrinsics)

<img src="assets/Screenshot from 2020-02-04 23-11-53.png" style="zoom:80%;" />