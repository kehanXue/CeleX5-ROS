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

使用该标定板进行CeleX™ CeleX5-MIPI事件相机的标定。

**标定原理：**

利用闪烁的棋盘格，控制CeleX5-MIPI相机生成棋盘格状的Event frame，然后通过传统的内参标定方法即可。这里采用ROS中集成的[`camera-calibration`](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration)标定工具进行标定，内部采用张正友标定法，标定结果的保存包含yaml文件、txt文件和用于最终标定结果的图像帧。

1. 首先安装`camera-calibration`。

   ```bash
   sudo apt install ros-kinetic-camera-calibration
   ```

2. 运行CeleX5-MIPI ROS节点：

   ```bash
   roslaunch celex5_ros celex5_ros_node.launch
   ```

   打开`rqt_reconfigure`，在**Event Off-Pixel Timestamp Mode**模式下，只保留`/celex5_mipi/display/binary_img`话题的输出，将其他的无关输出关闭。将`/celex5_mipi/display/binary_img`的帧率保持在20fps即可。过高可能会造成标定程序中的延迟。

3. 启动`camera-calibration`。各参数详细含义和更多用法请移步上面链接中的`camera-calibration`官方文档。

   ```bash
   rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.0212 image:=/celex5_mipi/display/binary_img
   ```

   其中参数含义：

   - `--size`：内点的数目（也就是出去边长最外圈方格数-1）的宽乘高。如果这个参数设置不对的话会在下面的标定步骤中发现标定程序毫无反应。
   - `--square`：标定板中的正方形的物理边长。单位为m。
   - `image:=`：监听的图像的话题名称。

   运行成功后会有以下窗口：

   <img src="assets/Screenshot from 2020-02-04 23-11-53.png" style="zoom:80%;" />

   移动闪烁棋盘格的屏幕，将右上角的X、Y、Size、Skew四个进度条都尽力变满变绿。X、Y顾名思义就是棋盘格在图像中的X方向和Y方向，Size是远近（棋盘格的大小）、Skew向上下左右的倾斜。具体详见`camera-calibration`官方文档。

   *Tips：移动过程中可以讲棋盘格图案暂停，从而避免在拖影严重时仍然进行角点检测。在固定好位置后再继续使得棋盘格闪烁，获得质量比较高的Event frame*

   当你看到**CALIBRATE**按钮变绿后，点击经过计算即可获得内参标定结果。然后点击**SAVE**按钮即可保存标定结果。它会提示你保存路径，是个压缩包。解压后里面存储的有标定过程中用到的图像Frame和标定结果文件（一个yaml一个txt）。

   结果示例：

   <img src="assets/Screenshot from 2020-02-04 23-28-25.png" style="zoom:100%;" />

   ost.yaml：

   ```yaml
   image_width: 1280
   image_height: 800
   camera_name: narrow_stereo
   camera_matrix:
     rows: 3
     cols: 3
     data: [1687.514919, 0.000000, 676.131784, 0.000000, 1694.799737, 417.348889, 0.000000, 0.000000, 1.000000]
   distortion_model: plumb_bob
   distortion_coefficients:
     rows: 1
     cols: 5
     data: [-0.089225, -0.433622, 0.003022, -0.001306, 0.000000]
   rectification_matrix:
     rows: 3
     cols: 3
     data: [1.000000, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000, 0.000000, 0.000000, 1.000000]
   projection_matrix:
     rows: 3
     cols: 4
     data: [1647.456299, 0.000000, 677.701102, 0.000000, 0.000000, 1683.463867, 418.507924, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000]
   ```

   过程中用到的图片样例：

   <img src="assets/left-0013.png" height="300" />

   <img src="assets/left-0021.png" height="300" />

   <img src="assets/left-0006.png" height="300" />
