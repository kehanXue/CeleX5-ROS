*[English Version](README.md)*

# CeleX5-MIPI Intrinsics Calibration Demo

> Calibrate CeleX5-MIPI camera **intrinsics** with `camera-calibration`.

- [标定原理](#标定原理)
- [运行](#运行)
- [结果示例](#结果示例)

我们使用一个频闪的棋盘格标定板（可以用仓库中的这个[工具](../pattern/)来生成）来对CeleX5-MIPI事件相机进行内参标定。

## 标定原理

利用闪烁的棋盘格，控制CeleX5-MIPI相机生成棋盘格状的Event frame，然后通过传统的内参标定方法即可。这里采用ROS中集成的[`camera-calibration`](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration)标定工具进行标定，内部采用张正友标定法，标定结果的保存包含yaml文件、txt文件和用于最终标定结果的图像帧。

## 运行

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

## 结果示例

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
