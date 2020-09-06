*[English Version](README.md)*

# CeleX5-ROS

> The ROS package for CeleX™ CeleX5-MIPI Dynamic Vision Sensor.

- [概述](#概述)
- [特性](#特性)
- [编译与运行](#编译与运行)
- [运行样例](#运行样例)
- [仍存在的问题](#仍存在的问题)

## 概述

本仓库提供了[CeleX5-MP](http://www.celepixel.com/#/Samples)系列[Event-based Camera](https://en.wikipedia.org/wiki/Event_camera)（事件相机）功能较为完善的ROS示例。可根据用户需求自由配置输出多路数据（原始Event数据、IMU数据、灰度帧、光流信息等），并提供了`rqt_reconfigure`调参面板以支持动态调参。

*目前仅仅在CeleX5-MIPI设备上测试通过，由于手里没有CeleX5系列的其他产品所以无法测试。*

<img src="assets/Screenshot from 2020-01-22 21-16-52.png" style="zoom:80%;" />

CeleX-5 是一款多功能智能图像传感器，具有一百万像素（分辨率为：1280*800）和片上集成的一些附加功能（如光流）。传感器支持几种不同的输出格式:纯二进制地址事件,具有像素强度信息或定时信息的地址事件。此外,传感器的读出方案可以是异步数据流或同步全帧。输出格式和读出方案的不同组合使该传感器具有很大的灵活性,总共支持 6 种独立的操作模式（但是其中有一种未在sdk中提供相应接口）。为了进一步满足不同应用的要求,传感器还可以配置为 Loop 模式,可以在三种不同模式之间自动切换。

[ROS](https://www.ros.org/)是目前很主流的实验平台，提供了丰富的开发接口和资源。但在[CeleX™官方开源仓库](https://github.com/CelePixel/CeleX5-MIPI)中，关于ROS下的示例非常的不完善：

1. 官方的SDK版本已经更新到了v2.0版本，然而其ROS-Sample支持的版本仍停留在v1.6版本。
2. 官方的ROS-Sample仅为一个简单的示例，只输出一个工作模式下的一种图像，未提供全面且方便的调参功能和界面。

因此，为了更为方便的利用ROS提供的资源，我开发了CeleX5-MIPI系列相机较为完善的ROS下的驱动包。

*建议在使用前，请首先认真阅读[CeleX官方提供的](https://github.com/CelePixel/CeleX5-MIPI/tree/master/Documentation)快速入门手册和API文档的概念部分，对基本名词术语有大概认识。*

## 特性

1. 基于`ddynamic_reconfigure`提供的动态调参机制，提供CeleX5调参的界面接口，覆盖几乎所有参数。用户还可自由配置想要输出的数据和其帧率。

   <img src="assets/Screenshot from 2020-01-22 21-22-20.png" style="zoom:80%;" />

2. 目前已支持CeleX5的五种工作模式：

   | Sensor 的工作模式              | SDK 输出的图像数据类型                                       |
   | ------------------------------ | ------------------------------------------------------------ |
   | Full-Picture Mode              | Full Pic Buffer/Mat                                          |
   | Event Off-Pixel Timestamp Mode | Event Binary Pic Buffer/Mat<br/>Event Denoised Pic Buffer/Mat<br/>Event Count Pic Buffer/Mat<br/>Event Vector<row, col, off-pixel timestamp> |
   | Event In-Pixel Timestamp Mode  | Event Optical-flow Pic Buffer/Mat<br/>Event Binary Pic Buffer/Mat<br/>Event Vector<row, col, in-pixel timestamp, off-pixel<br/>timestamp> |
   | Event Intensity Mode           | Event Binary Pic Buffer/Mat<br/>Event Gray Pic Buffer/Mat<br/>Event Count Pic Buffer/Mat<br/>Event Accumulated Pic Buffer/Mat<br/>Event Superimposed Pic Buffer/Mat<br/>Event Vector<row, col, brightness, polarity, off-pixel<br/>timestamp> |
   | Optical-Flow Mode              | Event Optical-flow Pic Buffer/Mat<br/>Event Optical-flow Direction Pic Buffer/Mat<br/>Event Optical-flow Speed Pic Buffer/Mat<br/>Event Binary Pic Buffer/Mat |

3. 支持**loop_mode**模式，在三种模式之间切换。

4. 将官方SDK的源代码加入到了工程中（因此无需再提前单独编译官方的SDK并放入指定路径），并修复了官方SDK中加载相机参数文件路径固定的问题，并保持之前SDK版本兼容。

5. （提供的nodelet机制可以使得实现在数据量大的情况下实现零拷贝消耗。但目前还有bug，见文末）

## 编译与运行

1. 编译

   ```bash
   mkdir -p ~/celex_ws/src
   cd ~/celex_ws/src
   git clone git@github.com:kehanXue/CeleX5-ROS.git
   git submodule update --init --recursive
   # Or with http: `git clone https://github.com/kehanXue/CeleX5-ROS.git`
   cd ..
   rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
   catkin_make # Or use `catkin build`
   ```

2. 创建udev规则以来解决设备读写权限问题

   ```shell
   cd ~/celex_ws/src/CeleX5-ROS/celex5_ros/scripts
   chmod +x ./create_udev_rules.sh
   ./create_udev_rules.sh
   ```

3. 运行

   首先将相机连接到电脑上。
   
   ```bash
   source ~/celex_ws/devel/setup.bash # Or source setup.zsh when you use zsh
   roslaunch celex5_ros celex5_ros_node.launch
   # In a new Terminal
   rosrun rqt_reconfigure rqt_reconfigure # Open rqt_reconfigure to config
   ```
   
   无报错即正常运行。使用`rostopic list`即可看到发布的话题，使用`rivz`或者`image_view`订阅相对应图像话题即可看到发布的图像。某图像话题中是否发布数据由参数配置、上表的相机工作模式所共同决定。
   
   ```bash
   $ rostopic list
   /celex5_mipi/display/accumulated_img/camera_info
   /celex5_mipi/display/accumulated_img/raw_image
   /celex5_mipi/display/binary_img/camera_info
   /celex5_mipi/display/binary_img/raw_image
   /celex5_mipi/display/count_img/camera_info
   /celex5_mipi/display/count_img/raw_image
   /celex5_mipi/display/denoised_binary_img/camera_info
   /celex5_mipi/display/denoised_binary_img/raw_image
   /celex5_mipi/display/full_frame_img/camera_info
   /celex5_mipi/display/full_frame_img/raw_image
   /celex5_mipi/display/gray_img/camera_info
   /celex5_mipi/display/gray_img/raw_image
   /celex5_mipi/display/in_pixel_img/camera_info
   /celex5_mipi/display/in_pixel_img/raw_image
   /celex5_mipi/display/optical_flow_direction_img/camera_info
   /celex5_mipi/display/optical_flow_direction_img/raw_image
   /celex5_mipi/display/optical_flow_img/camera_info
   /celex5_mipi/display/optical_flow_img/raw_image
   /celex5_mipi/display/optical_flow_speed_img/camera_info
   /celex5_mipi/display/optical_flow_speed_img/raw_image
   /celex5_mipi/display/parameter_descriptions
   /celex5_mipi/display/parameter_updates
   /celex5_mipi/display/superimposed_img/camera_info
   /celex5_mipi/display/superimposed_img/raw_image
   /celex5_mipi/events
   /celex5_mipi/imu_data
   /celex5_mipi/polarity_img/camera_info
   /celex5_mipi/polarity_img/raw_image
   /celex5_mipi/sensor/parameter_descriptions
   /celex5_mipi/sensor/parameter_updates
   /rosout
   /rosout_agg
   ```
   
   其中原始event数据的话题：`/celex5_mipi/events`
   
   imu数据的话题（注意由于CeleX5原始imu数据获取的方式，导致imu话题与ROS中标准的IMU话题的消息类型不同）：`/celex5_mipi/imu_data`
   
   通过`image_view`查看某一图像：
   
   ```bash
   rosrun image_view image_view image:=/celex5_mipi/display/binary_img/raw_image
   ```

## 运行样例

通过上述步骤成功运行后，通过在`rqt_reconfigure`调整参数进行使用了：

1. Event Intensity Mode下获取带极性的Event frame和Superimposed frame（当前二值图像叠加的累计灰度帧上）：

   <img src="assets/Screenshot from 2020-01-31 23-57-10.png" style="zoom:100%;" />

2. Optical Flow下，整体光流帧、光流方向信息帧与光流速度信息帧：

   <img src="assets/Screenshot from 2020-01-22 21-20-36.png" style="zoom:80%;" />

3. 工作在loop mode下（在三种工作模式中循环，图片从左到右、从上到下：原始二值frame、光流frame和传统图像灰度frame）：

   <img src="assets/Screenshot from 2020-01-22 21-16-52.png" style="zoom:80%;" />

## 仍存在的问题

1. 实现的[Nodelet](http://wiki.ros.org/nodelet)接口仍存在问题（不过ros node版本是完全正常工作的）。在通过nodelet加载相机的参数文件的过程中，解析xml文件会出现乱码。调了好久都没找到问题，如果大家提供解决建议的话我会非常感谢。
2. 关于文档中的`Multi_Read_Optical_Flow_Mode`模式，未在SDK中找到相关接口。// TODO
3. 暂未添加生成FPN的功能，请仍使用CeleX官方提供的GUI Demo进行FPN的生成（其Linux下的Demo运行可能会直接报段错误，Windows下的较为稳定一些）。
4. 暂未提供CeleX SDK里的有关录制bin文件的功能，但ROS下我们可以使用ROS bag来进行录制。
