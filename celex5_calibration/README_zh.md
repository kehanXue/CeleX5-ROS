*[English Version](README.md)*

# celex5_calibration

>  The ROS package used for calibrating CeleX™ CeleX5-MIPI Dynamic Vision Sensor.

<img src="assets/Screenshot from 2020-02-12 21-02-32.png" width="600" />

本仓库提供了有关于`celex5_calibration`基于Events数据进行标定相关的工具和教程。包括：

- 标定板生成（目前仅支持闪烁的棋盘格，其他类型待加入）
- 内参标定
- 与其他相机进行外参标定（使用Kalibr）
- 同步收集与定制发布标定所需的图像数据
- 与传统相机进行时间戳对齐（TODO，目前存在问题）等工具

## 使用`celex5_calibration`

提供了一系列基于Events数据进行相机参数标定的办法。

- 标定工具的生成

  运行：

  ```bash
  rosrun celex5_ros pattern_generator_node
  # In a new Terminal
  rosrun rqt_reconfigure rqt_reconfigure #  Open rqt_reconfigure to config
  ```

  更多细节：[链接](src/pattern)

- 基于ROS中的[camera_calibration](http://wiki.ros.org/camera_calibration/Tutorials)工具包进行的内参标定。（如果要标定与其他相机的外参，ROS中的这个工具仅支持分辨率一样的...）

  安装：

  ```bash
  sudo apt install ros-$ROS_DISTRO-camera-calibration
  ```

  CeleX5-MIPI进行基于该方法进行标定的具体过程：[链接](intrinsics_extrinsics/pkg_camera_calibration)

- 基于[Kalibr](https://github.com/ethz-asl/kalibr)的标定，支持同时进行内参、与传统相机的外参标定。基于Event数据。

  更多细节：[链接](src/intrinsics_extrinsics/kalibr)

  同时提供了收集标定数据的一系列工具。

- 与其他相机进行时间戳的标定。

  *TODO*，目前仍存在问题。[链接](src/temporal)

## 仍存在问题

**celex5_calibration**

-  关于与其他相机进行时间戳标定的相关功能还未完成。