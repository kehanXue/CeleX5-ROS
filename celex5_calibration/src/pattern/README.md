*[中文版本](README_zh.md)*

# Calibration Pattern Generator

> ROS node for generating **Blinking-Chessboard** calibration pattern for Event-based Camera(Dynamic Vision Sensor).

- [Overview](#overview)
- [Build and Run](#build-and-run)
- [CeleX5-MIPI Intrinsics Calibration Demo](#celex5-mipi-intrinsics-calibration-demo)

## Overview

An ROS node is provided to generate a **blinking chessboard** for calibrating the Event-based Camera(Dynamic Vision Sensor). It provides the support of `rqt_reconfigure` to control the size and blinking frequency of the chessboard. And provide to save it in ` .avi` video format.

<img src="assets/Screenshot from 2020-02-04 22-09-31.png" height="400" alt="Chessboard" />

<img src="assets/Screenshot from 2020-02-04 22-34-03.png" style="zoom:80%;" />

## Build and Run

1. Build

   If you have not yet compiled the entire repository, clone the entire repository and compile it (skip this step if you have already).

   ```bash
   mkdir -p ~/celex_ws/src
   cd ~/celex_ws/src
   git clone git@github.com:kehanXue/CeleX5-ROS.git
   # Or with http: `git clone https://github.com/kehanXue/CeleX5-ROS.git`
   cd ..
   rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
   catkin_make # Or use `catkin build`
   ```

2. Run

   ```bash
   source ~/celex_ws/devel/setup.bash # Or source setup.zsh when you use zsh
   rosrun celex5_ros pattern_generator_node
   # In a new Terminal
   rosrun rqt_reconfigure rqt_reconfigure #  Open rqt_reconfigure to config
   ```
   
   Normal operation without error. You can see that a window will pop up showing the blinking chessboard. At this time, you can adjust the parameters of the blinking calibration board through the `rqt_reconfigure` panel:

   - `square_size`: the length of each side of the squares in a chessboard. In pixels.
   
   - `row_corners_num`和`col_corners_num`: the number of interior points of the rows and columns of the generated chessboard.
   
   - `blinking_fps`: the blinking frequency / frame rate of the chessboard. It is a cycle before one flash and the next flash.
   
   - `static_board`: controls whether the checkerboard blinks.
   
   - `record_video`: click start to generate the video of blinking chessboard, and click (uncheck) again to stop recording the video. The frequency of the chessboard blinking in the video is consistent with the current `blinking_fps` parameter. Note that adjusting the `blinking_fps` in the process of generating video is invalid for the generated video, and **doesn't adjust the size of the checkerboard during the video generation processing!**
   
      The video will be generated under the current application running path:
   
      <img src="assets/Screenshot from 2020-02-04 22-11-21.png" style="zoom:80%;" />
   

## CeleX5-MIPI Intrinsics Calibration Demo

[CeleX5-MIPI Intrinsics Calibration Demo](../intrinsics)

<img src="assets/Screenshot from 2020-02-04 23-11-53.png" style="zoom:80%;" />