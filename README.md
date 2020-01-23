*[中文版本](README_zh.md)*

# CeleX5-ROS

> The ROS package for CeleX™ CeleX5-MIPI Dynamic Vision Sensor.

- [Overview](#overview)
- [Features](#features)
- [Build and Run](#build-and-run)
- [Running Demo](#running-demo)
- [Known Issues](#known-issues)

## Overview

<img src="assets/242069421.jpg" height="300" alt="CeleX5-MIPI"/>

This repository provides a complete ROS example of the [CeleX5-MP](http://www.celepixel.com/#/Samples) series [Event-based Camera](https://en.wikipedia.org/wiki/Event_camera)). You can freely configure and output multiple channels of data (raw event data, IMU data, grayscale frames, optical flow information, etc.) according to your needs, and provide the `rqt_reconfigure` panel to support dynamic parameter configuration.

*Currently only tested on CeleX5-MIPI products. Other series of CeleX5 are not tested because I have no such equipments.*

<img src="assets/Screenshot from 2020-01-22 21-16-52.png" style="zoom:80%;" />

CeleX™ is a family of smart image sensor specially designed for machine vision. Each pixel in CeleX™ sensor can individually monitor the relative change in light intensity and report an event if it reaches a certain threshold. Asynchronous row and column arbitration circuits process the pixel requests and make sure only one request is granted at a time in fairly manner when they received multiple simultaneous requests. The output of the sensor is not a frame, but a stream of asynchronous digital events. The speed of the sensor is not limited by any traditional concept such as exposure time and frame rate. It can detect fast motion which is traditionally captured by expensive, high speed cameras running at thousands of frames per second, but with drastic reduced amount of data.

CeleX-5 is a multifunctional smart image sensor with **1Mega-pixels**(1280*800) and some additional features integrating on-chip (such as on-chip optical-flow). The sensor supports several different output formats: pure binary address-events, address-events with either pixel intensity information or timing information. In addition, the readout scheme of the sensor could either be asynchronous data stream or synchronous full frames. Different combinations of the output format and readout scheme lead to great flexibility of this sensor, which supports **6 separate operation modes** in total (but one of them doesn't provide corresponding interface in SDK). To further meet the requirements of different applications, the sensor could also be configured into a **loop-mode**, in which it could automatically switch among three separate modes.

[ROS](https://www.ros.org/) is a very popular experimental platform, which provides rich development interfaces and resources. But in the [official open source repository](https://github.com/CelePixel/CeleX5-MIPI) of CeleX ™, the examples under ROS are very incomplete:

1. The official SDK version has been updated to v2.0, however the version supported by its ROS-Sample remains at v1.6.
2. The official ROS-Sample is just a simple example. It only outputs an image in a working mode. It does not provide a comprehensive and convenient parameter configuration function and interface.

Therefore, I have developed a more comprehensive CeleX5-ROS package v1.0 in the past week.

*It is recommended that you read the concept section of the [CeleX_SDK_Getting_Started_Guide](https://github.com/CelePixel/CeleX5-MIPI/blob/master/Documentation/CeleX_SDK_Getting_Started_Guide_EN.pdf) and [CeleX5_SDK_Reference](https://github.com/CelePixel/CeleX5-MIPI/blob/master/Documentation/CeleX5_SDK_Reference_EN.pdf) documentation provided by CeleX carefully before using it, and have a general understanding of basic terminology.*

## Features

1. Based on the dynamic parameter reconfiguration mechanism provided by `ddynamic_reconfigure`, this ROS sample provides the interface interface of CeleX5 parameters configuration, covering almost all parameters. You can also freely configure the data you want to output and its fps.

   <img src="assets/Screenshot from 2020-01-22 21-22-20.png" style="zoom:80%;" />

2. Currently supports five working modes of CeleX5-MIPI:

   | Sensor Mode                    | Data Types Output by SDK                                     |
   | ------------------------------ | ------------------------------------------------------------ |
   | Full-Picture Mode              | Full Pic Buffer/Mat (traditional grayscale image frames)     |
   | Event Off-Pixel Timestamp Mode | Event Binary Pic Buffer/Mat<br/>Event Denoised Pic Buffer/Mat<br/>Event Count Pic Buffer/Mat<br/>Event Vector<row, col, off-pixel timestamp> |
   | Event In-Pixel Timestamp Mode  | Event Optical-flow Pic Buffer/Mat<br/>Event Binary Pic Buffer/Mat<br/>Event Vector<row, col, in-pixel timestamp, off-pixel<br/>timestamp> |
   | Event Intensity Mode           | Event Binary Pic Buffer/Mat<br/>Event Gray Pic Buffer/Mat<br/>Event Count Pic Buffer/Mat<br/>Event Accumulated Pic Buffer/Mat<br/>Event Superimposed Pic Buffer/Mat<br/>Event Vector<row, col, brightness, polarity, off-pixel<br/>timestamp> |
   | Optical-Flow Mode              | Event Optical-flow Pic Buffer/Mat<br/>Event Optical-flow Direction Pic Buffer/Mat<br/>Event Optical-flow Speed Pic Buffer/Mat<br/>Event Binary Pic Buffer/Mat |

3. Support **loop_mode** mode, which can switch between three modes.

4. The source code of the official SDK was added to the project (so there is no need to separately compile the official SDK in advance and put it in the specified path), and fixed the problem of the fixed path to load the camera cfg file (cfg_mp/cfg_mp_wire) in the official SDK, and kept the previous SDK version compatible.

5. (The provided `nodelet` version can make the implementation achieve zero-copy consumption in the case of large amount of data. But there are still bugs, see the end of the article.)

## Build and Run

1. Build

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

   First plug the sensor into your computer.
   
   ```bash
   source ~/celex_ws/devel/setup.bash # Or source setup.zsh when you use zsh
   roslaunch celex5_ros celex5_ros_node.launch
   rosrun rqt_reconfigure rqt_reconfigure # Open rqt_reconfigure to config
   ```
   
   If the program runs without error, you have run successfully. Use `rostopic list` to see published topics, and use` rivz` or `image_view` to subscribe to corresponding image topics to see published images. Whether to publish data to an image topic is determined by both the whether display parameter configuration and the sensor current working mode.
   
   ```bash
   $ rostopic list 
   /celex5_mipi/display_accumulated_img
   /celex5_mipi/display_binary_img
   /celex5_mipi/display_colored_optical_flow_direction_img
   /celex5_mipi/display_colored_optical_flow_img
   /celex5_mipi/display_colored_optical_flow_speed_img
   /celex5_mipi/display_count_img
   /celex5_mipi/display_denoised_binary_img
   /celex5_mipi/display_full_frame_img
   /celex5_mipi/display_gray_img
   /celex5_mipi/display_in_pixel_img
   /celex5_mipi/display_optical_flow_direction_img
   /celex5_mipi/display_optical_flow_img
   /celex5_mipi/display_optical_flow_speed_img
   /celex5_mipi/display_superimposed_img
   /celex5_mipi/events
   /celex5_mipi/imu_data
   /celex5_mipi/parameter_descriptions
   /celex5_mipi/parameter_updates
   /celex5_mipi/polarity_img
   /rosout
   /rosout_agg
   ```
   
   Topic of raw events data: `/celex5_mipi/events`
   
   Topic of imu data: `/celex5_mipi/imu_data` . *Note that the ROS message type of imu topic is different from the standard IMU message type (sensor_msgs/Imu) in ROS due to the way of obtaining the raw imu data of CeleX5*
   
   View an image via `image_view`:
   
   ```bash
   rosrun image_view image_view image:=/celex5_mipi/display_binary_img
   ```

## Running Demo

After successfully running through the above steps, it was used by configuring parameters in `rqt_reconfigure`.

1. Obtain the **Polarized Event Frame** and **Superimposed Frame** (superimposes event binary picture onto event accumulated picture) in the **Event Intensity Mode**: 

   <img src="assets/Screenshot from 2020-01-23 13-58-40-1579759278532.png" style="zoom:100%;" />
   
2. The **Full-frame Optical-flow Frame,  the Speed Frame and Direction Frame** (each pixel calculated on the optical flow raw frame) when in **Optical-flow Mode**.

   <img src="assets/Screenshot from 2020-01-22 21-20-36.png" style="zoom:80%;" />

3. Work in loop mode, loop in three working modes (**Full-Picture Mode, Event Off-Pixel Timestamp Mode, Optical-flow Mode**). Pictures from left to right, top to bottom: original **Binary Event Frame, Optical-flow Frame and Traditional Image Gray Frame)**:

   <img src="assets/Screenshot from 2020-01-22 21-16-52-1579704106897.png" style="zoom:80%;" />

## Known Issues

1. The implementation of the [Nodelet](http://wiki.ros.org/nodelet) interface still has problems (the ros node version has no problem). During the process of loading the camera's parameter file through the nodelet, **parsed xml files will be garbled**. I haven't found the problem for a long time. If you provide suggestions, I will be very grateful.
2. Regarding the `Multi_Read_Optical_Flow_Mode` mode in the document, no relevant interface was found in the SDK. 
3. The function of generating FPN has not been added yet, please still use the official GUI Demo provided by CeleX™ to generate FPN (Demo running under Linux may report `Segmentation fault (core dumped)` errors directly. It is more stable under Windows).
4. The function of recording bin files use CeleX™ SDK is not provided for now, but under ROS we can use ROS bag for recording.
5. How to control the frequency of data release needs to become more scientific. In the case of a large amount of Event triggers, there will be a significant display delay.