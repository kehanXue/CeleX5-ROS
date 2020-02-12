*[中文版本](README_zh.md)*

# celex5_calibration

> The ROS package used for calibrating CeleX™ CeleX5-MIPI Dynamic Vision Sensor.

<img src="assets/Screenshot from 2020-02-12 21-02-32.png" width="600" />

This repository provides tools and tutorials related to the calibration of CeleX5-MIPI, based on events data. Including:

- calibration board generation (currently only support the blinking checkerboard, other types to be added)
- Intrinsics calibration
- Extrinsics calibration with another traditional frame camera (using [Kalibr](https://github.com/ethz-asl/kalibr))
- Synchronous data collection and publishing of image data required for calibration
- Time stamp alignment with traditional camera (*TODO*, existing problems)

## Run `celex5_calibration`:

A series of methods and tools for camera parameter calibration based on events data are provided.

- Pattern generator (blinking chessboard). [More details](src/pattern)

  Run:

  ```bash
  rosrun celex5_ros pattern_generator_node
  # In a new Terminal
  rosrun rqt_reconfigure rqt_reconfigure #  Open rqt_reconfigure to config
  ```

- Intrinsics calibration based on [camera_calibration](http://wiki.ros.org/camera_calibration/Tutorials) toolkit in ROS. (if you want to calibrate the external parameters of other cameras, this tool in ROS only supports the same resolution...) 

  Install:

  ```bash
  sudo apt install ros-$ROS_DISTRO-camera-calibration
  ```

  The detailed tutorials: [link](src/intrinsics_extrinsics/pkg_camera_calibration)

- Based on [Kalibr](https://github.com/ethz-asl/kalibr) toolkit, which supports both intrinsics calibration and extrinsics calibration with another (traditional or event-based) cameras, in different resolutions. Event data based method. 

  The detailed tutorials: [link](src/intrinsics_extrinsics/kalibr)

  Also provides a series of tools to collect calibration data of CeleX5-MIPI Camera.

- Time stamp calibration with another camera.

  *TODO*, existing problems. [link](src/temporal)

## Known Issues

**celex5_calibration**

- The time stamp calibration with another camera.