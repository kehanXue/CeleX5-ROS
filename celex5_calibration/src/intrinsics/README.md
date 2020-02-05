*[中文版本](README_zh.md)*

# CeleX5-MIPI Intrinsics Calibration Demo

> Calibrate CeleX5-MIPI camera **intrinsics** with `camera-calibration`.

- [Calibration principle](#calibration-principle)
- [Running](#running)
- [Example results](#example-results)  

We use the blinking chessboard calibration board (generated with [pattern-generator](../pattern/) in this repository) to calibrate the CeleX™ CeleX5-MIPI Event-based Camera.

## **Calibration principle**

By using the blinking chessboard, the CeleX5-MIPI camera can generate a chessboard like event frame, and then the traditional Camera Intrinsics calibration method can be used. Here [`camera calibration`](http://wiki.ros.org/camera'calibration/tutorials/monocularcalibration) calibration tool integrated in ROS is used for calibration, and Zhang Zhengyou-Calibration method is used internally. The calibration results are saved with .yaml file, .txt file and image frame for final calibration results.

## Running

1. First install `camera-calibration`.

   ```bash
   sudo apt install ros-kinetic-camera-calibration
   ```

2. Run `celex5_ros_node`:

   ```bash
   roslaunch celex5_ros celex5_ros_node.launch
   ```

   Open `rqt_reconfigure` panel, under **Event Off-Pixel Timestamp Mode** sensor mode, only keep `/celex5_mipi/display/binary_img` Image topic publishing messages, close other Image topics to reduce running consumption. Keep `/celex5_mipi/display/binary_img` publishing frequency at 20 fps, too high can cause a delay in the calibration procedure.

3. Launch `camera-calibration`.  For the detailed meaning and more usage of each parameter, please go to the official document of `camera-calibration` in the above link. 

   ```bash
   rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.0212 image:=/celex5_mipi/display/binary_img
   ```

   The meaning of parameters:

   - `--size`: The number of inner points (that is, the number of squares in the outer circle with the longest side out - 1), width*height. If this parameter is not set correctly, the calibration procedure will be found to be unresponsive in the following calibration steps.
   - `--square`: the physical length of each side of the squares in the calibration board. The unit is m.
   - `image:=`: Image topic name.

   After successful operation, the following windows will appear:

   <img src="assets/Screenshot from 2020-02-04 23-11-53.png" style="zoom:80%;" />

   Move the blinking chessboard screen, and make the X, Y, Size and Skew  progress bars in the upper right corner full and green as much as  possible. 

   Reference the [official document of `camera-calibration`](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration): 

   - checkerboard on the camera's left, right, top and bottom of field of view
   - X bar - left/right in field of view
   - Y bar - top/bottom in field of view
   - Size bar - toward/away and tilt from the camera
   - checkerboard filling the whole field of view
   - checkerboard tilted to the left, right, top and bottom (Skew)
   

*Tips: In the process of moving, the blinking chessboard pattern can be suspended, so as to avoid corner detection when the dragging is serious. After fixing the position, continue to make the checkerboard blinking to obtain a high-quality Event frame* 

When you see the **CALIBRATE** button turns green, click calculate to get the internal parameter calibration result. Then click the **SAVE** button to save the calibration results. It will prompt you to save the path, which is a compressed package. After decompression, the image frame used in the calibration process and calibration result file (one .yaml file and one .txt file) are stored.

## Example results

This is the whole calibration output:

<img src="assets/Screenshot from 2020-02-04 23-28-25.png" style="zoom:100%;" />

ost.yaml:

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

The sample image frame used in the calibration:

<img src="assets/left-0013.png" height="300" />

<img src="assets/left-0021.png" height="300" />

<img src="assets/left-0006.png" height="300" />