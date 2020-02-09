/**
 *  The ROS package for CeleX^TM CeleX5-MIPI Dynamic Vision Sensor.
 *
 *  Copyright (C) 2020  Kehan.Xue<kehan.xue@gmail.com>
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  ------------------------------------------------------------
 *  Create this class because of the Segfault of image_transport.
 *  gdb outputs:
 *  ```
 *  ROS__GI___libc_free (mem=0x3e646e657065645f) at malloc.c:2951
 *  2951	malloc.c: No such file or directory.
 *  ```
 *  Same problem can be found:
 *  https://answers.ros.org/question/275710/the-node-with-pluginlib-can-not-run-in-custom-boost-situation/
 *  https://stackoverflow.com/questions/38429036/not-able-to-run-image-transport-from-cpp-code-segmentation-fault-core-dumped
 *  https://answers.ros.org/question/275615/image_transportimagetransport-segfault/
 *  But my sysytem libboost version is 1.58, which is same to the pluginlib's dependence,
 *  and there seems to `link to different version libboost` doesn't happened.
 *  Still debuging...
 */

#ifndef CELEX5_ROS_SRC_BEAN_CAMERA_PUBLISHER_H_
#define CELEX5_ROS_SRC_BEAN_CAMERA_PUBLISHER_H_

#include <string>
#include <utility>

#include <ros/ros.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

namespace celex5_ros {
class CameraPublisher {
 public:
  explicit CameraPublisher(const ros::NodeHandle &nh,
                           int buffer_length = 10);
  CameraPublisher(const ros::NodeHandle &nh,
                  std::string parameters_file_url,
                  int buffer_length = 10);
  virtual ~CameraPublisher();
  void Publish(const cv::Mat &image,
               const std::string &encoding,
               const std::string &frame_id = "");
  bool IsSubscribed();

 private:
  // std::string image_name_;
  int buffer_length_;

  std::string parameters_file_url_;

  ros::NodeHandle nh_;
  ros::Publisher image_pub_;
  ros::Publisher camera_info_pub_;

  std::shared_ptr<camera_info_manager::CameraInfoManager> p_camera_info_;
};
};

#endif //CELEX5_ROS_SRC_BEAN_CAMERA_PUBLISHER_H_
