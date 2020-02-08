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
 */

#include <opencv-3.3.1-dev/opencv2/core/mat.hpp>
#include <utility>
#include "camera_publisher.h"

celex5_ros::CameraPublisher::CameraPublisher(std::string image_name,
                                             int buffer_length,
                                             const ros::NodeHandle &nh)
    : image_name_(std::move(image_name)),
      buffer_length_(buffer_length),
      nh_(nh) {
  image_pub_ = nh_.advertise<sensor_msgs::Image>(image_name_, buffer_length_);
  camera_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(image_name_ + "/camera_info", buffer_length_);

  std::string camera_cfg_path("./");
  nh_.param("sensor_cfg_file_dir", camera_cfg_path, camera_cfg_path);
  parameters_file_url_ = "file://" + camera_cfg_path + "celex5_events_parameters.yaml";
  p_camera_info_ =
      std::make_shared<camera_info_manager::CameraInfoManager>(nh_, "narrow_stereo",
                                                               parameters_file_url_);
}

celex5_ros::CameraPublisher::CameraPublisher(std::string image_name,
                                             int buffer_length,
                                             std::string parameters_file_url,
                                             const ros::NodeHandle &nh)
    : image_name_(std::move(image_name)),
      buffer_length_(buffer_length),
      parameters_file_url_(std::move(parameters_file_url)),
      nh_(nh) {
  image_pub_ = nh_.advertise<sensor_msgs::Image>(image_name_, buffer_length_);
  camera_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(image_name_ + "/camera_info", buffer_length_);

  p_camera_info_ =
      std::make_shared<camera_info_manager::CameraInfoManager>(nh_, "narrow_stereo",
                                                               parameters_file_url_);
}

celex5_ros::CameraPublisher::~CameraPublisher() = default;

void celex5_ros::CameraPublisher::Publish(const cv::Mat &image,
                                          const std::string &encoding,
                                          const std::string &frame_id) {
  if (IsSubscribed()) {
    ros::Time now_time = ros::Time::now();

    sensor_msgs::ImagePtr image_ptr_msg =
        cv_bridge::CvImage(std_msgs::Header(), encoding, image).toImageMsg();
    image_ptr_msg->header.stamp = now_time;
    image_ptr_msg->header.frame_id = frame_id;

    sensor_msgs::CameraInfoPtr
        camera_info_ptr_msg = boost::make_shared<sensor_msgs::CameraInfo>(p_camera_info_->getCameraInfo());
    camera_info_ptr_msg->header.stamp = now_time;
    camera_info_ptr_msg->header.frame_id = frame_id;

    image_pub_.publish(image_ptr_msg);
    camera_info_pub_.publish(camera_info_ptr_msg);
  }
}

bool celex5_ros::CameraPublisher::IsSubscribed() {
  return image_pub_.getNumSubscribers() > 0 || camera_info_pub_.getNumSubscribers() > 0;
}
