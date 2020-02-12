/**
 *  The celex5_calibration ROS package,
 *  used to calibrate camera parameters for CeleX5-MIPI Event-based Camera.
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


#include "pub_stored_sync_images.h"

PubStoredSyncImages::PubStoredSyncImages(const ros::NodeHandle &nh, bool is_publish_enable, int publish_rate)
    : nh_(nh),
      is_publish_enable_(is_publish_enable),
      publish_rate_(publish_rate) {

  std::string camera1_dir("camera1");
  nh_.param("camera1_dir", camera1_dir, camera1_dir);
  std::string camera1_pattern = camera1_dir + "*.jpg";
  cv::glob(camera1_pattern, image1_filenames_, false);

  std::string camera2_dir("camera2");
  nh_.param("camera2_dir", camera2_dir, camera2_dir);
  std::string camera2_pattern = camera2_dir + "*.jpg";
  cv::glob(camera2_pattern, image2_filenames_, false);

  std::string image1_topic_name("camera1/raw_image");
  nh_.param("image1_topic_name", image1_topic_name, image1_topic_name);
  image1_pub_ = nh_.advertise<sensor_msgs::Image>(image1_topic_name, 1);

  std::string image2_topic_name("camera2/raw_image");
  nh_.param("image2_topic_name", image2_topic_name, image2_topic_name);
  image2_pub_ = nh_.advertise<sensor_msgs::Image>(image2_topic_name, 1);

  p_ddyn_rec_ = std::make_shared<ddynamic_reconfigure::DDynamicReconfigure>(nh_);
  p_ddyn_rec_->registerVariable("is_publish_enable", &is_publish_enable_, "Publish enable.");

  p_main_thread_ = std::make_shared<std::thread>([&]() {
    ros::Rate loop_rate(publish_rate_);
    p_ddyn_rec_->registerVariable<int>("publish_rate", publish_rate_,
                                       [&loop_rate](int new_fps) {
                                         loop_rate = ros::Rate(new_fps);
                                       }, "Publish rate. Unit hz.", 0, 200);

    if (image1_filenames_.size() != image1_filenames_.size()) {
      ROS_ERROR("The each number of sync images set doesn't equal!");
      return;
    }

    int index = 0;
    ros::Time now_stamp = ros::Time::now();
    auto read_file_and_publish =
        [&index, &now_stamp](const ros::Publisher &pub, const std::vector<cv::String> &image_filenames) {
          cv::Mat image = cv::imread(image_filenames.at(index));
          sensor_msgs::ImagePtr image_ptr_msg =
              cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
          image_ptr_msg->header.stamp = now_stamp; // TODO add frame_id
          pub.publish(image_ptr_msg);
        };

    while (ros::ok()) {
      if (is_publish_enable_) {
        if (index >= image1_filenames_.size()) {
          ROS_INFO("Publish all images finished.");
          break;
        }
        now_stamp = ros::Time::now();
        read_file_and_publish(image1_pub_, image1_filenames_);
        read_file_and_publish(image2_pub_, image2_filenames_);
        index++;
      }
      loop_rate.sleep();
    }
  });

  usleep(3000);
  p_ddyn_rec_->publishServicesTopics();
}

PubStoredSyncImages::~PubStoredSyncImages() {
  if (p_main_thread_->joinable()) {
    p_main_thread_->join();
  }
}
