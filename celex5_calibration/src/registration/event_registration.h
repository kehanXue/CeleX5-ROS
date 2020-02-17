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

#ifndef CELEX5_CALIBRATION_SRC_REGISTRATION_EVENT_REGISTRATION_H_
#define CELEX5_CALIBRATION_SRC_REGISTRATION_EVENT_REGISTRATION_H_

#include <boost/thread.hpp>
#include <thread>
#include <condition_variable>
#include <mutex>

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <celex5_msgs/EventVector.h>
#include <cv_bridge/cv_bridge.h>

#include <Eigen/Geometry>

#include <opencv2/opencv.hpp>

class EventRegistration {
 public:
  explicit EventRegistration(const ros::NodeHandle &nh);
  virtual ~EventRegistration();

 private:
  void SyncImagesCallback(const sensor_msgs::ImageConstPtr &depth_msg,
                          const sensor_msgs::ImageConstPtr &rgb_msg);
  void EventsCallback(const celex5_msgs::EventVectorConstPtr &events_msg);
  void EventsCameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg);
  void RgbCameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg);

  ros::NodeHandle nh_;
  typedef message_filters::Subscriber<sensor_msgs::Image> MfImageSub;
  ros::Subscriber events_sub_;
  ros::Subscriber events_info_sub_;
  ros::Subscriber rgb_info_sub_;

  std::shared_ptr<MfImageSub> p_depth_image_sub_;
  std::shared_ptr<MfImageSub> p_rgb_image_sub_;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> p_sync_;

  cv::Mat rgb_frame_;
  cv::Mat depth_frame_;
  cv::Mat rgb_on_events_frame_;

  Eigen::Isometry3d T_;
  Eigen::Matrix3d rgb_K_;
  bool rgb_info_initialed;
  // Eigen::Matrix3d rgb_D_;
  Eigen::Matrix3d events_K_;
  cv::Mat events_D_;
  bool events_info_initialed;

  std::shared_ptr<std::thread> p_thread_process_;
  std::condition_variable cv_new_frame_;
  std::mutex mu_new_frame_;
};

#endif //CELEX5_CALIBRATION_SRC_REGISTRATION_EVENT_REGISTRATION_H_
