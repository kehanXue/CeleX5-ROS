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

#ifndef CELEX5_CALIBRATION_SRC_EXTRINSICS_SYNCIMAGES_H_
#define CELEX5_CALIBRATION_SRC_EXTRINSICS_SYNCIMAGES_H_

#include <string>
#include <boost/thread/thread.hpp>

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>

class SyncImages {
 public:
  explicit SyncImages(const ros::NodeHandle &nh = ros::NodeHandle("~"));
  virtual ~SyncImages();

 private:
  void SyncImagesCallback(const sensor_msgs::ImageConstPtr &frame_msg,
                          const sensor_msgs::ImageConstPtr &events_msg);

  ros::NodeHandle nh_;
  typedef message_filters::Subscriber<sensor_msgs::Image> MfImageSub;
  std::shared_ptr<MfImageSub> p_frame_sub_;
  std::shared_ptr<MfImageSub> p_events_sub_;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> p_sync_;
};

#endif //CELEX5_CALIBRATION_SRC_EXTRINSICS_SYNCIMAGES_H_
