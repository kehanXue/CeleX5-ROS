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

#include <vector>
#include <thread>

#include <ros/ros.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>

#ifndef CELEX5_CALIBRATION_SRC_EXTRINSICS_PUBSTOREDSYNCIMAGES_H_
#define CELEX5_CALIBRATION_SRC_EXTRINSICS_PUBSTOREDSYNCIMAGES_H_

class PubStoredSyncImages {
 public:
  explicit PubStoredSyncImages(const ros::NodeHandle &nh = ros::NodeHandle("~"),
                               bool is_publish_enable = false,
                               int publish_rate = 30);
  virtual ~PubStoredSyncImages();
 private:
  ros::NodeHandle nh_;
  std::vector<cv::String> image1_filenames_;
  std::vector<cv::String> image2_filenames_;

  ros::Publisher image1_pub_;
  ros::Publisher image2_pub_;

  std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> p_ddyn_rec_;
  bool is_publish_enable_;
  int publish_rate_;

  std::shared_ptr<std::thread> p_main_thread_;
};

#endif //CELEX5_CALIBRATION_SRC_EXTRINSICS_PUBSTOREDSYNCIMAGES_H_
