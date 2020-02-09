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

#ifndef CELEX5_CALIBRATION_SRC_EXTRINSICS_EXTRINSICS_CALCULATE_H_
#define CELEX5_CALIBRATION_SRC_EXTRINSICS_EXTRINSICS_CALCULATE_H_

#include <memory>

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

#include <ros/ros.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <celex5_msgs/EventVector.h>

class TemporalOffsetCalculate {
 public:
  explicit TemporalOffsetCalculate(const ros::NodeHandle &nh = ros::NodeHandle("~"),
                                   bool show_match = false);
  virtual ~TemporalOffsetCalculate();
  void Process(cv::Mat image1, cv::Mat image2);

 private:

  std::vector<cv::Point2f> FindCorners(const cv::Mat &image);

  ros::NodeHandle nh_;
  std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> p_ddyn_rec_;
  bool show_match_;


};

#endif //CELEX5_CALIBRATION_SRC_EXTRINSICS_EXTRINSICS_CALCULATE_H_
