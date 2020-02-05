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

#ifndef CELEX5_ROS_SRC_CALIBRATION_PATTERNGENERATOR_H_
#define CELEX5_ROS_SRC_CALIBRATION_PATTERNGENERATOR_H_

#include <memory>
#include <thread>

#include <ros/ros.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <opencv2/opencv.hpp>

class PatternGenerator {
 public:
  explicit PatternGenerator(const ros::NodeHandle &nh = ros::NodeHandle("~"),
                            int square_size = 100,
                            int col_corners_num = 8,
                            int row_corners_num = 6,
                            bool static_board = false,
                            int blinking_fps = 15,
                            bool record_video = false,
                            bool show_img = true);
  virtual ~PatternGenerator();

 private:
  void CreateChessBoard();
  void CreateBlackBoard();

  int square_size_;
  int col_corners_num_;
  int row_corners_num_;

  bool static_board_;
  int blinking_fps_;
  cv::Mat pattern_board_;

  bool record_video_;
  bool show_img_;

  ros::NodeHandle nh_;
  std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> p_ddyn_rec_;
  std::shared_ptr<std::thread> p_thread_;
};

#endif //CELEX5_ROS_SRC_CALIBRATION_PATTERNGENERATOR_H_
