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

#include "temporal_offset_calculate.h"

TemporalOffsetCalculate::TemporalOffsetCalculate(const ros::NodeHandle &nh, bool show_match)
    : nh_(nh),
      show_match_(show_match),
      corners_col_num_(8),    // default value
      corners_row_num_(6) {
  nh_.param("corners_col_num", corners_col_num_, corners_col_num_);
  nh_.param("corners_row_num", corners_row_num_, corners_row_num_);

  p_ddyn_rec_ = std::make_shared<ddynamic_reconfigure::DDynamicReconfigure>(nh_);
  p_ddyn_rec_->registerVariable<bool>("show_match", &show_match_, "Show corners matching");
  p_ddyn_rec_->publishServicesTopics();
}

TemporalOffsetCalculate::~TemporalOffsetCalculate() {

}

void TemporalOffsetCalculate::Process(cv::Mat image1, cv::Mat image2) {
  if (!K1_.empty() && !K2_.empty() && !D1_.empty() && !D2_.empty()) {
    auto image1_corners = FindCorners(image1);
    auto image2_corners = FindCorners(image2);
    cv::Mat homography_matrix;
    homography_matrix = cv::findHomography(image1_corners, image2_corners, cv::RANSAC, 3,
                                           cv::noArray(), 2000, 0.99);
    // TODO
  }
}

std::vector<cv::Point2f> TemporalOffsetCalculate::FindCorners(const cv::Mat &image) {
  std::vector<cv::Point2f> corners;
  corners.reserve(corners_row_num_ * corners_col_num_);
  bool found = cv::findChessboardCorners(image,
                                         cv::Size(corners_col_num_, corners_row_num_),
                                         corners,
                                         cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
  if (!found) {
    return std::vector<cv::Point2f>();
  }
  cv::TermCriteria criteria = cv::TermCriteria(
      static_cast<int>(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS),
      40, 0.1
  );
  cv::cornerSubPix(image, corners, cv::Size(5, 5), cv::Size(-1, -1), criteria);
  return corners;
}

