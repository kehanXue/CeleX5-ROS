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

#ifndef CELEX5_ROS_SRC_BEAN_CELEX5_NODELET_H_
#define CELEX5_ROS_SRC_BEAN_CELEX5_NODELET_H_

#include <memory>
#include <string>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "bean/celex5_ros_bean.h"

namespace celex5_ros {
class CeleX5Nodelet : public nodelet::Nodelet {

 public:
  CeleX5Nodelet();
  ~CeleX5Nodelet() override;

 private:
  void onInit() override;
  std::shared_ptr<CeleX5ROSBean> p_celex5_ros_bean_;
};
}
PLUGINLIB_EXPORT_CLASS(celex5_ros::CeleX5Nodelet, nodelet::Nodelet)

#endif //CELEX5_ROS_SRC_BEAN_CELEX5_NODELET_H_
