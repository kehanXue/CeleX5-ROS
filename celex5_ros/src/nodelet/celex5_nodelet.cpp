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

#include "celex5_nodelet.h"

using namespace celex5_ros;

CeleX5Nodelet::CeleX5Nodelet() {
}

CeleX5Nodelet::~CeleX5Nodelet() = default;

void CeleX5Nodelet::onInit() {
  p_celex5_ros_bean_ = std::make_shared<CeleX5ROSBean>(this->getPrivateNodeHandle());
  p_celex5_ros_bean_->Run();
}

//PLUGINLIB_EXPORT_CLASS(celex5_ros::CeleX5Nodelet, nodelet::Nodelet)
