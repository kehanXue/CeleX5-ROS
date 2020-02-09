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

#ifndef CELEX5_ROS_SRC_BEAN_CELEX5ROSBEAN_H_
#define CELEX5_ROS_SRC_BEAN_CELEX5ROSBEAN_H_

#include <memory>
#include <string>

#include <ros/ros.h>

#include "celex5/celex5datamanager.h"
#include "celex5_options.h"
#include "interactive/celex5_ddy_configure.h"
#include "interactive/celex5_display_controller.h"
#include "celex5_data_forwarder.h"

namespace celex5_ros {
class CeleX5ROSBean {
 public:
  explicit CeleX5ROSBean(const ros::NodeHandle &nh);
  virtual ~CeleX5ROSBean();
  void Run();

 private:
  void ReadParams();

  std::shared_ptr<CeleX5Options> p_celex5_options_;
  std::shared_ptr<CeleX5> p_celex5_sensor_;
  std::shared_ptr<CeleX5DdyConfigure> p_celex5_configure_;
  std::shared_ptr<CeleX5DataForwarder> p_celex5_data_forwarder_;

  std::shared_ptr<CeleX5DisplayController> p_celex5_display_;

  ros::NodeHandle nh_;
};
}

#endif //CELEX5_ROS_SRC_BEAN_CELEX5ROSBEAN_H_
