//
// Created by kehan on 2020/1/13.
//

#include "celex5_ros.h"

using namespace celex5_ros;

CeleX5ROS::CeleX5ROS()
    : nh_("~") {

  p_celex5_options_ = std::make_shared<CeleX5Options>();
  p_celex5_sensor_ = std::make_shared<CeleX5>();
  p_celex5_configure_ = std::make_shared<CeleX5Configure>(p_celex5_options_, nh_);


}

CeleX5ROS::~CeleX5ROS() {

}
