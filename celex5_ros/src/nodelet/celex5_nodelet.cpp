//
// Created by kehan on 2020/1/13.
//

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
