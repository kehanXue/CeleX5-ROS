//
// Created by kehan on 2020/1/13.
//

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
