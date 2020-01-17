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

#include "celex5/celex5datamanager.h"
#include "celex5_options.h"
#include "interface/celex5_configure.h"

#include "celex5_data_forwarder.h"

namespace celex5_ros {
class CeleX5Nodelet : public nodelet::Nodelet {

 public:
  CeleX5Nodelet();
  virtual ~CeleX5Nodelet();

 private:

  void ReadParams();

  void onInit() override;

  std::shared_ptr<CeleX5Options> p_celex5_options_;
  std::shared_ptr<CeleX5> p_celex5_sensor_;
  std::shared_ptr<CeleX5Configure> p_celex5_configure_;
  std::shared_ptr<CeleX5DataForwarder> p_celex5_data_forwarder_;

  ros::NodeHandle nh_;
};
}
PLUGINLIB_EXPORT_CLASS(celex5_ros::CeleX5Nodelet, nodelet::Nodelet)

#endif //CELEX5_ROS_SRC_BEAN_CELEX5_NODELET_H_
