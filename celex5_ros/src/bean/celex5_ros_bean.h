//
// Created by kehan on 2020/1/18.
//

#ifndef CELEX5_ROS_SRC_BEAN_CELEX5ROSBEAN_H_
#define CELEX5_ROS_SRC_BEAN_CELEX5ROSBEAN_H_

#include <memory>
#include <string>

#include <ros/ros.h>

#include "celex5/celex5datamanager.h"
#include "celex5_options.h"
#include "interactive/celex5_configure.h"
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
  std::shared_ptr<CeleX5Configure> p_celex5_configure_;
  std::shared_ptr<CeleX5DataForwarder> p_celex5_data_forwarder_;

  std::shared_ptr<CeleX5DisplayController> p_celex5_display_;

  ros::NodeHandle nh_;
};
}

#endif //CELEX5_ROS_SRC_BEAN_CELEX5ROSBEAN_H_
