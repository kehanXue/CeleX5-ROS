//
// Created by kehan on 2020/1/13.
//

#ifndef CELEX5_ROS_SRC_BEAN_CELEX5_ROS_H_
#define CELEX5_ROS_SRC_BEAN_CELEX5_ROS_H_

#include <ros/ros.h>
#include "celex5/celex5datamanager.h"
#include "celex5_options.h"
#include "interface/celex5_configure.h"

class CeleX5ROS : public CeleX5DataManager {

 public:
  CeleX5ROS();
  virtual ~CeleX5ROS();

 private:

  std::shared_ptr<CeleX5Options> p_celex5_options_;
  std::shared_ptr<CeleX5> p_celex5_sensor_;
  CeleX5Configure celex5_configure_;

  ros::NodeHandle nh_;

  ros::Publisher events_pub_;
  ros::Publisher imu_pub_;
  ros::Publisher mag_pub_;

  ros::Publisher event_binary_img_pub_;
  ros::Publisher event_denoised_img_pub_;
  ros::Publisher event_count_img_pub_;
  ros::Publisher event_optical_flow_img_pub_;

  ros::Publisher event_gray_img_pub_;
  ros::Publisher event_accumulated_img_pub_;
  ros::Publisher event_superimposed_img_pub_;
  ros::Publisher event_optical_flow_direction_img_pub_;
  ros::Publisher event_optical_flow_speed_img_pub_;
};

#endif //CELEX5_ROS_SRC_BEAN_CELEX5_ROS_H_
