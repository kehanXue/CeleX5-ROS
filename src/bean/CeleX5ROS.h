//
// Created by kehan on 2020/1/13.
//

#ifndef CELEX5_ROS_SRC_BEAN_CELEX5ROS_H_
#define CELEX5_ROS_SRC_BEAN_CELEX5ROS_H_

#include <ros/ros.h>
#include "celex5/celex5datamanager.h"

class CeleX5ROS : public CeleX5DataManager {
 public:

 protected:

 private:

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

#endif //CELEX5_ROS_SRC_BEAN_CELEX5ROS_H_
