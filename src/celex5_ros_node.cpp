//
// Created by kehan on 2020/1/18.
//

#include <ros/ros.h>

#include "bean/celex5_ros_bean.h"

using namespace celex5_ros;

int main(int argc, char **argv) {
  ros::init(argc, argv, "celex5_ros_node");
  CeleX5ROSBean celex5_ros_bean;
  celex5_ros_bean.Run();

  ros::Rate loop_rate(60);
  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
}

