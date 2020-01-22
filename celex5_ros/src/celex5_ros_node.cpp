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

#include <ros/ros.h>

#include "bean/celex5_ros_bean.h"

using namespace celex5_ros;

int main(int argc, char **argv) {
  ros::init(argc, argv, "celex5_ros_node");
  ros::NodeHandle nh("~");
  CeleX5ROSBean celex5_ros_bean(nh);
  celex5_ros_bean.Run();

  // std::shared_ptr<CeleX5> p_celex5_sensor = std::make_shared<CeleX5>();
  // p_celex5_sensor->openSensor(CeleX5::CeleX5_MIPI);
  // CeleX5::CeleX5Mode sensorMode = CeleX5::Full_Picture_Mode;
  // p_celex5_sensor->setSensorFixedMode(sensorMode);
  //
  // auto publish_thread = std::make_shared<std::thread>([&]() {
  //   ros::Rate loop_rate(30);
  //   while (ros::ok()) {
  //     if (sensorMode==CeleX5::Full_Picture_Mode) {
  //       if (!p_celex5_sensor->getFullPicMat().empty()) {
  //         cv::Mat fullPicMat = p_celex5_sensor->getFullPicMat();
  //         cv::imshow("FullPic", fullPicMat);
  //         cv::waitKey(10);
  //       }
  //     } else if (sensorMode==CeleX5::Event_Off_Pixel_Timestamp_Mode) {
  //       if (!p_celex5_sensor->getEventPicMat(CeleX5::EventBinaryPic).empty()) {
  //         cv::Mat eventPicMat = p_celex5_sensor->getEventPicMat(CeleX5::EventBinaryPic);
  //         cv::imshow("Event-EventBinaryPic", eventPicMat);
  //       }
  //       cv::waitKey(10);
  //     }
  //     loop_rate.sleep();
  //   }
  // });
  //
  // publish_thread->detach();
  ros::Rate loop_rate(60);
  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
  // ros::spin();
  // ROS_INFO("!!!");
  return 0;
}

