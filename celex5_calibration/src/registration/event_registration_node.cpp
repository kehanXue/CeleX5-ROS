/**
 *  The celex5_calibration ROS package,
 *  used to calibrate camera parameters for CeleX5-MIPI Event-based Camera.
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
#include "event_registration.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "event_registration_node");
  ros::NodeHandle nh("~");

  // TicToc t_start;
  // ros::Time start = ros::Time::now();
  // for (int i = 0; i < 2280 * 1242; ++i) {
  //   double b = 1.2 * 3;
  // }
  // std::cout << (ros::Time::now() - start).toSec() << std::endl;
  // std::cout << t_start.toc() << std::endl;
  EventRegistration event_registration(nh);
  ros::spin();
}
