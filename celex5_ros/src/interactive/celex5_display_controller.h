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

#ifndef CELEX5_ROS_SRC_BEAN_CELEX5_DISPLAY_CONTROLLER_H_
#define CELEX5_ROS_SRC_BEAN_CELEX5_DISPLAY_CONTROLLER_H_

#include <unordered_map>

#include <ros/ros.h>
#include "celex5_display_pub_factory.h"
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>

namespace celex5_ros {

typedef std::shared_ptr<CeleX5DisplayPubFactory> CeleX5DisplayPubFactoryPtr;

class CeleX5DisplayController {
 public:
  static std::shared_ptr<CeleX5DisplayController> GetInstance(const ros::NodeHandle &nh,
                                                              const std::shared_ptr<CeleX5> &p_celex5_sensor);
  virtual ~CeleX5DisplayController();
  void SetCeleX5Mode(CeleX5::CeleX5Mode mode);

 private:

  CeleX5DisplayController(const ros::NodeHandle &nh,
                          std::shared_ptr<CeleX5> p_celex5_sensor);
  CeleX5DisplayController(const CeleX5DisplayController &);
  CeleX5DisplayController &operator=(const CeleX5DisplayController &);

  static std::shared_ptr<CeleX5DisplayController> instance;
  static std::shared_ptr<std::mutex> mutex_instance;

  void CloseAll();
  void ChangeOptions(uint16_t mask);

  std::shared_ptr<CeleX5> p_celex5_sensor_;
  std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> p_ddyn_rec_;

  ros::NodeHandle nh_;

  CeleX5DisplayPubFactoryPtr p_binary_img_pub_;
  CeleX5DisplayPubFactoryPtr p_denoised_binary_img_pub_;
  CeleX5DisplayPubFactoryPtr p_count_img_pub_;
  CeleX5DisplayPubFactoryPtr p_optical_flow_img_pub_;
  CeleX5DisplayPubFactoryPtr p_accumulated_img_pub_;
  CeleX5DisplayPubFactoryPtr p_gray_img_pub_;
  CeleX5DisplayPubFactoryPtr p_superimposed_img_pub_;
  CeleX5DisplayPubFactoryPtr p_optical_flow_direction_img_pub_;
  CeleX5DisplayPubFactoryPtr p_optical_flow_speed_img_pub_;
  CeleX5DisplayPubFactoryPtr p_in_pixel_img_pub_;
  CeleX5DisplayPubFactoryPtr p_full_frame_img_pub_;

  std::unordered_map<uint8_t, CeleX5DisplayPubFactoryPtr> map_pub_mask;
  std::unordered_map<int, uint16_t> map_mode_mask;
};
}

#endif //CELEX5_ROS_SRC_BEAN_CELEX5_DISPLAY_CONTROLLER_H_
