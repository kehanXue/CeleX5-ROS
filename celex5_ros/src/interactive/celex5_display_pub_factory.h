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

#ifndef CELEX5_ROS_SRC_BEAN_CELEX5_DISPLAY_PUB_FACTORY_H_
#define CELEX5_ROS_SRC_BEAN_CELEX5_DISPLAY_PUB_FACTORY_H_

#include <thread>
#include <memory>
#include <mutex>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>

#include "celex5/celex5.h"

namespace celex5_ros {

class CeleX5DisplayPubFactory {

 public:
  CeleX5DisplayPubFactory(const ros::NodeHandle &nh,
                          std::string topic_name,
                          std::shared_ptr<CeleX5> p_celex5_sensor,
                          std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> p_ddyn_rec,
                          CeleX5::EventPicType event_pic_type);
  CeleX5DisplayPubFactory(const ros::NodeHandle &nh,
                          std::string topic_name,
                          std::shared_ptr<CeleX5> p_celex5_sensor,
                          std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> p_ddyn_rec,
                          CeleX5::OpticalFlowPicType optical_flow_pic_type);
  CeleX5DisplayPubFactory(const ros::NodeHandle &nh,
                          std::string topic_name,
                          std::shared_ptr<CeleX5> p_celex5_sensor,
                          std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> p_ddyn_rec,
                          CeleX5::FullFramePicType full_frame_pic_type);

  virtual ~CeleX5DisplayPubFactory();

  void Open();
  void Close();

  bool IsPublishEnable() const;

 private:

  static void ToColorOpticalMat(cv::Mat &optical_mat);

  std::shared_ptr<CeleX5> p_celex5_sensor_;
  std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> p_ddyn_rec_;

  ros::NodeHandle nh_;
  std::string frame_id_;
  ros::Publisher publisher_;

  bool publish_enable_;
  std::shared_ptr<std::mutex> p_mutex_; //TODO Delete
  std::shared_ptr<std::thread> publish_thread_;

//  CeleX5PicType celex5_pic_type_;
  CeleX5::EventPicType event_pic_type_;
  CeleX5::OpticalFlowPicType optical_flow_pic_type_;
  CeleX5::FullFramePicType full_frame_pic_type_;
};
}

#endif //CELEX5_ROS_SRC_BEAN_CELEX5_DISPLAY_PUB_FACTORY_H_
