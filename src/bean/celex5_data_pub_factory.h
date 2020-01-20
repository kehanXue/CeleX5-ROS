//
// Created by kehan on 2020/1/19.
//

#ifndef CELEX5_ROS_SRC_BEAN_CELEX5_DATA_PUB_FACTORY_H_
#define CELEX5_ROS_SRC_BEAN_CELEX5_DATA_PUB_FACTORY_H_

#include <thread>
#include <memory>
#include <mutex>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include "celex5_msgs/event.h"
#include "celex5_msgs/eventVector.h"
#include "celex5/celex5.h"

namespace celex5_ros {

//enum CeleX5PicType {
//  Event = 0,
//  OpticalFlow = 1
//};

class CeleX5DataPubFactory {

 public:
  CeleX5DataPubFactory(const ros::NodeHandle &nh,
                       std::string topic_name,
                       std::shared_ptr<CeleX5> p_celex5_sensor,
                       CeleX5::EventPicType event_pic_type);

  CeleX5DataPubFactory(const ros::NodeHandle &nh,
                       std::string topic_name,
                       std::shared_ptr<CeleX5> p_celex5_sensor,
                       CeleX5::OpticalFlowPicType optical_flow_pic_type);

  virtual ~CeleX5DataPubFactory();

  void Open();

  void Close();

 private:
  std::shared_ptr<CeleX5> p_celex5_sensor_;
  ros::NodeHandle nh_;
  ros::Publisher publisher_;

  bool publish_enable_;
  std::shared_ptr<std::mutex> p_mutex_; //TODO Delete

//  CeleX5PicType celex5_pic_type_;
  CeleX5::EventPicType event_pic_type_;
  CeleX5::OpticalFlowPicType optical_flow_pic_type_;
};
}

#endif //CELEX5_ROS_SRC_BEAN_CELEX5_DATA_PUB_FACTORY_H_
