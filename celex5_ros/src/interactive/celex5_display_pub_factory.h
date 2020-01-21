//
// Created by kehan on 2020/1/19.
//

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
