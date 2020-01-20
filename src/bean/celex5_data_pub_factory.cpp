//
// Created by kehan on 2020/1/19.
//

#include "celex5_data_pub_factory.h"

#include <utility>

using namespace celex5_ros;

CeleX5DataPubFactory::CeleX5DataPubFactory(const ros::NodeHandle &nh,
                                           std::string topic_name,
                                           std::shared_ptr<CeleX5> p_celex5_sensor,
                                           CeleX5::EventPicType event_pic_type)
    : nh_(nh),
      p_celex5_sensor_(std::move(p_celex5_sensor)),
      event_pic_type_(event_pic_type),
      publish_enable_(false) {
  optical_flow_pic_type_ = CeleX5::OpticalFlowPicType::Unknown_Optical_Flow_Type;
  p_mutex_ = std::make_shared<std::mutex>();

  auto readIntParam = [&](const std::string &param_name, int &param) {
    if (nh_.hasParam(param_name)) {
      nh_.getParam(param_name, param);
      ROS_INFO("%s: Use the param %s: %d",
               ros::this_node::getName().c_str(),
               param_name.c_str(),
               param);
    } else {
      ROS_WARN("%s: Didn't provide param %s, use the default value: %d",
               ros::this_node::getName().c_str(),
               param_name.c_str(),
               param);
    }
  };

  std::thread publish_thread([&]() {
    publisher_ = nh_.advertise<sensor_msgs::Image>(topic_name, 10);
    int fps = 30;
    readIntParam(topic_name + "_fps", fps);
    ros::Rate loop_rate(fps);
    while (ros::ok()) {
      if (publish_enable_) {
        cv::Mat event_img = p_celex5_sensor_->getEventPicMat(event_pic_type_);
        sensor_msgs::ImagePtr image_ptr_msg =
            cv_bridge::CvImage(std_msgs::Header(), "mono8", event_img).toImageMsg();
        publisher_.publish(image_ptr_msg);
      }
    }
  });
  // TODO
  publish_thread.detach();
}

CeleX5DataPubFactory::CeleX5DataPubFactory(const ros::NodeHandle &nh,
                                           std::string topic_name,
                                           std::shared_ptr<CeleX5> p_celex5_sensor,
                                           CeleX5::OpticalFlowPicType optical_flow_pic_type)
    : nh_(nh),
      p_celex5_sensor_(std::move(p_celex5_sensor)),
      optical_flow_pic_type_(optical_flow_pic_type),
      publish_enable_(false) {
  event_pic_type_ = CeleX5::EventPicType::Unknown_Event_Type;
  p_mutex_ = std::make_shared<std::mutex>();

  auto readIntParam = [&](const std::string &param_name, int &param) {
    if (nh_.hasParam(param_name)) {
      nh_.getParam(param_name, param);
      ROS_INFO("%s: Use the param %s: %d",
               ros::this_node::getName().c_str(),
               param_name.c_str(),
               param);
    } else {
      ROS_WARN("%s: Didn't provide param %s, use the default value: %d",
               ros::this_node::getName().c_str(),
               param_name.c_str(),
               param);
    }
  };

  std::thread publish_thread([&]() {
    publisher_ = nh_.advertise<sensor_msgs::Image>(topic_name, 10);
    int fps = 30;
    readIntParam(topic_name + "_fps", fps);
    ros::Rate loop_rate(fps);
    while (ros::ok()) {
      if (publish_enable_) {
        cv::Mat event_img = p_celex5_sensor_->getOpticalFlowPicMat(optical_flow_pic_type_);
        sensor_msgs::ImagePtr image_ptr_msg =
            cv_bridge::CvImage(std_msgs::Header(), "mono8", event_img).toImageMsg();
        publisher_.publish(image_ptr_msg);
      }
    }
  });
  // TODO
  publish_thread.detach();
}

CeleX5DataPubFactory::~CeleX5DataPubFactory() = default;

void CeleX5DataPubFactory::Open() {
  publish_enable_ = true;
}

void CeleX5DataPubFactory::Close() {
  publish_enable_ = false;
}
