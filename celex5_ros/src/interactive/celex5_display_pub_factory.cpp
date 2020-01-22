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

#include "celex5_display_pub_factory.h"

#include <utility>

using namespace celex5_ros;

CeleX5DisplayPubFactory::CeleX5DisplayPubFactory(const ros::NodeHandle &nh,
                                                 std::string topic_name,
                                                 std::shared_ptr<CeleX5> p_celex5_sensor,
                                                 std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> p_ddyn_rec,
                                                 CeleX5::EventPicType event_pic_type)
    : nh_(nh),
      frame_id_("celex5_mipi"),
      p_celex5_sensor_(std::move(p_celex5_sensor)),
      p_ddyn_rec_(std::move(p_ddyn_rec)),
      event_pic_type_(event_pic_type),
      publish_enable_(false) {
  // ROS_INFO("topic_name transfer in: %s", topic_name.c_str());
  nh_.param("frame_id", frame_id_, frame_id_);
  optical_flow_pic_type_ = CeleX5::OpticalFlowPicType::Unknown_Optical_Flow_Type;
  full_frame_pic_type_ = CeleX5::FullFramePicType::Unknown_Full_Frame_Type;
  p_mutex_ = std::make_shared<std::mutex>();

  publish_thread_ = std::make_shared<std::thread>([&]() {
    ROS_INFO("Register display topic name: %s", topic_name.c_str());
    publisher_ = nh_.advertise<sensor_msgs::Image>("display_" + topic_name, 10);
    int fps = 60;
    ros::Rate loop_rate(fps);
    bool is_display = true;
    p_ddyn_rec_->registerVariable<int>(topic_name + "_display_fps", fps,
                                       [&loop_rate](int new_fps) {
                                         loop_rate = ros::Rate(new_fps);
                                       }, "FPS of this display image", 0, 144);
    p_ddyn_rec_->registerVariable<bool>(topic_name + "_enable", is_display,
                                        [&is_display](int new_is_display) {
                                          is_display = new_is_display;
                                        }, "Whether display this image");
    while (ros::ok()) {
      if (publish_enable_ && is_display) {
        cv::Mat event_img = p_celex5_sensor_->getEventPicMat(event_pic_type_);
        sensor_msgs::ImagePtr image_ptr_msg =
            cv_bridge::CvImage(std_msgs::Header(), "mono8", event_img).toImageMsg();
        image_ptr_msg->header.stamp = ros::Time::now();
        image_ptr_msg->header.frame_id = this->frame_id_;
        publisher_.publish(image_ptr_msg);
      }
      loop_rate.sleep();
    }
  });
  publish_thread_->detach();
  usleep(3000);
}

CeleX5DisplayPubFactory::CeleX5DisplayPubFactory(const ros::NodeHandle &nh,
                                                 std::string topic_name,
                                                 std::shared_ptr<CeleX5> p_celex5_sensor,
                                                 std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> p_ddyn_rec,
                                                 CeleX5::OpticalFlowPicType optical_flow_pic_type)
    : nh_(nh),
      p_celex5_sensor_(std::move(p_celex5_sensor)),
      p_ddyn_rec_(std::move(p_ddyn_rec)),
      optical_flow_pic_type_(optical_flow_pic_type),
      publish_enable_(false) {
  // ROS_INFO("topic_name transfer in: %s", topic_name.c_str());
  event_pic_type_ = CeleX5::EventPicType::Unknown_Event_Type;
  full_frame_pic_type_ = CeleX5::FullFramePicType::Unknown_Full_Frame_Type;
  p_mutex_ = std::make_shared<std::mutex>();

  publish_thread_ = std::make_shared<std::thread>([&]() {
    ROS_INFO("Register display topic name: %s", topic_name.c_str());
    publisher_ = nh_.advertise<sensor_msgs::Image>("display_" + topic_name, 10);
    ros::Publisher colored_publisher = nh_.advertise<sensor_msgs::Image>("display_colored_" + topic_name, 10);
    int fps = 60;
    ros::Rate loop_rate(fps);
    bool is_display = true;
    p_ddyn_rec_->registerVariable<int>(topic_name + "_display_fps", fps,
                                       [&loop_rate](int new_fps) {
                                         loop_rate = ros::Rate(new_fps);
                                       }, "FPS of this display image", 0, 144);
    p_ddyn_rec_->registerVariable<bool>(topic_name + "_enable", is_display,
                                        [&is_display](int new_is_display) {
                                          is_display = new_is_display;
                                        }, "Whether display this image");
    while (ros::ok()) {
      if (publish_enable_ && is_display) {

        cv::Mat optical_flow_img = p_celex5_sensor_->getOpticalFlowPicMat(optical_flow_pic_type_);
        sensor_msgs::ImagePtr image_ptr_msg =
            cv_bridge::CvImage(std_msgs::Header(), "mono8", optical_flow_img).toImageMsg();
        image_ptr_msg->header.stamp = ros::Time::now();
        image_ptr_msg->header.frame_id = this->frame_id_;
        publisher_.publish(image_ptr_msg);

        ToColorOpticalMat(optical_flow_img);
        sensor_msgs::ImagePtr colored_image_ptr_msg =
            cv_bridge::CvImage(std_msgs::Header(), "bgr8", optical_flow_img).toImageMsg();
        image_ptr_msg->header.stamp = ros::Time::now();
        image_ptr_msg->header.frame_id = this->frame_id_;
        colored_publisher.publish(colored_image_ptr_msg);
      }
      loop_rate.sleep();
    }
  });
  publish_thread_->detach();
  usleep(3000);
}

CeleX5DisplayPubFactory::CeleX5DisplayPubFactory(const ros::NodeHandle &nh,
                                                 std::string topic_name,
                                                 std::shared_ptr<CeleX5> p_celex5_sensor,
                                                 std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> p_ddyn_rec,
                                                 CeleX5::FullFramePicType full_frame_pic_type)

    : nh_(nh),
      p_celex5_sensor_(std::move(p_celex5_sensor)),
      p_ddyn_rec_(std::move(p_ddyn_rec)),
      full_frame_pic_type_(full_frame_pic_type),
      publish_enable_(false) {
  // ROS_INFO("topic_name transfer in: %s", topic_name.c_str());
  event_pic_type_ = CeleX5::EventPicType::Unknown_Event_Type;
  optical_flow_pic_type_ = CeleX5::OpticalFlowPicType::Unknown_Optical_Flow_Type;
  p_mutex_ = std::make_shared<std::mutex>();

  publish_thread_ = std::make_shared<std::thread>([&]() {
    ROS_INFO("Register display topic name: %s", topic_name.c_str());
    publisher_ = nh_.advertise<sensor_msgs::Image>("display_" + topic_name, 10);
    int fps = 60;
    ros::Rate loop_rate(fps);
    bool is_display = true;
    p_ddyn_rec_->registerVariable<int>(topic_name + "_display_fps", fps,
                                       [&loop_rate](int new_fps) {
                                         loop_rate = ros::Rate(new_fps);
                                       }, "FPS of this display image", 0, 144);
    p_ddyn_rec_->registerVariable<bool>(topic_name + "_enable", is_display,
                                        [&is_display](int new_is_display) {
                                          is_display = new_is_display;
                                        }, "Whether display this image");
    while (ros::ok()) {
      if (publish_enable_ && is_display) {
        // p_celex5_sensor_->getSensorFixedMode()==CeleX5::Full_Picture_Mode) {
        if (!p_celex5_sensor_->getFullPicMat().empty()) {
          cv::Mat full_frame_img = p_celex5_sensor_->getFullPicMat();
          // cv::imshow("FullPic", full_frame_img);
          // cv::waitKey(10);
          sensor_msgs::ImagePtr image_ptr_msg =
              cv_bridge::CvImage(std_msgs::Header(), "mono8", full_frame_img).toImageMsg();
          image_ptr_msg->header.stamp = ros::Time::now();
          image_ptr_msg->header.frame_id = this->frame_id_;
          publisher_.publish(image_ptr_msg);
        }
      }
      loop_rate.sleep();
    }
  });
  publish_thread_->detach();
  usleep(3000);
}

CeleX5DisplayPubFactory::~CeleX5DisplayPubFactory() {
  if (publish_thread_->joinable()) {
    publish_thread_->join();
  }
}

void CeleX5DisplayPubFactory::Open() {
  publish_enable_ = true;
}

void CeleX5DisplayPubFactory::Close() {
  publish_enable_ = false;
}

bool CeleX5DisplayPubFactory::IsPublishEnable() const {
  return publish_enable_;
}

void CeleX5DisplayPubFactory::ToColorOpticalMat(cv::Mat &optical_mat) {
  cv::Mat color_optical_mat(800, 1280, CV_8UC3);
  for (int i = 0; i < color_optical_mat.rows; ++i) {
    auto *p = color_optical_mat.ptr<cv::Vec3b>(i);
    for (int j = 0; j < color_optical_mat.cols; ++j) {
      int value = optical_mat.at<uchar>(i, j);
      if (value==0) {
        p[j][0] = 0;
        p[j][1] = 0;
        p[j][2] = 0;
      } else if (value < 50) {
        //blue
        p[j][0] = 255;
        p[j][1] = 0;
        p[j][2] = 0;
      } else if (value < 100) {
        p[j][0] = 255;
        p[j][1] = 255;
        p[j][2] = 0;
      } else if (value < 150) {
        //green
        p[j][0] = 0;
        p[j][1] = 255;
        p[j][2] = 0;
      } else if (value < 200) {
        p[j][0] = 0;
        p[j][1] = 255;
        p[j][2] = 255;
      } else {
        //red
        p[j][0] = 0;
        p[j][1] = 0;
        p[j][2] = 255;
      }
    }
  }
  optical_mat = color_optical_mat.clone();
}


