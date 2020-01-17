//
// Created by kehan on 2020/1/17.
//

#include "celex5_data_forwarder.h"

celex5_ros::CeleX5DataForwarder::CeleX5DataForwarder(const ros::NodeHandle &nh,
                                                     const std::shared_ptr<CeleX5> &p_celex5_sensor)
    : nh_(nh),
      p_celex5_sensor_(p_celex5_sensor) {

  p_celex5_data_server_ = p_celex5_sensor->getSensorDataServer();
  p_celex5_data_server_->registerData(this, emDataType::CeleX_Frame_Data);

  events_pub_ = nh_.advertise<celex5_msgs::eventVector>("events", 10);
  imu_pub_ = nh_.advertise<sensor_msgs::Imu>("imu", 10);
  mag_pub_ = nh_.advertise<sensor_msgs::MagneticField>("magnetic", 10);

  event_binary_img_pub_ = nh_.advertise<sensor_msgs::Image>("event_binary_img", 10);
  event_denoised_img_pub_ = nh_.advertise<sensor_msgs::Image>("event_denoised_img", 10);;
  event_count_img_pub_ = nh_.advertise<sensor_msgs::Image>("event_count_img", 10);;
  event_optical_flow_img_pub_ = nh_.advertise<sensor_msgs::Image>("event_optical_flow_img", 10);;

  event_gray_img_pub_ = nh_.advertise<sensor_msgs::Image>("event_gray_img", 10);;
  event_accumulated_img_pub_ = nh_.advertise<sensor_msgs::Image>("event_accumulated_img", 10);;
  event_superimposed_img_pub_ = nh_.advertise<sensor_msgs::Image>("event_superimposed_img", 10);;
  event_optical_flow_direction_img_pub_ = nh_.advertise<sensor_msgs::Image>("event_optical_flow_direction_img", 10);;
  event_optical_flow_speed_img_pub_ = nh_.advertise<sensor_msgs::Image>("event_optical_flow_speed_img", 10);;
}

celex5_ros::CeleX5DataForwarder::~CeleX5DataForwarder() {
  p_celex5_data_server_->unregisterData(this, emDataType::CeleX_Frame_Data);
}

void celex5_ros::CeleX5DataForwarder::onFrameDataUpdated(CeleX5ProcessedData *p_sensor_data) {
  if (!p_celex5_sensor_->isLoopModeEnabled()) {
    if (CeleX5::CeleX5Mode::Event_Off_Pixel_Timestamp_Mode==p_celex5_sensor_->getSensorFixedMode()) {
      std::vector<EventData> vec_events;
      if (!p_celex5_sensor_->getEventDataVector(vec_events)) {
        ROS_ERROR("Read events error!");
        return;
      }
      celex5_msgs::eventVectorPtr event_vector_ptr_msg =
          boost::make_shared<celex5_msgs::eventVector>();
      auto convertVector2EventMsg = [&]() {
        event_vector_ptr_msg->vectorIndex = 0;
        event_vector_ptr_msg->height = CELEX5_MAT_ROWS;
        event_vector_ptr_msg->width = CELEX5_MAT_COLS;
        event_vector_ptr_msg->vectorLength = vec_events.size();
        for (auto event_i : vec_events) {
          celex5_msgs::event tmp_event;
          tmp_event.x = event_i.row;
          tmp_event.y = event_i.col;
          tmp_event.brightness = 255;
          tmp_event.timestamp = event_i.tOffPixelIncreasing;
          event_vector_ptr_msg->events.emplace_back(tmp_event);
        }
      };
      convertVector2EventMsg();
      events_pub_.publish(event_vector_ptr_msg);

      cv::Mat binary_img = p_celex5_sensor_->getEventPicMat(CeleX5::EventBinaryPic);
      sensor_msgs::ImagePtr image_ptr_msg =
          cv_bridge::CvImage(std_msgs::Header(), "mono8", binary_img).toImageMsg();
      event_binary_img_pub_.publish(image_ptr_msg);

    }
  }
}


