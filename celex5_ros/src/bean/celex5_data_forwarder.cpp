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

#include "celex5_data_forwarder.h"

celex5_ros::CeleX5DataForwarder::CeleX5DataForwarder(const ros::NodeHandle &nh,
                                                     const std::shared_ptr<CeleX5> &p_celex5_sensor)
    : nh_(nh),
      p_celex5_sensor_(p_celex5_sensor) {

  p_celex5_data_server_ = p_celex5_sensor->getSensorDataServer();
  p_celex5_data_server_->registerData(this, emDataType::CeleX_Frame_Data);

  CeleX5Configure::ReadROSParam(nh_, "frame_id", this->frame_id_);

  events_pub_ = nh_.advertise<celex5_msgs::EventVector>("events", 10);
  imu_pub_ = nh_.advertise<celex5_msgs::ImuVector>("imu_data", 10);
  polarity_img_pub_ = nh_.advertise<sensor_msgs::Image>("polarity_img", 10);

  // binary_img_pub_ = nh_.advertise<sensor_msgs::Image>("event_binary_img", 10);
  // denoised_img_pub_ = nh_.advertise<sensor_msgs::Image>("event_denoised_img", 10);;
  // count_img_pub_ = nh_.advertise<sensor_msgs::Image>("event_count_img", 10);;
  // optical_flow_img_pub_ = nh_.advertise<sensor_msgs::Image>("event_optical_flow_img", 10);;
  //
  // gray_img_pub_ = nh_.advertise<sensor_msgs::Image>("event_gray_img", 10);;
  // accumulated_img_pub_ = nh_.advertise<sensor_msgs::Image>("event_accumulated_img", 10);;
  // superimposed_img_pub_ = nh_.advertise<sensor_msgs::Image>("event_superimposed_img", 10);;
  // optical_flow_direction_img_pub_ = nh_.advertise<sensor_msgs::Image>("event_optical_flow_direction_img", 10);;
  // optical_flow_speed_img_pub_ = nh_.advertise<sensor_msgs::Image>("event_optical_flow_speed_img", 10);;
}

celex5_ros::CeleX5DataForwarder::~CeleX5DataForwarder() {
  p_celex5_data_server_->unregisterData(this, emDataType::CeleX_Frame_Data);
}

void celex5_ros::CeleX5DataForwarder::onFrameDataUpdated(CeleX5ProcessedData *p_sensor_data) {
  // clock_t time_now = clock();
  // static clock_t time_last = clock();
  // ROS_ERROR("!!!!!!!!!!!!!!!fps time!!!!!!!!!!!!!!!!");
  // ROS_ERROR("%lf ms\n\n", 1000.0*(time_now - time_last)/CLOCKS_PER_SEC);
  // time_last = time_now;
  CeleX5::CeleX5Mode current_mode = p_celex5_sensor_->getSensorFixedMode();
  // if (!p_celex5_sensor_->isLoopModeEnabled()) {
  if ((current_mode!=CeleX5::CeleX5Mode::Full_Picture_Mode &&
      current_mode!=CeleX5::CeleX5Mode::Optical_Flow_Mode &&
      !p_celex5_sensor_->isLoopModeEnabled())
      || p_celex5_sensor_->isLoopModeEnabled()) {
    std::vector<EventData> vec_events;
    if (!p_celex5_sensor_->getEventDataVector(vec_events)) {
      ROS_ERROR("Read events error!");
      return;
    }

    celex5_msgs::EventVectorPtr event_vector_ptr_msg =
        boost::make_shared<celex5_msgs::EventVector>();
    event_vector_ptr_msg->header.stamp = ros::Time::now();
    event_vector_ptr_msg->header.frame_id = this->frame_id_;
    event_vector_ptr_msg->height = CELEX5_MAT_ROWS;
    event_vector_ptr_msg->width = CELEX5_MAT_COLS;
    event_vector_ptr_msg->vector_length = vec_events.size();
    for (auto event_i : vec_events) {
      celex5_msgs::Event tmp_event;
      tmp_event.x = event_i.row;
      tmp_event.y = event_i.col;
      if (current_mode==CeleX5::Event_Off_Pixel_Timestamp_Mode) {
        tmp_event.brightness = 255;
      } else if (current_mode==CeleX5::Event_In_Pixel_Timestamp_Mode) {
        tmp_event.brightness = 255;
        tmp_event.in_pixel_timestamp = event_i.tInPixelIncreasing;
      } else if (current_mode==CeleX5::Event_Intensity_Mode) {
        tmp_event.brightness = event_i.adc;
        tmp_event.polarity = event_i.polarity;
      }
      tmp_event.off_pixel_timestamp = event_i.tOffPixelIncreasing;
      event_vector_ptr_msg->events.emplace_back(tmp_event);
    }
    events_pub_.publish(event_vector_ptr_msg);

    /*
     * Publish polarity image in Event Intensity Mode
     */
    if ((current_mode==CeleX5::Event_Intensity_Mode && !p_celex5_sensor_->isLoopModeEnabled())
        || (p_celex5_sensor_->isLoopModeEnabled()
            && p_celex5_sensor_->getSensorLoopMode(2)==CeleX5::Event_Intensity_Mode)) {
      cv::Mat polarity_mat(800, 1280, CV_8UC3, cv::Scalar::all(0));
      int data_size = vec_events.size();
      int row = 0, col = 0;
      for (int i = 0; i < data_size; i++) {
        row = 799 - vec_events[i].row;
        // col = 1279 - vec_events[i].col;
        col = vec_events[i].col;
        auto *p = polarity_mat.ptr<cv::Vec3b>(row);
        if (vec_events[i].polarity==1) {
          p[col][0] = 0;
          p[col][1] = 0;
          p[col][2] = 255;
        } else if (vec_events[i].polarity==-1) {
          p[col][0] = 255;
          p[col][1] = 0;
          p[col][2] = 0;
        } else {
          p[col][0] = 0;
          p[col][1] = 0;
          p[col][2] = 0;
        }
      }
      sensor_msgs::ImagePtr image_ptr_msg =
          cv_bridge::CvImage(std_msgs::Header(), "bgr8", polarity_mat).toImageMsg();
      image_ptr_msg->header.stamp = ros::Time::now();
      image_ptr_msg->header.frame_id = this->frame_id_;
      polarity_img_pub_.publish(image_ptr_msg);
      // if (data_size > 0) {
      //   cv::imshow("Event Polarity Pic", polarity_mat);
      //   cv::waitKey(1);
      // }
    }
  }
  // }

  if (p_celex5_sensor_->isIMUModuleEnabled()) {
    std::vector<IMUData> vec_imus;
    p_celex5_sensor_->getIMUData(vec_imus);
    celex5_msgs::ImuVectorPtr imu_vector_ptr_msg =
        boost::make_shared<celex5_msgs::ImuVector>();
    imu_vector_ptr_msg->header.stamp = ros::Time::now();
    imu_vector_ptr_msg->header.frame_id = this->frame_id_;
    imu_vector_ptr_msg->vector_length = vec_imus.size();
    for (auto imu_i : vec_imus) {
      celex5_msgs::Imu tmp_imu_msg;
      tmp_imu_msg.header.stamp = ros::Time::now();
      tmp_imu_msg.header.frame_id = this->frame_id_;
      tmp_imu_msg.gyro_x = imu_i.xGYROS;
      tmp_imu_msg.gyro_y = imu_i.yGYROS;
      tmp_imu_msg.gyro_z = imu_i.zGYROS;
      tmp_imu_msg.acc_x = imu_i.xACC;
      tmp_imu_msg.acc_y = imu_i.yACC;
      tmp_imu_msg.acc_z = imu_i.zACC;
      tmp_imu_msg.mag_x = imu_i.xMAG;
      tmp_imu_msg.mag_y = imu_i.yMAG;
      tmp_imu_msg.mag_z = imu_i.zMAG;

      imu_vector_ptr_msg->imus.emplace_back(tmp_imu_msg);
    }
    // TODO Add params: sensor_type, iso, imu_enable
    imu_pub_.publish(imu_vector_ptr_msg);
  }
}


