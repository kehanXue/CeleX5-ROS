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

#include "celex5_ros_bean.h"

using namespace celex5_ros;

celex5_ros::CeleX5ROSBean::CeleX5ROSBean(const ros::NodeHandle &nh)
    : nh_(nh) {

  p_celex5_options_ = CeleX5Options::GetInstance();
  /*
   * Read initial parameters from ROS Param Server
   */
  ReadParams();

  p_celex5_sensor_ = std::make_shared<CeleX5>();
  p_celex5_configure_ =
      std::make_shared<CeleX5DdyConfigure>(p_celex5_sensor_, nh_);
}

celex5_ros::CeleX5ROSBean::~CeleX5ROSBean() = default;

void celex5_ros::CeleX5ROSBean::Run() {
  /*
   * Open CeleX5 Sensor
   */
  std::string sensor_cfg_file_dir;
  CeleX5DdyConfigure::ReadROSParam(nh_, "sensor_cfg_file_dir", sensor_cfg_file_dir);
  p_celex5_sensor_->setSensorCfgFileDir(sensor_cfg_file_dir);
  std::string fpn_file_dir;
  CeleX5DdyConfigure::ReadROSParam(nh_, "fpn_file_dir", fpn_file_dir);
  p_celex5_sensor_->setFpnFileDir(fpn_file_dir);

  /*
   * Open sensor
   */
  int device_type = CeleX5::DeviceType::Unknown_Devive;
  CeleX5DdyConfigure::ReadROSParam(nh_, "device_type", device_type);
  p_celex5_sensor_->openSensor(static_cast<CeleX5::DeviceType>(device_type));
  ROS_INFO("Sensor status: %d", p_celex5_sensor_->isSensorReady());

  /*
   * Update CeleX5 sensor config with default value
   */
  p_celex5_configure_->UpdateCeleX5AllOptions();

  /*
   * Create CeleX5DataForwarder handle to forward full stream data from CeleX5 Sensor to ROS framework
   */
  p_celex5_data_forwarder_ = std::make_shared<CeleX5DataForwarder>(nh_, p_celex5_sensor_);

  /*
   * Provide display function. Can change the display fps in rqt_reconfigure
   */
  p_celex5_display_ = CeleX5DisplayController::GetInstance(nh_,
                                                           p_celex5_sensor_);
  p_celex5_display_->SetCeleX5Mode(p_celex5_sensor_->getSensorFixedMode());

}

void celex5_ros::CeleX5ROSBean::ReadParams() {
  /*
   * Read parameters from ROS param server
   */

  // TODO Get default value from sensor
  int tmp_celex5_mode_param = p_celex5_options_->GetFixedMode();
  CeleX5DdyConfigure::ReadROSParam(nh_, "fixed_mode", tmp_celex5_mode_param);
  p_celex5_options_->SetFixedMode(static_cast<CeleX5::CeleX5Mode>(tmp_celex5_mode_param));
  CeleX5DdyConfigure::ReadROSParam(nh_, "loop_mode1", tmp_celex5_mode_param);
  p_celex5_options_->SetLoopMode1(static_cast<CeleX5::CeleX5Mode>(tmp_celex5_mode_param));
  CeleX5DdyConfigure::ReadROSParam(nh_, "loop_mode2", tmp_celex5_mode_param);
  p_celex5_options_->SetLoopMode2(static_cast<CeleX5::CeleX5Mode>(tmp_celex5_mode_param));
  CeleX5DdyConfigure::ReadROSParam(nh_, "loop_mode3", tmp_celex5_mode_param);
  p_celex5_options_->SetLoopMode3(static_cast<CeleX5::CeleX5Mode>(tmp_celex5_mode_param));

  CeleX5DdyConfigure::ReadROSParam(nh_, "event_frame_time", p_celex5_options_->event_frame_time_);
  CeleX5DdyConfigure::ReadROSParam(nh_, "optical_flow_frame_time", p_celex5_options_->optical_flow_frame_time_);
  CeleX5DdyConfigure::ReadROSParam(nh_, "threshold", p_celex5_options_->threshold_);
  CeleX5DdyConfigure::ReadROSParam(nh_, "brightness", p_celex5_options_->brightness_);
  CeleX5DdyConfigure::ReadROSParam(nh_, "imu_enabled", p_celex5_options_->imu_enabled_);
  CeleX5DdyConfigure::ReadROSParam(nh_, "ISO_level", p_celex5_options_->ISO_level_);
  // readIntParam("contrast", p_celex5_options_->contrast_);
  CeleX5DdyConfigure::ReadROSParam(nh_, "clock_rate", p_celex5_options_->clock_rate_);
  CeleX5DdyConfigure::ReadROSParam(nh_, "is_loop_mode_enabled", p_celex5_options_->is_loop_mode_enabled_);
  CeleX5DdyConfigure::ReadROSParam(nh_, "event_duration_in_loop", p_celex5_options_->event_duration_in_loop_);
  CeleX5DdyConfigure::ReadROSParam(nh_, "picture_number_in_loop", p_celex5_options_->picture_number_in_loop_);
  CeleX5DdyConfigure::ReadROSParam(nh_, "event_FPN_file_path", p_celex5_options_->event_FPN_file_path_);
  CeleX5DdyConfigure::ReadROSParam(nh_, "frame_FPN_file_path", p_celex5_options_->frame_FPN_file_path_);

}

