//
// Created by kehan on 2020/1/18.
//

#include "celex5_ros_bean.h"

using namespace celex5_ros;

celex5_ros::CeleX5ROSBean::CeleX5ROSBean(const ros::NodeHandle &nh)
    : nh_(nh) {
  // nh_ = ros::NodeHandle("~");
  p_celex5_options_ = std::make_shared<CeleX5Options>();
  p_celex5_sensor_ = std::make_shared<CeleX5>();
  p_celex5_configure_ =
      std::make_shared<CeleX5Configure>(p_celex5_options_, p_celex5_sensor_, nh_);
}

celex5_ros::CeleX5ROSBean::~CeleX5ROSBean() = default;

void celex5_ros::CeleX5ROSBean::Run() {
  /*
   * Open CeleX5 Sensor
   */
  p_celex5_sensor_->openSensor(CeleX5::CeleX5_MIPI);
  ROS_INFO("Sensor status: %d", p_celex5_sensor_->isSensorReady());

  /*
   * Read initial parameters from ROS Param Server
   */
  ReadParams();

  /*
   * Create CeleX5DataForwarder handle to forward full stream data from CeleX5 Sensor to ROS framework
   */
  p_celex5_data_forwarder_ = std::make_shared<CeleX5DataForwarder>(nh_, p_celex5_sensor_);

  /*
   * Provide display function. Can change the display fps in rqt_reconfigure
   */
  p_celex5_display_ = CeleX5DisplayController::GetInstance(nh_,
                                                           p_celex5_sensor_,
                                                           p_celex5_configure_->GetPtrDDynRec());
  p_celex5_display_->SetCeleX5Mode(p_celex5_sensor_->getSensorFixedMode());

  /*
   * Publish dynamic reconfigure services
   */
  p_celex5_configure_->PublishReconfigureServices();
}

void celex5_ros::CeleX5ROSBean::ReadParams() {
  /*
   * Read parameters from ROS param server
   */

  uint32_t tmp_celex5_mode_param = p_celex5_options_->GetFixedMode();
  CeleX5Configure::ReadROSParam(nh_, "fixed_mode", tmp_celex5_mode_param);
  p_celex5_options_->SetFixedMode(static_cast<CeleX5::CeleX5Mode>(tmp_celex5_mode_param));
  CeleX5Configure::ReadROSParam(nh_, "loop_mode1", tmp_celex5_mode_param);
  p_celex5_options_->SetLoopMode1(static_cast<CeleX5::CeleX5Mode>(tmp_celex5_mode_param));
  CeleX5Configure::ReadROSParam(nh_, "loop_mode2", tmp_celex5_mode_param);
  p_celex5_options_->SetLoopMode2(static_cast<CeleX5::CeleX5Mode>(tmp_celex5_mode_param));
  CeleX5Configure::ReadROSParam(nh_, "loop_mode3", tmp_celex5_mode_param);
  p_celex5_options_->SetLoopMode3(static_cast<CeleX5::CeleX5Mode>(tmp_celex5_mode_param));

  CeleX5Configure::ReadROSParam(nh_, "event_frame_time", p_celex5_options_->event_frame_time_);
  CeleX5Configure::ReadROSParam(nh_, "optical_flow_frame_time", p_celex5_options_->optical_flow_frame_time_);
  CeleX5Configure::ReadROSParam(nh_, "threshold", p_celex5_options_->threshold_);
  CeleX5Configure::ReadROSParam(nh_, "brightness", p_celex5_options_->brightness_);
  // readIntParam("contrast", p_celex5_options_->contrast_);
  CeleX5Configure::ReadROSParam(nh_, "clock_rate", p_celex5_options_->clock_rate_);
  CeleX5Configure::ReadROSParam(nh_, "is_loop_mode_enabled", p_celex5_options_->is_loop_mode_enabled_);
  CeleX5Configure::ReadROSParam(nh_, "event_duration_in_loop", p_celex5_options_->event_duration_in_loop_);
  CeleX5Configure::ReadROSParam(nh_, "picture_number_in_loop", p_celex5_options_->picture_number_in_loop_);
  CeleX5Configure::ReadROSParam(nh_, "event_FPN_file_path", p_celex5_options_->event_FPN_file_path_);
  CeleX5Configure::ReadROSParam(nh_, "frame_FPN_file_path", p_celex5_options_->frame_FPN_file_path_);

  p_celex5_configure_->UpdateCeleX5AllOptions();
}

