//
// Created by kehan on 2020/1/13.
//

#include "celex5_nodelet.h"

using namespace celex5_ros;

CeleX5Nodelet::CeleX5Nodelet()
    : nh_("~") {

  p_celex5_options_ = std::make_shared<CeleX5Options>();
  p_celex5_sensor_ = std::make_shared<CeleX5>();
  p_celex5_configure_ =
      std::make_shared<CeleX5Configure>(p_celex5_options_, p_celex5_sensor_, nh_);

  /*
   * Read parameters from ROS param server
   */
  auto readIntParam = [=](const std::string &param_name, uint32_t &param) {
    if (nh_.hasParam(param_name)) {
      int tmp_param = 0;
      nh_.getParam(param_name, tmp_param);
      param = static_cast<uint32_t>(tmp_param);
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

  auto readBoolParam = [=](const std::string &param_name, bool &param) {
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

  auto readStringParam = [=](const std::string &param_name, std::string &param) {
    if (nh_.hasParam(param_name)) {
      nh_.getParam(param_name, param);
      ROS_INFO("%s: Use the param %s: %s",
               ros::this_node::getName().c_str(),
               param_name.c_str(),
               param.c_str());
    } else {
      ROS_WARN("%s: Didn't provide param %s, use the default value: %s",
               ros::this_node::getName().c_str(),
               param_name.c_str(),
               param.c_str());
    }
  };

  uint32_t tmp_celex5_mode_param = -1;
  readIntParam("fixed_mode", tmp_celex5_mode_param);
  p_celex5_options_->SetFixedMode(static_cast<CeleX5::CeleX5Mode>(tmp_celex5_mode_param));
  readIntParam("loop_mode1", tmp_celex5_mode_param);
  p_celex5_options_->SetLoopMode1(static_cast<CeleX5::CeleX5Mode>(tmp_celex5_mode_param));
  readIntParam("loop_mode2", tmp_celex5_mode_param);
  p_celex5_options_->SetLoopMode2(static_cast<CeleX5::CeleX5Mode>(tmp_celex5_mode_param));
  readIntParam("loop_mode3", tmp_celex5_mode_param);
  p_celex5_options_->SetLoopMode3(static_cast<CeleX5::CeleX5Mode>(tmp_celex5_mode_param));

  readIntParam("event_frame_time", p_celex5_options_->event_frame_time_);
  readIntParam("optical_flow_frame_time", p_celex5_options_->optical_flow_frame_time_);
  readIntParam("threshold", p_celex5_options_->threshold_);
  readIntParam("brightness", p_celex5_options_->brightness_);
  // readIntParam("contrast", p_celex5_options_->contrast_);
  readIntParam("clock_rate", p_celex5_options_->clock_rate_);
  readBoolParam("is_loop_mode_enabled", p_celex5_options_->is_loop_mode_enabled_);
  readIntParam("event_duration_in_loop", p_celex5_options_->event_duration_in_loop_);
  readIntParam("picture_number_in_loop", p_celex5_options_->picture_number_in_loop_);
  readStringParam("event_FPN_file_path", p_celex5_options_->event_FPN_file_path_);
  readStringParam("frame_FPN_file_path", p_celex5_options_->frame_FPN_file_path_);


}

CeleX5Nodelet::~CeleX5Nodelet() = default;

PLUGINLIB_EXPORT_CLASS(celex5_ros::CeleX5Nodelet, nodelet::Nodelet)
