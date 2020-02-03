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

#include "celex5_configure.h"

using namespace celex5_ros;

CeleX5Configure::CeleX5Configure(
    std::shared_ptr<CeleX5> p_celex5_sensor,
    const ros::NodeHandle &nh)
    : nh_(nh),
      p_celex5_sensor_(std::move(p_celex5_sensor)) {

  p_celex5_options_ = CeleX5Options::GetInstance();

  ros::NodeHandle nh_config = ros::NodeHandle(nh_, "sensor");
  p_ddyn_rec_ = std::make_shared<ddynamic_reconfigure::DDynamicReconfigure>(nh_config);

  // Associate with enum CeleX5::CeleX5Mode
  std::map<std::string, int> map_enum_celex5_fixed_mode = {
      {"Event_Off_Pixel_Timestamp_Mode", 0},
      {"Event_In_Pixel_Timestamp_Mode", 1},
      {"Event_Intensity_Mode", 2},
      {"Full_Picture_Mode", 3},
      {"Optical_Flow_Mode", 4},
      // TODO
      // {"Optical_Flow_FPN_Mode", 5},
      // {"Multi_Read_Optical_Flow_Mode", 6}
  };

  p_ddyn_rec_->registerEnumVariable<int>
      ("fixed_mode", static_cast<int>(p_celex5_options_->GetFixedMode()),
       boost::bind(&CeleX5Configure::ParamFixedModeCb, this, _1),
       "Fixed mode of the CeleX5 event camera.",
       map_enum_celex5_fixed_mode);

  /*
   * Loop mode suggestions follow the doc which CeleX^TM provide
   */
  // Associate with enum CeleX5::CeleX5Mode
  std::map<std::string, int> map_enum_celex5_loop_mode1 = {
      {"Full_Picture_Mode", 3},
  };
  p_ddyn_rec_->registerEnumVariable<int>
      ("loop_mode1", static_cast<int>(p_celex5_options_->GetLoopModes().at(0)),
       boost::bind(&CeleX5Configure::ParamLoopMode1Cb, this, _1),
       "Loop mode 1 of the CeleX5 event camera.",
       map_enum_celex5_loop_mode1);

  // Associate with enum CeleX5::CeleX5Mode
  std::map<std::string, int> map_enum_celex5_loop_mode2 = {
      {"Event_Off_Pixel_Timestamp_Mode", 0},
      {"Event_In_Pixel_Timestamp_Mode", 1},
      {"Event_Intensity_Mode", 2},
  };
  p_ddyn_rec_->registerEnumVariable<int>
      ("loop_mode2", static_cast<int>(p_celex5_options_->GetLoopModes().at(1)),
       boost::bind(&CeleX5Configure::ParamLoopMode2Cb, this, _1),
       "Loop mode 2 of the CeleX5 event camera.",
       map_enum_celex5_loop_mode2);

  // Associate with enum CeleX5::CeleX5Mode
  std::map<std::string, int> map_enum_celex5_loop_mode3 = {
      {"Optical_Flow_Mode", 4},
      {"Optical_Flow_FPN_Mode", 5},
      {"Multi_Read_Optical_Flow_Mode", 6}
  };
  p_ddyn_rec_->registerEnumVariable<int>
      ("loop_mode3", static_cast<int>(p_celex5_options_->GetLoopModes().at(2)),
       boost::bind(&CeleX5Configure::ParamLoopMode3Cb, this, _1),
       "Loop mode 3 of the CeleX5 event camera.",
       map_enum_celex5_loop_mode3);

  p_ddyn_rec_->registerVariable<int>
      ("event_frame_time", p_celex5_options_->GetEventFrameTime(),
       boost::bind(&CeleX5Configure::ParamEventFrameTimeCb, this, _1),
       "The frame time of Event Mode, unit is ms. It modifies\n"
       "the frame length when the software creates event frames without changing the hardware\n"
       "parameters.",
       1,
       1000000);

  p_ddyn_rec_->registerVariable<int>
      ("optical_flow_frame_time", p_celex5_options_->GetOpticalFlowFrameTime(),
       boost::bind(&CeleX5Configure::ParamOpticalFlowFrameTimeCb, this, _1),
       "The time of generating a full-frame optical-flow picture, unit is ms.",
       1,
       10000);

  p_ddyn_rec_->registerVariable<int>
      ("threshold", p_celex5_options_->GetThreshold(),
       boost::bind(&CeleX5Configure::ParamThresholdCb, this, _1),
       "The threshold value where the event triggers",
       50,
       511);

  p_ddyn_rec_->registerVariable<int>
      ("brightness", p_celex5_options_->GetBrightness(),
       boost::bind(&CeleX5Configure::ParamBrightnessCb, this, _1),
       "Controls the brightness of the image CeleX-5 sensor generated",
       0,
       1023);

  p_ddyn_rec_->registerVariable<int>
      ("ISO_level", p_celex5_options_->GetIsoLevel(),
       boost::bind(&CeleX5Configure::ParamISOLevelCb, this, _1),
       "Controls the ISO level of the image CeleX-5 sensor generated",
       1,
       4);

  p_ddyn_rec_->registerVariable<bool>
      ("imu_enabled", p_celex5_options_->IsImuEnabled(),
       boost::bind(&CeleX5Configure::ParamImuEnabledCb, this, _1),
       "Enable the IMU data output or not");

  p_ddyn_rec_->registerVariable<bool>
      ("raw_events_enabled", p_celex5_options_->IsRawEventsEnabled(),
       boost::bind(&CeleX5Configure::ParamRawEventsEnabledCb, this, _1),
       "Enable the Raw Events data output or not");

  p_ddyn_rec_->registerVariable<bool>
      ("polarity_img_enabled", p_celex5_options_->IsPolarityImgEnabled(),
       boost::bind(&CeleX5Configure::ParamPolarityImgEnabledCb, this, _1),
       "Enable the Polarity Img publish or not");

//  p_ddyn_rec_.registerVariable<int>
//      ("contrast", p_celex5_options_->GetContrast(),
//       boost::bind(&CeleX5Configure::ParamContrastCb, this, _1),
//       "Controls the contrast of the image CeleX-5 sensor generated.",
//       1,
//       3);

  p_ddyn_rec_->registerVariable<int>
      ("clock_rate", p_celex5_options_->GetClockRate(),
       boost::bind(&CeleX5Configure::ParamClockRateCb, this, _1),
       "The clock rate of the CeleX-5 sensor, unit is MHz, step is 10.",
       20,
       100);

  p_ddyn_rec_->registerVariable<bool>
      ("is_loop_mode_enabled", p_celex5_options_->IsLoopModeEnabled(),
       boost::bind(&CeleX5Configure::ParamLoopModeEnabledCb, this, _1),
       "Whether enable loop mode.");

  // TODO range
  p_ddyn_rec_->registerVariable<int>
      ("event_duration_in_loop", p_celex5_options_->GetEventDurationInLoop(),
       boost::bind(&CeleX5Configure::ParamEventDurationInLoopCb, this, _1),
       "The time duration of working in the Event Mode when in Loop Mode.",
       1,
       200);

  p_ddyn_rec_->registerVariable<int>
      ("picture_number_in_loop", p_celex5_options_->GetPictureNumberInLoop(),
       boost::bind(&CeleX5Configure::ParamPictureNumberInLoopCb, this, _1),
       "The picture number of working in the Full-frame Mode when the CeleX-5\n"
       "sensor is in the loop mode.",
       1,
       200);

  p_ddyn_rec_->registerVariable<std::string>
      ("event_FPN_file_path", p_celex5_options_->GetEventFpnFilePath(),
       boost::bind(&CeleX5Configure::ParamEventFpnFilePathCb, this, _1),
       "The path of FPN file of Event Mode.");

  p_ddyn_rec_->registerVariable<std::string>
      ("frame_FPN_file_path", p_celex5_options_->GetFrameFpnFilePath(),
       boost::bind(&CeleX5Configure::ParamFrameFpnFilePathCb, this, _1),
       "The path of FPN file of Frame Mode.");

  p_ddyn_rec_->publishServicesTopics();
}

CeleX5Configure::~CeleX5Configure() = default;

void CeleX5Configure::UpdateCeleX5AllOptions() {

  p_celex5_sensor_->setSensorFixedMode(p_celex5_options_->GetFixedMode());
  usleep(5000);

  p_celex5_sensor_->setLoopModeEnabled(p_celex5_options_->IsLoopModeEnabled());
  usleep(5000);
  if (p_celex5_options_->IsLoopModeEnabled()) {
    p_celex5_sensor_->setSensorLoopMode(p_celex5_options_->GetLoopModes().at(0), 1);
    usleep(5000);
    p_celex5_sensor_->setSensorLoopMode(p_celex5_options_->GetLoopModes().at(1), 2);
    usleep(5000);
    p_celex5_sensor_->setSensorLoopMode(p_celex5_options_->GetLoopModes().at(2), 3);
    usleep(5000);
  }

  p_celex5_sensor_->setEventFrameTime(p_celex5_options_->GetEventFrameTime());
  usleep(5000);
  p_celex5_sensor_->setOpticalFlowFrameTime(p_celex5_options_->GetOpticalFlowFrameTime());
  usleep(5000);
  p_celex5_sensor_->setThreshold(p_celex5_options_->GetThreshold());
  usleep(5000);
  p_celex5_sensor_->setBrightness(p_celex5_options_->GetBrightness());
  usleep(5000);
  p_celex5_sensor_->setISOLevel(p_celex5_options_->GetIsoLevel());
  usleep(5000);
  // p_celex5_sensor_->setContrast(p_celex5_options_->GetContrast());
  p_celex5_sensor_->setClockRate(p_celex5_options_->GetClockRate());
  usleep(5000);

  p_celex5_sensor_->setEventDuration(p_celex5_options_->GetEventDurationInLoop());
  usleep(5000);
  p_celex5_sensor_->setPictureNumber(p_celex5_options_->GetPictureNumberInLoop(),
                                     p_celex5_options_->GetLoopModes().at(1));
  usleep(5000);
  if (p_celex5_options_->IsImuEnabled()) {
    p_celex5_sensor_->enableIMUModule();
  } else {
    p_celex5_sensor_->disableIMUModule();
  }
}

void CeleX5Configure::ParamFixedModeCb(int fixed_mode) {
  auto mode = static_cast<CeleX5::CeleX5Mode>(fixed_mode);
  if (!p_celex5_sensor_->isLoopModeEnabled()) {
    if (mode==CeleX5::Full_Picture_Mode) {
      p_celex5_sensor_->setFpnFile(p_celex5_options_->GetFrameFpnFilePath());
    } else {
      p_celex5_sensor_->setFpnFile(p_celex5_options_->GetEventFpnFilePath());
    }
    usleep(5000);
    p_celex5_sensor_->setSensorFixedMode(mode);
    usleep(5000);
    /*
    * Change the Display Controller Options
    * TODO Open a thread in Controller class to monitor
    */
    CeleX5DisplayController::GetInstance(
        nh_, p_celex5_sensor_)->SetCeleX5Mode(mode);
  }
  /*
   * Update the Global CeleX5 Option
   */
  p_celex5_options_->SetFixedMode(mode);

}

void CeleX5Configure::ParamLoopMode1Cb(int loop_mode1) {
  auto mode = static_cast<CeleX5::CeleX5Mode>(loop_mode1);
  p_celex5_sensor_->setSensorLoopMode(mode, 1);
  p_celex5_options_->SetLoopMode1(mode);
  CeleX5DisplayController::GetInstance(
      nh_, p_celex5_sensor_)->SetCeleX5Mode(mode);
}

void CeleX5Configure::ParamLoopMode2Cb(int loop_mode2) {
  auto mode = static_cast<CeleX5::CeleX5Mode>(loop_mode2);
  p_celex5_sensor_->setSensorLoopMode(mode, 2);
  p_celex5_options_->SetLoopMode2(mode);
  CeleX5DisplayController::GetInstance(
      nh_, p_celex5_sensor_)->SetCeleX5Mode(mode);
}

void CeleX5Configure::ParamLoopMode3Cb(int loop_mode3) {
  auto mode = static_cast<CeleX5::CeleX5Mode>(loop_mode3);
  p_celex5_sensor_->setSensorLoopMode(mode, 3);
  p_celex5_options_->SetLoopMode3(mode);
  CeleX5DisplayController::GetInstance(
      nh_, p_celex5_sensor_)->SetCeleX5Mode(mode);
}

void CeleX5Configure::ParamEventFrameTimeCb(int new_event_frame_time) {
  auto value = static_cast<uint32_t>(new_event_frame_time);
  p_celex5_sensor_->setEventFrameTime(value);
  p_celex5_options_->SetEventFrameTime(value);
}

void CeleX5Configure::ParamOpticalFlowFrameTimeCb(int new_optical_flow_frame_time) {
  auto value = static_cast<uint32_t>(new_optical_flow_frame_time);
  p_celex5_sensor_->setOpticalFlowFrameTime(value);
  p_celex5_options_->SetOpticalFlowFrameTime(value);
}

void CeleX5Configure::ParamThresholdCb(int new_threshold) {
  auto value = static_cast<uint32_t>(new_threshold);
  p_celex5_sensor_->setThreshold(value);
  p_celex5_options_->SetThreshold(value);
}

void CeleX5Configure::ParamBrightnessCb(int new_brightness) {
  auto value = static_cast<uint32_t>(new_brightness);
  p_celex5_sensor_->setBrightness(value);
  p_celex5_options_->SetBrightness(value);
}

//void CeleX5Configure::ParamContrastCb(int new_contrast) {
//  p_celex5_options_->
//      SetContrast(static_cast<uint32_t>(new_contrast));
//}

void CeleX5Configure::ParamClockRateCb(int new_clock_rate) {
  auto value = static_cast<uint32_t>(new_clock_rate);
  p_celex5_sensor_->setClockRate(value);
  p_celex5_options_->SetClockRate(value);
}

void CeleX5Configure::ParamLoopModeEnabledCb(bool new_loop_mode_status) {
  p_celex5_sensor_->setLoopModeEnabled(new_loop_mode_status);
  p_celex5_options_->SetIsLoopModeEnabled(new_loop_mode_status);

  if (new_loop_mode_status) {
    p_celex5_sensor_->setSensorLoopMode(p_celex5_options_->GetLoopModes().at(0), 1);
    usleep(5000);
    p_celex5_sensor_->setSensorLoopMode(p_celex5_options_->GetLoopModes().at(1), 2);
    usleep(5000);
    p_celex5_sensor_->setSensorLoopMode(p_celex5_options_->GetLoopModes().at(2), 3);;
    usleep(5000);
    CeleX5DisplayController::GetInstance(
        nh_, p_celex5_sensor_
    )->SetCeleX5Mode(p_celex5_options_->GetLoopModes().at(1));
  } else {
    CeleX5DisplayController::GetInstance(
        nh_, p_celex5_sensor_)->SetCeleX5Mode(p_celex5_options_->GetFixedMode());
  }

}

void CeleX5Configure::ParamEventDurationInLoopCb(int new_event_duration_in_loop) {
  auto value = static_cast<uint32_t>(new_event_duration_in_loop);
  p_celex5_sensor_->setEventDuration(value);
  p_celex5_options_->SetEventDurationInLoop(value);
}

void CeleX5Configure::ParamPictureNumberInLoopCb(int new_picture_number_in_loop) {
  auto value = static_cast<uint32_t>(new_picture_number_in_loop);
  p_celex5_sensor_->setPictureNumber(value, p_celex5_options_->GetLoopModes().at(1));
  p_celex5_options_->SetPictureNumberInLoop(value);
}

void CeleX5Configure::ParamEventFpnFilePathCb(const std::string &new_fpn_file_path) {
  if (p_celex5_sensor_->getSensorFixedMode()!=CeleX5::Full_Picture_Mode
      || p_celex5_sensor_->isLoopModeEnabled()) {
    p_celex5_sensor_->setFpnFile(new_fpn_file_path);
  }
  p_celex5_options_->SetEventFpnFilePath(new_fpn_file_path);
}

void CeleX5Configure::ParamFrameFpnFilePathCb(const std::string &new_fpn_file_path) {
  if (p_celex5_sensor_->getSensorFixedMode()==CeleX5::Full_Picture_Mode
      || p_celex5_sensor_->isLoopModeEnabled()) {
    p_celex5_sensor_->setFpnFile(new_fpn_file_path);
  }
  p_celex5_options_->SetFrameFpnFilePath(new_fpn_file_path);
}

void CeleX5Configure::PublishReconfigureServices() {
  p_ddyn_rec_->publishServicesTopics();
}

const std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> &CeleX5Configure::GetPtrDDynRec() const {
  return p_ddyn_rec_;
}

void CeleX5Configure::ReadROSParam(const ros::NodeHandle &nh, const std::string &param_name, uint32_t &param) {
  if (nh.hasParam(param_name)) {
    int tmp_param = 0;
    nh.getParam(param_name, tmp_param);
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
}

void CeleX5Configure::ReadROSParam(const ros::NodeHandle &nh, const std::string &param_name, int &param) {
  if (nh.hasParam(param_name)) {
    nh.getParam(param_name, param);
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
}

void CeleX5Configure::ReadROSParam(const ros::NodeHandle &nh, const std::string &param_name, bool &param) {
  if (nh.hasParam(param_name)) {
    nh.getParam(param_name, param);
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
}

void CeleX5Configure::ReadROSParam(const ros::NodeHandle &nh, const std::string &param_name, std::string &param) {
  if (nh.hasParam(param_name)) {
    nh.getParam(param_name, param);
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
}

void CeleX5Configure::ParamISOLevelCb(int new_iso_level) {
  p_celex5_sensor_->setISOLevel(new_iso_level);
  p_celex5_options_->SetIsoLevel(new_iso_level);
}

void CeleX5Configure::ParamImuEnabledCb(bool new_imu_status) {
  if (new_imu_status) {
    p_celex5_sensor_->enableIMUModule();
  } else {
    p_celex5_sensor_->disableIMUModule();
  }
}

void CeleX5Configure::ParamRawEventsEnabledCb(bool new_raw_events_status) {
  p_celex5_options_->SetRawEventsEnabled(new_raw_events_status);
}

void CeleX5Configure::ParamPolarityImgEnabledCb(bool new_raw_events_status) {
  p_celex5_options_->SetPolarityImgEnabled(new_raw_events_status);
}




