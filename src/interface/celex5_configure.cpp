//
// Created by kehan on 2020/1/14.
//

#include "celex5_configure.h"

using namespace celex5_ros;

CeleX5Configure::CeleX5Configure(
    std::shared_ptr<CeleX5Options> p_celex5_options,
    const ros::NodeHandle &nh)
    : nh_(nh),
      p_celex5_options_(std::move(p_celex5_options)) {

  // Associate with enum CeleX5::CeleX5Mode
  std::map<std::string, int> map_enum_celex5_mode = {
      {"Event_Off_Pixel_Timestamp_Mode", 0},
      {"Event_In_Pixel_Timestamp_Mode", 1},
      {"Event_Intensity_Mode", 2},
      {"Full_Picture_Mode", 3},
      {"Optical_Flow_Mode", 4},
      {"Optical_Flow_FPN_Mode", 5},
      {"Multi_Read_Optical_Flow_Mode", 6}
  };

  ddynamic_reconfigure_.registerEnumVariable<int>
      ("fixed_mode", static_cast<int>(p_celex5_options_->GetFixedMode()),
       boost::bind(&CeleX5Configure::ParamFixedModeCb, this, _1),
       "Fixed mode of the CeleX5 event camera.",
       map_enum_celex5_mode);

  ddynamic_reconfigure_.registerEnumVariable<int>
      ("loop_mode1", static_cast<int>(p_celex5_options_->GetLoopModes().at(0)),
       boost::bind(&CeleX5Configure::ParamLoopMode1Cb, this, _1),
       "Loop mode 1 of the CeleX5 event camera.",
       map_enum_celex5_mode);

  ddynamic_reconfigure_.registerEnumVariable<int>
      ("loop_mode2", static_cast<int>(p_celex5_options_->GetLoopModes().at(1)),
       boost::bind(&CeleX5Configure::ParamLoopMode2Cb, this, _1),
       "Loop mode 2 of the CeleX5 event camera.",
       map_enum_celex5_mode);

  ddynamic_reconfigure_.registerEnumVariable<int>
      ("loop_mode3", static_cast<int>(p_celex5_options_->GetLoopModes().at(2)),
       boost::bind(&CeleX5Configure::ParamLoopMode3Cb, this, _1),
       "Loop mode 3 of the CeleX5 event camera.",
       map_enum_celex5_mode);


  // TODO range
  ddynamic_reconfigure_.registerVariable<int>
      ("event_frame_time", p_celex5_options_->GetEventFrameTime(),
       boost::bind(&CeleX5Configure::ParamEventFrameTimeCb, this, _1),
       "The frame time of Event Mode, unit is ms. It modifies\n"
       "the frame length when the software creates event frames without changing the hardware\n"
       "parameters.",
       1,
       10000);

  // TODO range
  ddynamic_reconfigure_.registerVariable<int>
      ("optical_flow_frame_time", p_celex5_options_->GetOpticalFlowFrameTime(),
       boost::bind(&CeleX5Configure::ParamOpticalFlowFrameTimeCb, this, _1),
       "The time of generating a full-frame optical-flow picture, unit is ms.",
       1,
       10000);

  ddynamic_reconfigure_.registerVariable<int>
      ("threshold", p_celex5_options_->GetThreshold(),
       boost::bind(&CeleX5Configure::ParamThresholdCb, this, _1),
       "The threshold value where the event triggers",
       50,
       511);

  ddynamic_reconfigure_.registerVariable<int>
      ("brightness", p_celex5_options_->GetBrightness(),
       boost::bind(&CeleX5Configure::ParamBrightnessCb, this, _1),
       "Controls the brightness of the image CeleX-5 sensor generated",
       0,
       1023);

  ddynamic_reconfigure_.registerVariable<int>
      ("contrast", p_celex5_options_->GetContrast(),
       boost::bind(&CeleX5Configure::ParamContrastCb, this, _1),
       "Controls the contrast of the image CeleX-5 sensor generated.",
       1,
       3);

  ddynamic_reconfigure_.registerVariable<int>
      ("clock_rate", p_celex5_options_->GetClockRate(),
       boost::bind(&CeleX5Configure::ParamClockRateCb, this, _1),
       "The clock rate of the CeleX-5 sensor, unit is MHz, step is 10.",
       20,
       100);

  ddynamic_reconfigure_.registerVariable<bool>
      ("is_loop_mode_enabled", p_celex5_options_->IsLoopModeEnabled(),
       boost::bind(&CeleX5Configure::ParamIsLoopModeEnabled, this, _1),
       "Whether enable loop mode.");

  // TODO range
  ddynamic_reconfigure_.registerVariable<int>
      ("event_duration_in_loop", p_celex5_options_->GetEventDurationInLoop(),
       boost::bind(&CeleX5Configure::ParamEventDurationInLoopCb, this, _1),
       "The time duration of working in the Event Mode when in Loop Mode.",
       1,
       200);

  ddynamic_reconfigure_.registerVariable<int>
      ("picture_number_in_loop", p_celex5_options_->GetPictureNumberInLoop(),
       boost::bind(&CeleX5Configure::ParamPictureNumberInLoopCb, this, _1),
       "The picture number of working in the Full-frame Mode when the CeleX-5\n"
       "sensor is in the loop mode.",
       1,
       200);

  ddynamic_reconfigure_.registerVariable<std::string>
      ("event_FPN_file_path", p_celex5_options_->GetEventFpnFilePath(),
       boost::bind(&CeleX5Configure::ParamEventFpnFilePathCb, this, _1),
       "The path of FPN file of Event Mode.");

  ddynamic_reconfigure_.registerVariable<std::string>
      ("frame_FPN_file_path", p_celex5_options_->GetFrameFpnFilePath(),
       boost::bind(&CeleX5Configure::ParamFrameFpnFilePathCb, this, _1),
       "The path of FPN file of Frame Mode.");

  ddynamic_reconfigure_.publishServicesTopics();
}

CeleX5Configure::~CeleX5Configure() = default;

void CeleX5Configure::ParamFixedModeCb(int fixed_mode) {
  // TODO
  p_celex5_options_->
      SetFixedMode(static_cast<CeleX5::CeleX5Mode>(fixed_mode));
}

void CeleX5Configure::ParamLoopMode1Cb(int loop_mode1) {

}

void CeleX5Configure::ParamLoopMode2Cb(int loop_mode2) {

}

void CeleX5Configure::ParamLoopMode3Cb(int loop_mode3) {

}

void CeleX5Configure::ParamEventFrameTimeCb(int new_event_frame_time) {
  p_celex5_options_->
      SetEventFrameTime(static_cast<uint32_t>(new_event_frame_time));
}

void CeleX5Configure::ParamOpticalFlowFrameTimeCb(int new_optical_flow_frame_time) {
  p_celex5_options_->
      SetOpticalFlowFrameTime(static_cast<uint32_t>(new_optical_flow_frame_time));
}

void CeleX5Configure::ParamThresholdCb(int new_threshold) {
  p_celex5_options_->
      SetThreshold(static_cast<uint32_t>(new_threshold));
}

void CeleX5Configure::ParamBrightnessCb(int new_brightness) {
  p_celex5_options_->
      SetBrightness(static_cast<uint32_t>(new_brightness));
}

void CeleX5Configure::ParamContrastCb(int new_contrast) {
  p_celex5_options_->
      SetContrast(static_cast<uint32_t>(new_contrast));
}

void CeleX5Configure::ParamClockRateCb(int new_clock_rate) {
  p_celex5_options_->
      SetClockRate(static_cast<uint32_t>(new_clock_rate));
}

void CeleX5Configure::ParamIsLoopModeEnabled(bool new_loop_mode_status) {
  p_celex5_options_->
      SetIsLoopModeEnabled(new_loop_mode_status);
}

void CeleX5Configure::ParamEventDurationInLoopCb(int new_event_duration_in_loop) {
  p_celex5_options_->
      SetEventDurationInLoop(static_cast<uint32_t>(new_event_duration_in_loop));
}

void CeleX5Configure::ParamPictureNumberInLoopCb(int new_picture_number_in_loop) {
  p_celex5_options_->
      SetPictureNumberInLoop(static_cast<uint32_t>(new_picture_number_in_loop));
}

void CeleX5Configure::ParamEventFpnFilePathCb(const std::string &new_fpn_file_path) {
  p_celex5_options_->
      SetEventFpnFilePath(new_fpn_file_path);
}

void CeleX5Configure::ParamFrameFpnFilePathCb(const std::string &new_fpn_file_path) {
  p_celex5_options_->
      SetFrameFpnFilePath(new_fpn_file_path);
}


