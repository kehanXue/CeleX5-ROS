//
// Created by kehan on 2020/1/14.
//

#ifndef CELEX5_ROS_SRC_BEAN_CELEX5_OPTIONS_H_
#define CELEX5_ROS_SRC_BEAN_CELEX5_OPTIONS_H_

#include <string>
#include <vector>
#include "celex5/celex5.h"

namespace celex5_ros {
class CeleX5Options {

 public:

  // TODO default value
  explicit CeleX5Options(CeleX5::CeleX5Mode fixed_mode = CeleX5::CeleX5Mode::Event_Off_Pixel_Timestamp_Mode,
                         bool is_loop_mode_enabled = false,
                         std::vector<CeleX5::CeleX5Mode> loop_modes = std::vector<CeleX5::CeleX5Mode>(3),
                         uint32_t event_duration_in_loop = 100,
                         uint32_t picture_number_in_loop = 30,
                         uint32_t event_frame_time_ = 1000,
                         uint32_t optical_flow_frame_time_ = 20,
                         uint32_t threshold = 185,
                         uint32_t brightness = 150,
                         uint32_t contrast = 1,
                         uint32_t clock_rate = 100,
                         std::string event_fpn_file_path = "",
                         std::string frame_fpn_file_path = "");

  virtual ~CeleX5Options();

  CeleX5::CeleX5Mode GetFixedMode() const;
  void SetFixedMode(CeleX5::CeleX5Mode fixed_mode);

  bool IsLoopModeEnabled() const;
  void SetIsLoopModeEnabled(bool is_loop_mode_enabled);

  const std::vector<CeleX5::CeleX5Mode> &GetLoopModes() const;
  void SetLoopModes(const std::vector<CeleX5::CeleX5Mode> &loop_modes);

  uint32_t GetEventFrameTime() const;
  void SetEventFrameTime(uint32_t event_frame_time);

  uint32_t GetOpticalFlowFrameTime() const;
  void SetOpticalFlowFrameTime(uint32_t optical_flow_frame_time);

  uint32_t GetThreshold() const;
  void SetThreshold(uint32_t threshold);

  uint32_t GetBrightness() const;
  void SetBrightness(uint32_t brightness);

  uint32_t GetContrast() const;
  void SetContrast(uint32_t contrast);

  uint32_t GetClockRate() const;
  void SetClockRate(uint32_t clock_rate);

  uint32_t GetEventDurationInLoop() const;
  void SetEventDurationInLoop(uint32_t event_duration);

  uint32_t GetPictureNumberInLoop() const;
  void SetPictureNumberInLoop(uint32_t picture_number_in_loop);

  const std::string &GetEventFpnFilePath() const;
  void SetEventFpnFilePath(const std::string &event_fpn_file_path);

  const std::string &GetFrameFpnFilePath() const;
  void SetFrameFpnFilePath(const std::string &frame_fpn_file_path);

 private:
  CeleX5::CeleX5Mode fixed_mode_;
  std::vector<CeleX5::CeleX5Mode> loop_modes_;

  uint32_t event_frame_time_;
  uint32_t optical_flow_frame_time_;

  uint32_t threshold_;
  uint32_t brightness_;
  uint32_t contrast_;
  uint32_t clock_rate_;

  bool is_loop_mode_enabled_;
  uint32_t event_duration_in_loop_;
  uint32_t picture_number_in_loop_;

  std::string event_FPN_file_path_;
  std::string frame_FPN_file_path_;
};
}

#endif //CELEX5_ROS_SRC_BEAN_CELEX5_OPTIONS_H_
