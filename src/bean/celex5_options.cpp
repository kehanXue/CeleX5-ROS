//
// Created by kehan on 2020/1/14.
//

#include "celex5_options.h"

CeleX5Options::CeleX5Options(CeleX5::CeleX5Mode fixed_mode,
                             bool is_loop_mode_enabled,
                             std::vector<CeleX5::CeleX5Mode> loop_modes,
                             uint32_t event_duration,
                             uint32_t threshold,
                             uint32_t brightness,
                             uint32_t contrast,
                             uint32_t clock_rate,
                             std::string event_fpn_file_path,
                             std::string frame_fpn_file_path)
    : fixed_mode_(fixed_mode),
      is_loop_mode_enabled_(is_loop_mode_enabled),
      loop_modes_(std::move(loop_modes)),
      event_duration_in_loop_(event_duration),
      threshold_(threshold),
      brightness_(brightness),
      contrast_(contrast),
      clock_rate_(clock_rate),
      event_FPN_file_path_(std::move(event_fpn_file_path)),
      frame_FPN_file_path_(std::move(frame_fpn_file_path)) {

}

CeleX5Options::~CeleX5Options() = default;

CeleX5::CeleX5Mode CeleX5Options::GetFixedMode() const {
  return fixed_mode_;
}

void CeleX5Options::SetFixedMode(CeleX5::CeleX5Mode fixed_mode) {
  fixed_mode_ = fixed_mode;
}

bool CeleX5Options::IsLoopModeEnabled() const {
  return is_loop_mode_enabled_;
}

void CeleX5Options::SetIsLoopModeEnabled(bool is_loop_mode_enabled) {
  is_loop_mode_enabled_ = is_loop_mode_enabled;
}

const std::vector<CeleX5::CeleX5Mode> &CeleX5Options::GetLoopModes() const {
  return loop_modes_;
}

void CeleX5Options::SetLoopModes(const std::vector<CeleX5::CeleX5Mode> &loop_modes) {
  loop_modes_ = loop_modes;
}

uint32_t CeleX5Options::GetEventDurationInLoop() const {
  return event_duration_in_loop_;
}

void CeleX5Options::SetEventDurationInLoop(uint32_t event_duration) {
  event_duration_in_loop_ = event_duration;
}

uint32_t CeleX5Options::GetThreshold() const {
  return threshold_;
}

void CeleX5Options::SetThreshold(uint32_t threshold) {
  threshold_ = threshold;
}

uint32_t CeleX5Options::GetBrightness() const {
  return brightness_;
}

void CeleX5Options::SetBrightness(uint32_t brightness) {
  brightness_ = brightness;
}

uint32_t CeleX5Options::GetContrast() const {
  return contrast_;
}

void CeleX5Options::SetContrast(uint32_t contrast) {
  contrast_ = contrast;
}

uint32_t CeleX5Options::GetClockRate() const {
  return clock_rate_;
}

void CeleX5Options::SetClockRate(uint32_t clock_rate) {
  clock_rate_ = clock_rate;
}

const std::string &CeleX5Options::GetEventFpnFilePath() const {
  return event_FPN_file_path_;
}

void CeleX5Options::SetEventFpnFilePath(const std::string &event_fpn_file_path) {
  event_FPN_file_path_ = event_fpn_file_path;
}

const std::string &CeleX5Options::GetFrameFpnFilePath() const {
  return frame_FPN_file_path_;
}

void CeleX5Options::SetFrameFpnFilePath(const std::string &frame_fpn_file_path) {
  frame_FPN_file_path_ = frame_fpn_file_path;
}

uint32_t CeleX5Options::GetEventFrameTime() const {
  return event_frame_time_;
}

void CeleX5Options::SetEventFrameTime(uint32_t event_frame_time) {
  event_frame_time_ = event_frame_time;
}

uint32_t CeleX5Options::GetPictureNumberInLoop() const {
  return picture_number_in_loop_;
}

void CeleX5Options::SetPictureNumberInLoop(uint32_t picture_number_in_loop) {
  picture_number_in_loop_ = picture_number_in_loop;
}

