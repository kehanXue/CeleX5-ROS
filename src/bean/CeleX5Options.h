//
// Created by kehan on 2020/1/14.
//

#ifndef CELEX5_ROS_SRC_BEAN_CELEX5OPTIONS_H_
#define CELEX5_ROS_SRC_BEAN_CELEX5OPTIONS_H_

#include <string>
#include <vector>
#include "celex5/celex5.h"

class CeleX5Options {

 public:

  static bool setCeleX5Options(CeleX5Options cele_x_5_options);

// protected:

 private:

  CeleX5::CeleX5Mode fixed_mode_;

  bool is_loop_mode_enabled_;
  std::vector<CeleX5::CeleX5Mode> loop_modes_;

  uint32_t event_duration_;
  uint32_t threshold_;
  uint32_t brightness_;
  uint32_t contrast_;
  uint32_t clock_rate_;

  std::string event_FPN_file_path_;
  std::string frame_FPN_file_path_;
};

#endif //CELEX5_ROS_SRC_BEAN_CELEX5OPTIONS_H_
