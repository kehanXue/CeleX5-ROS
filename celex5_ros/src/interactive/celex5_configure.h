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

#ifndef CELEX5_ROS_SRC_INTERFACE_CELEX5_CONFIGURE_H_
#define CELEX5_ROS_SRC_INTERFACE_CELEX5_CONFIGURE_H_

#include <string>
#include <map>
#include <utility>

#include <ros/ros.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>

#include "celex5_display_controller.h"
#include "bean/celex5_options.h"

namespace celex5_ros {
class CeleX5Configure {

 public:
  explicit CeleX5Configure(std::shared_ptr<CeleX5> p_celex5_sensor,
                           const ros::NodeHandle &nh = ros::NodeHandle("~"));
  virtual ~CeleX5Configure();
  void UpdateCeleX5AllOptions();
  void PublishReconfigureServices();
  const std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> &GetPtrDDynRec() const;

  /*
   * Function tools to get parameters from ROS Param Server
   */
  static void ReadROSParam(const ros::NodeHandle &nh,
                           const std::string &param_name,
                           uint32_t &param);
  static void ReadROSParam(const ros::NodeHandle &nh,
                           const std::string &param_name,
                           int &param);
  static void ReadROSParam(const ros::NodeHandle &nh,
                           const std::string &param_name,
                           bool &param);
  static void ReadROSParam(const ros::NodeHandle &nh,
                           const std::string &param_name,
                           std::string &param);
 private:
  ros::NodeHandle nh_;
  std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> p_ddyn_rec_;
  std::shared_ptr<CeleX5> p_celex5_sensor_;
  std::shared_ptr<CeleX5Options> p_celex5_options_;

  /*
   * Parameters reconfigure callback
   */
  void ParamFixedModeCb(int fixed_mode);
  void ParamLoopMode1Cb(int loop_mode1);
  void ParamLoopMode2Cb(int loop_mode2);
  void ParamLoopMode3Cb(int loop_mode3);

  void ParamEventFrameTimeCb(int new_event_frame_time);
  void ParamOpticalFlowFrameTimeCb(int new_optical_flow_frame_time);

  void ParamThresholdCb(int new_threshold);
  void ParamBrightnessCb(int new_brightness);
  void ParamISOLevelCb(int new_iso_level);
  void ParamRawEventsEnabledCb(bool new_raw_events_status);
  void ParamImuEnabledCb(bool new_imu_status);
//  void ParamContrastCb(int new_contrast);
  void ParamClockRateCb(int new_clock_rate);
  void ParamLoopModeEnabledCb(bool new_loop_mode_status);
  void ParamEventDurationInLoopCb(int new_event_duration_in_loop);
  void ParamPictureNumberInLoopCb(int new_picture_number_in_loop);

  void ParamEventFpnFilePathCb(const std::string &new_fpn_file_path);
  void ParamFrameFpnFilePathCb(const std::string &new_fpn_file_path);
};
}

#endif //CELEX5_ROS_SRC_INTERFACE_CELEX5_CONFIGURE_H_
