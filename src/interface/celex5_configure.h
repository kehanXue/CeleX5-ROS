//
// Created by kehan on 2020/1/14.
//

#ifndef CELEX5_ROS_SRC_INTERFACE_CELEX5_CONFIGURE_H_
#define CELEX5_ROS_SRC_INTERFACE_CELEX5_CONFIGURE_H_

#include <string>
#include <map>
#include <utility>

#include <ros/ros.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>

#include "bean/celex5_display_controller.h"
#include "bean/celex5_options.h"

namespace celex5_ros {
class CeleX5Configure {

 public:
  explicit CeleX5Configure(std::shared_ptr<CeleX5Options> p_celex5_options,
                           std::shared_ptr<CeleX5> p_celex5_sensor,
                           const ros::NodeHandle &nh = ros::NodeHandle("~"));
  virtual ~CeleX5Configure();
  void UpdateCeleX5AllOptions();
  void PublishReconfigureServices();
  const std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> &GetPtrDDynRec() const;
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
//  void ParamContrastCb(int new_contrast);
  void ParamClockRateCb(int new_clock_rate);
  void ParamIsLoopModeEnabled(bool new_loop_mode_status);
  void ParamEventDurationInLoopCb(int new_event_duration_in_loop);
  void ParamPictureNumberInLoopCb(int new_picture_number_in_loop);

  void ParamEventFpnFilePathCb(const std::string &new_fpn_file_path);
  void ParamFrameFpnFilePathCb(const std::string &new_fpn_file_path);
};
}

#endif //CELEX5_ROS_SRC_INTERFACE_CELEX5_CONFIGURE_H_
