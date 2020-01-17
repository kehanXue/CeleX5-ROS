//
// Created by kehan on 2020/1/17.
//

#ifndef CELEX5_ROS_SRC_BEAN_CELEX5_DATA_FORWARDER_H_
#define CELEX5_ROS_SRC_BEAN_CELEX5_DATA_FORWARDER_H_

#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "celex5_msgs/event.h"
#include "celex5_msgs/eventVector.h"

#include "celex5/celex5datamanager.h"

namespace celex5_ros {

const int CELEX5_MAT_ROWS = 800;
const int CELEX5_MAT_COLS = 1280;

class CeleX5DataForwarder : public CeleX5DataManager {

 public:
  explicit CeleX5DataForwarder(const ros::NodeHandle &nh,
                               const std::shared_ptr<CeleX5> &p_celex5_sensor);
  virtual ~CeleX5DataForwarder();

 private:
  void onFrameDataUpdated(CeleX5ProcessedData *processed_data) override;

  ros::NodeHandle nh_;
  std::shared_ptr<CeleX5> p_celex5_sensor_;
  CX5SensorDataServer *p_celex5_data_server_;

  ros::Publisher events_pub_;
  ros::Publisher imu_pub_;
  ros::Publisher mag_pub_;

  ros::Publisher event_binary_img_pub_;
  ros::Publisher event_denoised_img_pub_;
  ros::Publisher event_count_img_pub_;
  ros::Publisher event_optical_flow_img_pub_;

  ros::Publisher event_gray_img_pub_;
  ros::Publisher event_accumulated_img_pub_;
  ros::Publisher event_superimposed_img_pub_;
  ros::Publisher event_optical_flow_direction_img_pub_;
  ros::Publisher event_optical_flow_speed_img_pub_;
};
}

#endif //CELEX5_ROS_SRC_BEAN_CELEX5_DATA_FORWARDER_H_
