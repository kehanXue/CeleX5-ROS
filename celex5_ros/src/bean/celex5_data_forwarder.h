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

#ifndef CELEX5_ROS_SRC_BEAN_CELEX5_DATA_FORWARDER_H_
#define CELEX5_ROS_SRC_BEAN_CELEX5_DATA_FORWARDER_H_

#include <vector>
#include <mutex>
#include <thread>
#include <condition_variable>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "celex5_msgs/Event.h"
#include "celex5_msgs/EventVector.h"
#include "celex5_msgs/Imu.h"
#include "celex5_msgs/ImuVector.h"

#include "interactive/celex5_ddy_configure.h"
#include "bean/celex5_options.h"
#include "bean/camera_publisher.h"

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
  void CreatePubThreads();
  void CreateRawEventsPubThread();
  void CreatePolarityImgPubThread();
  void CreateImuDataPubThread();

  std::condition_variable cv_raw_events_;
  std::condition_variable cv_polarity_img_;
  std::condition_variable cv_imu_data_;
  std::mutex mu_raw_events_;
  std::mutex mu_polarity_img_;
  std::mutex mu_imu_data_;

  std::vector<EventData> vec_events_;
  std::vector<EventData> vec_raw_events_;
  std::vector<EventData> vec_polarity_events_;

  std::shared_ptr<std::thread> p_raw_events_pub_thread_;
  std::shared_ptr<std::thread> p_polarity_img_pub_thread_;
  std::shared_ptr<std::thread> p_imu_data_pub_thread_;

  ros::NodeHandle nh_;
  std::string frame_id_;

  std::shared_ptr<CeleX5> p_celex5_sensor_;
  std::shared_ptr<CeleX5Options> p_celex5_options_;

  CX5SensorDataServer *p_celex5_data_server_;

  ros::Publisher events_pub_;
  ros::Publisher celex_imu_pub_;
  ros::Publisher imu_pub_;
  ros::Publisher dvs_events_pub_;

  std::shared_ptr<celex5_ros::CameraPublisher> p_polarity_img_pub_;
  // ros::Publisher binary_img_pub_;
  // ros::Publisher denoised_img_pub_;
  // ros::Publisher count_img_pub_;
  // ros::Publisher optical_flow_img_pub_;
  //
  // ros::Publisher gray_img_pub_;
  // ros::Publisher accumulated_img_pub_;
  // ros::Publisher superimposed_img_pub_;
  // ros::Publisher optical_flow_direction_img_pub_;
  // ros::Publisher optical_flow_speed_img_pub_;
};
}

#endif //CELEX5_ROS_SRC_BEAN_CELEX5_DATA_FORWARDER_H_
