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

#include "celex5_display_controller.h"

#include <utility>

using namespace celex5_ros;

CeleX5DisplayController::CeleX5DisplayController(const ros::NodeHandle &nh,
                                                 std::shared_ptr<CeleX5> p_celex5_sensor)
    : p_celex5_sensor_(std::move(p_celex5_sensor)) {

  nh_ = ros::NodeHandle(nh, "display");
  p_ddyn_rec_ = std::make_shared<ddynamic_reconfigure::DDynamicReconfigure>(nh_);

  p_binary_img_pub_ =
      std::make_shared<CeleX5DisplayPubFactory>(nh_,
                                                "binary_img",
                                                p_celex5_sensor_,
                                                p_ddyn_rec_,
                                                CeleX5::EventBinaryPic);
  p_in_pixel_img_pub_ =
      std::make_shared<CeleX5DisplayPubFactory>(nh_,
                                                "in_pixel_img",
                                                p_celex5_sensor_,
                                                p_ddyn_rec_,
                                                CeleX5::EventInPixelTimestampPic);
  p_denoised_binary_img_pub_ =
      std::make_shared<CeleX5DisplayPubFactory>(nh_,
                                                "denoised_binary_img",
                                                p_celex5_sensor_,
                                                p_ddyn_rec_,
                                                CeleX5::EventDenoisedBinaryPic);
  p_count_img_pub_ =
      std::make_shared<CeleX5DisplayPubFactory>(nh_, "count_img",
                                                p_celex5_sensor_,
                                                p_ddyn_rec_,
                                                CeleX5::EventCountPic);
  p_optical_flow_img_pub_ =
      std::make_shared<CeleX5DisplayPubFactory>(nh_,
                                                "optical_flow_img",
                                                p_celex5_sensor_,
                                                p_ddyn_rec_,
                                                CeleX5::OpticalFlowPic);
  p_accumulated_img_pub_ =
      std::make_shared<CeleX5DisplayPubFactory>(nh_,
                                                "accumulated_img",
                                                p_celex5_sensor_,
                                                p_ddyn_rec_,
                                                CeleX5::EventAccumulatedPic);
  p_gray_img_pub_ =
      std::make_shared<CeleX5DisplayPubFactory>(nh_,
                                                "gray_img",
                                                p_celex5_sensor_,
                                                p_ddyn_rec_,
                                                CeleX5::EventGrayPic);
  p_superimposed_img_pub_ =
      std::make_shared<CeleX5DisplayPubFactory>(nh_,
                                                "superimposed_img",
                                                p_celex5_sensor_,
                                                p_ddyn_rec_,
                                                CeleX5::EventSuperimposedPic);
  p_optical_flow_direction_img_pub_ =
      std::make_shared<CeleX5DisplayPubFactory>(nh_,
                                                "optical_flow_direction_img",
                                                p_celex5_sensor_,
                                                p_ddyn_rec_,
                                                CeleX5::OpticalFlowDirectionPic);
  p_optical_flow_speed_img_pub_ =
      std::make_shared<CeleX5DisplayPubFactory>(nh_,
                                                "optical_flow_speed_img",
                                                p_celex5_sensor_,
                                                p_ddyn_rec_,
                                                CeleX5::OpticalFlowSpeedPic);
  p_full_frame_img_pub_ =
      std::make_shared<CeleX5DisplayPubFactory>(nh_,
                                                "full_frame_img",
                                                p_celex5_sensor_,
                                                p_ddyn_rec_,
                                                CeleX5::FullFramePic);

  /*
   * Create association between mask_id and publisher
   */
  map_pub_mask.insert(std::unordered_map
                          <uint8_t, CeleX5DisplayPubFactoryPtr>::value_type
                          (0, p_binary_img_pub_));
  map_pub_mask.insert(std::unordered_map
                          <uint8_t, CeleX5DisplayPubFactoryPtr>::value_type
                          (1, p_denoised_binary_img_pub_));
  map_pub_mask.insert(std::unordered_map
                          <uint8_t, CeleX5DisplayPubFactoryPtr>::value_type
                          (2, p_count_img_pub_));
  map_pub_mask.insert(std::unordered_map
                          <uint8_t, CeleX5DisplayPubFactoryPtr>::value_type
                          (3, p_optical_flow_img_pub_));
  map_pub_mask.insert(std::unordered_map
                          <uint8_t, CeleX5DisplayPubFactoryPtr>::value_type
                          (4, p_accumulated_img_pub_));
  map_pub_mask.insert(std::unordered_map
                          <uint8_t, CeleX5DisplayPubFactoryPtr>::value_type
                          (5, p_gray_img_pub_));
  map_pub_mask.insert(std::unordered_map
                          <uint8_t, CeleX5DisplayPubFactoryPtr>::value_type
                          (6, p_superimposed_img_pub_));
  map_pub_mask.insert(std::unordered_map
                          <uint8_t, CeleX5DisplayPubFactoryPtr>::value_type
                          (7, p_optical_flow_direction_img_pub_));
  map_pub_mask.insert(std::unordered_map
                          <uint8_t, CeleX5DisplayPubFactoryPtr>::value_type
                          (8, p_optical_flow_speed_img_pub_));
  map_pub_mask.insert(std::unordered_map
                          <uint8_t, CeleX5DisplayPubFactoryPtr>::value_type
                          (9, p_in_pixel_img_pub_));
  map_pub_mask.insert(std::unordered_map
                          <uint8_t, CeleX5DisplayPubFactoryPtr>::value_type
                          (10, p_full_frame_img_pub_));

  /*
   * Create association between mask_id and publisher
   */
  map_mode_mask.insert(std::unordered_map
                           <int, uint16_t>::value_type
                           (CeleX5::Event_Off_Pixel_Timestamp_Mode,
                            static_cast<uint16_t>(1 << 0 | 1 << 1 | 1 << 2)));

  map_mode_mask.insert(std::unordered_map
                           <int, uint16_t>::value_type
                           (CeleX5::Event_In_Pixel_Timestamp_Mode,
                            static_cast<uint16_t>(1 << 0 | 1 << 3 | 1 << 9)));
  map_mode_mask.insert(std::unordered_map
                           <int, uint16_t>::value_type
                           (CeleX5::Event_Intensity_Mode,
                            static_cast<uint16_t>(1 << 0 | 1 << 2 | 1 << 4 | 1 << 5 | 1 << 6)));
  map_mode_mask.insert(std::unordered_map
                           <int, uint16_t>::value_type
                           (CeleX5::Optical_Flow_Mode,
                            static_cast<uint16_t>(1 << 0 | 1 << 3 | 1 << 7 | 1 << 8)));
  map_mode_mask.insert(std::unordered_map
                           <int, uint16_t>::value_type
                           (CeleX5::Full_Picture_Mode,
                            static_cast<uint16_t>(1 << 10)));
  // map_mode_mask.insert(std::unordered_map
  //                          <int, uint16_t>::value_type
  //                          (CeleX5::Optical_Flow_FPN_Mode,
  //                           static_cast<uint16_t>(1 << 0 | 1 << 1 | 1 << 2)));
  // map_mode_mask.insert(std::unordered_map
  //                          <int, uint16_t>::value_type
  //                          (CeleX5::Multi_Read_Optical_Flow_Mode,
  //                           static_cast<uint16_t>(1 << 0 | 1 << 1 | 1 << 2)));
  p_ddyn_rec_->publishServicesTopics();
  this->CloseAll();
}

CeleX5DisplayController::CeleX5DisplayController(const CeleX5DisplayController &) {

}

CeleX5DisplayController &CeleX5DisplayController::operator=(const CeleX5DisplayController &) {
//  return <#initializer#>;
}

CeleX5DisplayController::~CeleX5DisplayController() = default;

std::shared_ptr<CeleX5DisplayController> CeleX5DisplayController::instance = nullptr;
std::shared_ptr<std::mutex> CeleX5DisplayController::mutex_instance = std::make_shared<std::mutex>();

std::shared_ptr<CeleX5DisplayController>
CeleX5DisplayController::GetInstance(const ros::NodeHandle &nh,
                                     const std::shared_ptr<CeleX5> &p_celex5_sensor) {
  if (instance==nullptr) {
    std::unique_lock<std::mutex> uq_lock_instance(*mutex_instance);
    if (instance==nullptr) {
//      instance = std::make_shared<CeleX5DisplayController>(nh, p_celex5_sensor);
      instance = std::shared_ptr<CeleX5DisplayController>(new CeleX5DisplayController(nh, p_celex5_sensor));
    }
  }

  return instance;
}

void CeleX5DisplayController::SetCeleX5Mode(CeleX5::CeleX5Mode mode) {
  if (!p_celex5_sensor_->isLoopModeEnabled()) {
    ChangeOptions(map_mode_mask.at(static_cast<int>(mode)));
  } else {
    uint16_t enable_mask =
        map_mode_mask.at(p_celex5_sensor_->getSensorLoopMode(1));
    enable_mask |= map_mode_mask.at(p_celex5_sensor_->getSensorLoopMode(2));
    enable_mask |= map_mode_mask.at(p_celex5_sensor_->getSensorLoopMode(3));
    ChangeOptions(enable_mask);
  }
}

void CeleX5DisplayController::CloseAll() {
  for (const auto &pub : map_pub_mask) {
    auto pub_id = pub.first;
    map_pub_mask.at(pub_id)->Close();
  }
}

void CeleX5DisplayController::ChangeOptions(uint16_t mask) {
  CloseAll();
  auto judge_bit1 = [=](uint8_t bit_index) {
    return 0!=((mask >> (bit_index)) & static_cast<uint16_t>(1));
  };
  for (const auto &pub : map_pub_mask) {
    auto pub_id = pub.first;
    if (judge_bit1(pub_id)) {
      map_pub_mask.at(pub_id)->Open();
      // ROS_ERROR("%d, open", pub_id);
    }
  }
}

