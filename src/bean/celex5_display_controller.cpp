//
// Created by kehan on 2020/1/20.
//

#include "celex5_display_controller.h"

#include <utility>

using namespace celex5_ros;

CeleX5DisplayController::CeleX5DisplayController(const ros::NodeHandle &nh,
                                                 std::shared_ptr<CeleX5> p_celex5_sensor,
                                                 std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> p_ddyn_rec)
    : nh_(nh),
      p_celex5_sensor_(std::move(p_celex5_sensor)),
      p_ddyn_rec_(std::move(p_ddyn_rec)) {
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
                                     const std::shared_ptr<CeleX5> &p_celex5_sensor,
                                     const std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> &p_ddyn_rec) {
  if (instance==nullptr) {
    std::unique_lock<std::mutex> uq_lock_instance(*mutex_instance);
    if (instance==nullptr) {
//      instance = std::make_shared<CeleX5DisplayController>(nh, p_celex5_sensor);
      instance = std::shared_ptr<CeleX5DisplayController>(new CeleX5DisplayController(nh, p_celex5_sensor, p_ddyn_rec));
    }
  }

  return instance;
}

void CeleX5DisplayController::SetCeleX5Mode(CeleX5::CeleX5Mode mode) {
  this->CloseAll();
  if (mode==CeleX5::Event_Off_Pixel_Timestamp_Mode) {
    p_binary_img_pub_->Open();
    p_denoised_binary_img_pub_->Open();
    p_count_img_pub_->Open();
    // TODO Event Vector
  } else if (mode==CeleX5::Event_In_Pixel_Timestamp_Mode) {
    p_optical_flow_img_pub_->Open();
    p_binary_img_pub_->Open();
    p_in_pixel_img_pub_->Open();
    // TODO Event Vector
  } else if (mode==CeleX5::Event_Intensity_Mode) {
    p_binary_img_pub_->Open();
    p_gray_img_pub_->Open();
    p_count_img_pub_->Open();
    p_accumulated_img_pub_->Open();
    p_superimposed_img_pub_->Open();
    // TODO Event Vector
  } else if (mode==CeleX5::Optical_Flow_Mode) {
    p_optical_flow_img_pub_->Open();
    p_optical_flow_speed_img_pub_->Open();
    p_optical_flow_direction_img_pub_->Open();
    p_binary_img_pub_->Open();
  } else if (mode==CeleX5::Full_Picture_Mode) {
    p_full_frame_img_pub_->Open();
  }
}

void CeleX5DisplayController::CloseAll() {
  p_binary_img_pub_->Close();
  p_denoised_binary_img_pub_->Close();
  p_count_img_pub_->Close();
  p_optical_flow_img_pub_->Close();
  p_accumulated_img_pub_->Close();
  p_gray_img_pub_->Close();
  p_superimposed_img_pub_->Close();
  p_optical_flow_direction_img_pub_->Close();
  p_optical_flow_speed_img_pub_->Close();
  p_in_pixel_img_pub_->Close();
  p_full_frame_img_pub_->Close();
}

