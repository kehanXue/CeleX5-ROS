/**
 *  The celex5_calibration ROS package,
 *  used to calibrate camera parameters for CeleX5-MIPI Event-based Camera.
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

#include "event_registration.h"

EventRegistration::EventRegistration(const ros::NodeHandle &nh)
    : nh_(nh),
      rgb_info_initialed(false),
      events_info_initialed(false) {
  /*
   * Create sync images handle
   */
  // std::string depth_image_topic_name("/depth/image");
  std::string depth_image_topic_name("/zed/zed_node/depth/depth_registered");
  nh_.param("depth_image", depth_image_topic_name, depth_image_topic_name);
  p_depth_image_sub_ = std::make_shared<MfImageSub>(nh_, depth_image_topic_name, 2,
                                                    ros::TransportHints().tcpNoDelay());
  std::string rgb_image_topic_name("/zed/zed_node/left/image_rect_color");
  // std::string rgb_image_topic_name("/rgb/image");
  nh_.param("rgb_image", rgb_image_topic_name, rgb_image_topic_name);
  p_rgb_image_sub_ = std::make_shared<MfImageSub>(nh_, rgb_image_topic_name, 2,
                                                  ros::TransportHints().tcpNoDelay());

  p_sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(10), *p_depth_image_sub_, *p_rgb_image_sub_
  );
  p_sync_->registerCallback(boost::bind(&EventRegistration::SyncImagesCallback, this, _1, _2));

  std::string events_topic_name("/celex5_mipi/events");
  // std::string events_topic_name("/events");
  nh_.param("events_topic", events_topic_name, events_topic_name);
  events_sub_ = nh_.subscribe<celex5_msgs::EventVector>(events_topic_name,
                                                        1,
                                                        &EventRegistration::EventsCallback,
                                                        this);

  std::string events_camera_info_topic_name("/celex5_mipi/display/binary_img/camera_info");
  // std::string events_camera_info_topic_name("/events/camera_info");
  nh_.param("events_camera_info", events_camera_info_topic_name, events_camera_info_topic_name);
  events_info_sub_ = nh_.subscribe<sensor_msgs::CameraInfo>(events_camera_info_topic_name,
                                                            1,
                                                            &EventRegistration::EventsCameraInfoCallback,
                                                            this);

  std::string rgb_camera_info_topic_name("/zed/zed_node/left/camera_info");
  // std::string rgb_camera_info_topic_name("/rgb/camera_info");
  nh_.param("rgb_camera_info", rgb_camera_info_topic_name, rgb_camera_info_topic_name);
  rgb_info_sub_ = nh_.subscribe<sensor_msgs::CameraInfo>(rgb_camera_info_topic_name,
                                                         1,
                                                         &EventRegistration::RgbCameraInfoCallback,
                                                         this);

  T_ = Eigen::Isometry3d::Identity();
  // Eigen::Matrix3d R;
  // R << 10, 1, 1,
  //     1, 1, 1,
  //     1, 1, 1;
  // T_.rotate(R);
  T_.pretranslate(Eigen::Vector3d(-0.06014965, 0.04751369, 0.01298155));

  p_thread_process_ = std::make_shared<std::thread>([&]() {
    while (ros::ok()) {
      std::unique_lock<std::mutex> lck(mu_new_frame_);
      cv_new_frame_.wait(lck);

      if (rgb_info_initialed && events_info_initialed) {

        rgb_on_events_frame_ = cv::Mat(800, 1280, CV_8UC3, cv::Scalar::all(0));

        Eigen::Matrix3d rgb_K_inverse = rgb_K_.inverse();
        std::cout << rgb_K_inverse << std::endl;
        // for (int v = 0; v < depth_frame_.rows; ++v) {
        //   for (int u = 0; u < depth_frame_.cols; ++u) {
        int cnt = 0;
        for (int v = 0; v < depth_frame_.rows; v += 5) {
          for (int u = 0; u < depth_frame_.cols; u += 5) {

            ros::Time start_time = ros::Time::now();

            auto depth = depth_frame_.at<float>(v, u);
            if (depth == 0) {
              return;
            }

            // auto depth = static_cast<float>(depth_frame_.at<float>(v, u));
            Eigen::Vector3d points_in_rgb_world =
                // static_cast<double>(depth) * rgb_K_.colPivHouseholderQr().solve(Eigen::Vector3d(u, v, 1));
                static_cast<double>(depth) * rgb_K_inverse * Eigen::Vector3d(u, v, 1);
            Eigen::Vector3d points_in_events_world = T_ * points_in_rgb_world;
            Eigen::Vector3d uv_in_events = static_cast<double>(1 / depth) * events_K_ * points_in_events_world;
            if (uv_in_events[2] == 0) {
              return;
            }
            uv_in_events = uv_in_events / uv_in_events[2];
            std::cout << "[u, v, 1] in Event Camera:" << std::endl << uv_in_events << std::endl << std::endl;
            std::cout << "Pixel count: " << cnt++ << std::endl;
            int u_in_events = std::ceil(uv_in_events[0]);
            int v_in_events = std::ceil(uv_in_events[1]);
            if ((v_in_events < rgb_on_events_frame_.rows && v_in_events >= 0) &&
                (u_in_events < rgb_on_events_frame_.cols && u_in_events >= 0)) {
              rgb_on_events_frame_.at<cv::Vec3b>(v_in_events, u_in_events) = rgb_frame_.at<cv::Vec3b>(v, u);
              // rgb_on_events_frame_.at<cv::Vec3b>(u_in_events, v_in_events)[1] = rgb_frame_.at<cv::Vec3b>(u, v)[1];
              // rgb_on_events_frame_.at<cv::Vec3b>(u_in_events, v_in_events)[2] = rgb_frame_.at<cv::Vec3b>(u, v)[2];
            }

            std::cout << "Time cost: " << (ros::Time::now() - start_time).toSec() << " s" << std::endl;
            std::cout << "--------------------------" << std::endl;
          }
        }
        cv::imshow("Convert: ", rgb_on_events_frame_);
        cv::waitKey(1);
      }

      lck.unlock();
    }
  });
}

EventRegistration::~EventRegistration() {
  if (p_thread_process_->joinable()) {
    p_thread_process_->join();
  }
}

void EventRegistration::SyncImagesCallback(const sensor_msgs::ImageConstPtr &depth_msg,
                                           const sensor_msgs::ImageConstPtr &rgb_msg) {
  ROS_WARN("!!!Get here");
  if (rgb_info_initialed && events_info_initialed) {
    /*
     * Get depth values
     * TODO value
     */
    // auto *depths = (float *) (&depth_msg->data[0]);
    // std::vector<float> vec_depths(depths, depths + depth_msg->data.size());
    if (depth_msg->height != rgb_msg->height ||
        depth_msg->width != rgb_msg->width) {
      return;
    }

    if (mu_new_frame_.try_lock()) {
      depth_frame_ = cv_bridge::toCvShare(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1)->image.clone();
      rgb_frame_ = cv_bridge::toCvShare(rgb_msg, "bgr8")->image.clone();
      mu_new_frame_.unlock();
      cv_new_frame_.notify_all();
    }

  }
}

void EventRegistration::EventsCallback(const celex5_msgs::EventVectorConstPtr &event_msg) {

}

void EventRegistration::RgbCameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg) {
  if (!rgb_info_initialed) {
    rgb_K_ << msg->K.at(0), msg->K.at(1), msg->K.at(2),
        msg->K.at(3), msg->K.at(4), msg->K.at(5),
        msg->K.at(6), msg->K.at(7), msg->K.at(8);
    rgb_info_initialed = true;
  }
}

void EventRegistration::EventsCameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg) {
  if (!events_info_initialed) {
    events_K_ << msg->K.at(0), msg->K.at(1), msg->K.at(2),
        msg->K.at(3), msg->K.at(4), msg->K.at(5),
        msg->K.at(6), msg->K.at(7), msg->K.at(8);

    events_info_initialed = true;
  }
}

