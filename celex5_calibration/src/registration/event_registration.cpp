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
  Eigen::Matrix3d R;
  R << 0.9998723854537899, -0.0077667289193917135, 0.013960326960080705,
      0.007148637873527468, 0.9990150840929704, 0.043792222269031116,
      -0.01428669953113176, -0.04368683642242523, 0.9989431167688675;
  T_.rotate(Eigen::AngleAxisd(R));
  T_.pretranslate(Eigen::Vector3d(-0.06014965, 0.04751369, 0.01298155));
  // T_.pretranslate(Eigen::Vector3d(0.06014965, -0.04751369, -0.01298155));

  p_thread_process_ = std::make_shared<std::thread>([&]() {
    while (ros::ok()) {
      std::unique_lock<std::mutex> lck(mu_new_frame_);
      cv_new_frame_.wait(lck);

      if (rgb_info_initialed && events_info_initialed) {

        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> pixel_data(3, depth_frame_.rows * depth_frame_.cols);
        int index = 0;
        for (int v = 0; v < depth_frame_.rows; ++v) {
          for (int u = 0; u < depth_frame_.cols; ++u) {
            pixel_data(0, index) = static_cast<double>(u) * depth_frame_.at<float>(v, u);
            pixel_data(1, index) = static_cast<double>(v) * depth_frame_.at<float>(v, u);
            pixel_data(2, index) = static_cast<double>(depth_frame_.at<float>(v, u));
            index++;
          }
        }

        pixel_data = events_K_ * T_ * (rgb_K_.colPivHouseholderQr().solve(pixel_data));

        rgb_on_events_frame_ = cv::Mat(800, 1280, CV_8UC3, cv::Scalar::all(0));
        index = 0;
        for (int v = 0; v < depth_frame_.rows; ++v) {
          for (int u = 0; u < depth_frame_.cols; ++u) {
            if (pixel_data(2, index) == 0) {
              index++;
              continue;
            }

            /*
             * Normalization
             */
            int v_in_events = std::ceil(pixel_data(1, index) / pixel_data(2, index));
            int u_in_events = std::ceil(pixel_data(0, index) / pixel_data(2, index));

            if ((v_in_events < rgb_on_events_frame_.rows && v_in_events >= 0) &&
                (u_in_events < rgb_on_events_frame_.cols && u_in_events >= 0)) {
              rgb_on_events_frame_.at<cv::Vec3b>(v_in_events, u_in_events) = rgb_frame_.at<cv::Vec3b>(v, u);
            }
            index++;
          }
        }

        // cv::imshow("Convert: ", rgb_on_events_frame_);
        // cv::waitKey(1);
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
  if (!rgb_on_events_frame_.empty() && rgb_info_initialed && events_info_initialed) {
    cv::Mat frame = rgb_on_events_frame_.clone();
    for (const auto &event : event_msg->events) {
      auto *p = frame.ptr<cv::Vec3b>(frame.rows - 1 - event.x);
      p[event.y][0] = 0;
      p[event.y][1] = 0;
      p[event.y][2] = 255;
    }
    cv::imshow("Events on frame", frame);
    cv::waitKey(1);
  }
}

void EventRegistration::RgbCameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg) {
  if (!rgb_info_initialed) {
    rgb_K_ << msg->K.at(0), msg->K.at(1), msg->K.at(2),
        msg->K.at(3), msg->K.at(4), msg->K.at(5),
        msg->K.at(6), msg->K.at(7), msg->K.at(8);

    // rgb_K_ << 1437.8958223832353, 0., 1181.7885268260493,
    //     0., 1437.3087636208852, 675.0761478882741,
    //     0., 0., 1.;
    rgb_info_initialed = true;
  }
}

void EventRegistration::EventsCameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg) {
  if (!events_info_initialed) {
    // events_K_ << msg->K.at(0), msg->K.at(1), msg->K.at(2),
    //     msg->K.at(3), msg->K.at(4), msg->K.at(5),
    //     msg->K.at(6), msg->K.at(7), msg->K.at(8);

    events_K_ << 1707.9980790098505, 0., 631.895926240894,
        0., 1705.2747370661336, 413.56095397845604,
        0., 0., 1.;
    events_info_initialed = true;
  }
}

