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


#include "sync_images.h"


SyncImages::SyncImages(const ros::NodeHandle &nh)
        : nh_(nh),
          store_dir_("./"),
          save_(false)
{

    /*
     * Create a new clean directory to store images
     */
    nh_.param("store_dir", store_dir_, store_dir_);
    // TODO Fix bugs, doesn't work.
    // if (!boost::filesystem::is_empty(store_dir_)) {
    //   boost::filesystem::remove_all(store_dir_);
    // }
    // if (!boost::filesystem::exists(store_dir_)) {
    //   boost::filesystem::create_directory(store_dir_);
    // boost::filesystem::create_directory(store_dir_ + "frame/");
    // boost::filesystem::create_directory(store_dir_ + "events/");
    // }

    /*
     * Create sync images handle
     */
    std::string frame_topic_name("/frame_image");
    nh_.param("frame_topic", frame_topic_name, frame_topic_name);
    p_frame_sync_sub_ = std::make_shared<MfImageSub>(nh_, frame_topic_name, 2,
                                                     ros::TransportHints().tcpNoDelay());
    std::string events_topic_name("/events_image");
    nh_.param("events_topic", events_topic_name, events_topic_name);
    p_events_sync_sub_ = std::make_shared<MfImageSub>(nh_, events_topic_name, 2,
                                                      ros::TransportHints().tcpNoDelay());

    p_sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), *p_frame_sync_sub_, *p_events_sync_sub_
    );
    p_sync_->registerCallback(boost::bind(&SyncImages::SyncImagesCallback, this, _1, _2));

    p_ddyn_rec_ = std::make_shared<ddynamic_reconfigure::DDynamicReconfigure>(nh_);
    p_ddyn_rec_->registerVariable<bool>("save", &save_, "Save images", false, true);
    p_ddyn_rec_->publishServicesTopics();
    // frame_sub_ = nh_.subscribe<sensor_msgs::Image>(frame_topic_name, 10,
    //                                                &SyncImages::FrameImageCallback,
    //                                                this);
    // events_sub_ = nh_.subscribe<sensor_msgs::Image>(events_topic_name, 10,
    //                                                 &SyncImages::EventsImageCallback,
    //                                                 this);
}


SyncImages::~SyncImages() = default;


void SyncImages::SyncImagesCallback(const sensor_msgs::ImageConstPtr &frame_msg,
                                    const sensor_msgs::ImageConstPtr &events_msg)
{
    if (save_)
    {
        cv_bridge::CvImagePtr frame_image_ptr;
        cv_bridge::CvImagePtr events_image_ptr;
        try
        {
            // TODO Gray or RGB
            frame_image_ptr = cv_bridge::toCvCopy(frame_msg, sensor_msgs::image_encodings::BGR8);
            events_image_ptr = cv_bridge::toCvCopy(events_msg, sensor_msgs::image_encodings::MONO8);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR_STREAM("cv_bridge Exception:" << e.what());
            return;
        }

        cv::Mat frame_img = frame_image_ptr->image;
        cv::Mat events_img = events_image_ptr->image;

        std::string pair_id = std::to_string(frame_msg->header.seq);
        // boost::filesystem::create_directory(store_dir_ + pair_id);

        // ROS_INFO("store_path, %s", store_dir.c_str());
        cv::imwrite(store_dir_ + "frame/" + pair_id + ".jpg", frame_img);
        cv::imwrite(store_dir_ + "events/" + pair_id + ".jpg", events_img);
        cv::waitKey(1);
        ROS_INFO("Write! %s", pair_id.c_str());
    }
}

// void SyncImages::FrameImageCallback(const sensor_msgs::ImageConstPtr &frame_msg) {
//   ROS_ERROR("Frame : %lf", frame_msg->header.stamp.toSec());
// }
//
// void SyncImages::EventsImageCallback(const sensor_msgs::ImageConstPtr &events_msg) {
//   ROS_WARN("Events: %lf", events_msg->header.stamp.toSec());
// }
