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

#include "temporal_offset_calculate.h"

// TemporalOffsetCalculate::TemporalOffsetCalculate(const ros::NodeHandle &nh, bool show_match)
//     : nh_(nh),
//       show_match_(show_match),
//       corners_col_num_(8),    // default value
//       corners_row_num_(6) {
//   nh_.param("corners_col_num", corners_col_num_, corners_col_num_);
//   nh_.param("corners_row_num", corners_row_num_, corners_row_num_);
//
//   p_ddyn_rec_ = std::make_shared<ddynamic_reconfigure::DDynamicReconfigure>(nh_);
//   p_ddyn_rec_->registerVariable<bool>("show_match", &show_match_, "Show corners matching");
//   p_ddyn_rec_->publishServicesTopics();
// }
//
// TemporalOffsetCalculate::~TemporalOffsetCalculate() {
//
// }


TemporalOffsetCalculate::TemporalOffsetCalculate(const ros::NodeHandle &nh)
        : nh_(nh),
          is_plot_(true),
          x_length_(20)
{

    init_stamp_ = ros::Time::now();
    last_events_stamp_ = init_stamp_;
    usleep(3000);

    std::string events_topic("/events");
    nh_.param("events_topic", events_topic, events_topic);
    events_sub_ = nh_.subscribe<celex5_msgs::EventVector>(events_topic, 1,
                                                          &TemporalOffsetCalculate::EventsCallback,
                                                          this);
    std::string frame_topic("/frame");
    nh_.param("frame_topic", frame_topic, frame_topic);
    frame_sub_ = nh_.subscribe<sensor_msgs::Image>(frame_topic, 1,
                                                   &TemporalOffsetCalculate::FrameCallback,
                                                   this);

    p_ddyn_rec_ = std::make_shared<ddynamic_reconfigure::DDynamicReconfigure>(nh_);
    p_ddyn_rec_->registerVariable<bool>("is_plot", &is_plot_, "Whether plot.", false, true);
    p_ddyn_rec_->registerVariable<double>("x_length", &x_length_, "X axis range in plot.", -2000, 2000);
    p_ddyn_rec_->publishServicesTopics();
}


TemporalOffsetCalculate::~TemporalOffsetCalculate()
{

}


void TemporalOffsetCalculate::EventsCallback(const celex5_msgs::EventVectorConstPtr &msg)
{
    // ROS_WARN("Get callback!");
    vec_events_rate_stamps_.emplace_back((msg->header.stamp - init_stamp_).toSec());
    ros::Duration duration = msg->header.stamp - last_events_stamp_;
    double events_rate = std::log((msg->events.size() / duration.toSec()));
    vec_events_rate_history_.emplace_back(events_rate);
    AnimationPlot();
    last_events_stamp_ = msg->header.stamp;
}


void TemporalOffsetCalculate::FrameCallback(const sensor_msgs::ImageConstPtr &msg)
{
    cv::Mat current_frame = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image; // TODO encoding
    cv::cvtColor(current_frame, current_frame, cv::COLOR_RGB2GRAY);
    if (last_frame_.empty())
    {
        // Didn't initial
        last_frame_ = current_frame;
        return;
    }
    else
    {
        int64_t intensity_changes = std::log(CalculateIntensityChanges(last_frame_, current_frame));
        vec_intensity_changes_stamps_.emplace_back((msg->header.stamp - init_stamp_).toSec());
        vec_intensity_changes_history_.emplace_back(intensity_changes);
        AnimationPlot();
        last_frame_ = current_frame;
    }
}


int64_t TemporalOffsetCalculate::CalculateIntensityChanges(const cv::Mat &frame1, const cv::Mat &frame2)
{
    int64_t intensity = 0;
    if (frame1.rows != frame2.rows || frame1.cols != frame2.cols)
    {
        return -1;
    }
    for (int i = 0; i < frame2.rows; ++i)
    {
        for (int j = 0; j < frame2.cols; ++j)
        {
            intensity += static_cast<int64_t>(std::abs(frame2.at<uchar>(i, j) - frame1.at<uchar>(i, j)));
        }
    }
    return intensity;
}


void TemporalOffsetCalculate::AnimationPlot()
{
    // ROS_WARN("Get plot!");
    plt::clf();;
    plt::named_plot("Events Rate", vec_events_rate_stamps_, vec_events_rate_history_);
    plt::named_plot("Intensity Changes", vec_intensity_changes_stamps_, vec_intensity_changes_history_);
    double now_stamped = vec_events_rate_stamps_.at(vec_events_rate_stamps_.size() - 1);
    plt::xlim((now_stamped - 20 > 0 ? (now_stamped - x_length_) : 0), now_stamped);
    plt::title("Temporal Offset");
    plt::legend();
    plt::pause(0.001);
}


