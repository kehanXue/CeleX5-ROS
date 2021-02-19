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

#include "pattern_generator.h"


PatternGenerator::PatternGenerator(const ros::NodeHandle &nh,
                                   int square_size,
                                   int col_corners_num,
                                   int row_corners_num,
                                   bool static_board,
                                   int blinking_fps,
                                   bool record_video,
                                   bool show_image)
        : nh_(nh),
          square_size_(square_size),
          col_corners_num_(col_corners_num),
          row_corners_num_(row_corners_num),
          static_board_(static_board),
          blinking_fps_(blinking_fps),
          record_video_(record_video),
          show_img_(show_image)
{

    p_ddyn_rec_ = std::make_shared<ddynamic_reconfigure::DDynamicReconfigure>(nh_);
    p_ddyn_rec_->registerVariable<int>("square_size", &square_size_,
                                       "Square size in chessboard. Uint: pixel", 0, 400);
    p_ddyn_rec_->registerVariable<int>("col_corners_num", &col_corners_num_,
                                       "Corners' number in col of chessboard.", 0, 20);
    p_ddyn_rec_->registerVariable<int>("row_corners_num", &row_corners_num_,
                                       "Corners' number in row of chessboard.", 0, 20);
    p_ddyn_rec_->registerVariable<bool>("static_board", &static_board_,
                                        "Static chessboard.", false, true);
    p_ddyn_rec_->registerVariable<bool>("record_video", &record_video_,
                                        "Clicked start record video, and cancel it to stop record video.", false, true);
    p_ddyn_rec_->registerVariable<bool>("show_image", &show_img_,
                                        "Enable to show blinking images.", false, true);

    p_thread_ =
            std::make_shared<std::thread>(
                    [&]()
                    {
                        ros::Rate loop_rate(blinking_fps_ * 2);
                        p_ddyn_rec_->registerVariable<int>("blinking_fps", blinking_fps_,
                                                           [&loop_rate](int new_fps)
                                                           {
                                                               loop_rate = ros::Rate(
                                                                       new_fps * 2);
                                                           }, "Blinking fps of chessboard.",
                                                           0, 72);

                        cv::VideoWriter video_writer;

                        int cnt = 0;
                        while (ros::ok())
                        {
                            cnt = (++cnt) % 2;
                            if (cnt == 0)
                            {
                                // CreateChessBoard();
                                CreateChessBoard();
                            }
                            else
                            {
                                if (!static_board_)
                                {
                                    CreateBlackBoard();
                                }
                            }
                            if (record_video_)
                            {
                                if (video_writer.isOpened())
                                {
                                    video_writer << pattern_board_;
                                }
                                else
                                {
                                    video_writer.open("blinking_chessboard.avi",
                                                      CV_FOURCC('M', 'J', 'P', 'G'),
                                                      blinking_fps_ * 2,
                                                      cv::Size(pattern_board_.cols,
                                                               pattern_board_.rows));
                                }
                            }
                            if (show_img_)
                            {
                                cv::imshow("Pattern Board", pattern_board_);
                            }
                            cv::waitKey(1);
                            loop_rate.sleep();
                        }
                    });
    usleep(3000);

    p_ddyn_rec_->publishServicesTopics();
}


PatternGenerator::~PatternGenerator()
{
    if (p_thread_->joinable())
    {
        p_thread_->join();
    }
}


void PatternGenerator::CreateBlackBoard()
{
    pattern_board_ = cv::Mat(square_size_ * (row_corners_num_ + 3),
                             square_size_ * (col_corners_num_ + 3),
                             CV_8UC3, cv::Scalar::all(0));
}


void PatternGenerator::CreateChessBoard()
{
    pattern_board_ = cv::Mat(square_size_ * (row_corners_num_ + 3),
                             square_size_ * (col_corners_num_ + 3),
                             CV_8UC3, cv::Scalar::all(0));
    for (int j = 0; j < pattern_board_.rows; ++j)
    {
        auto* data = pattern_board_.ptr<cv::Vec3b>(j);
        for (int i = 0; i < pattern_board_.cols; ++i)
        {
            if ((i / square_size_) == 0 || (i / square_size_ == (col_corners_num_ + 2)))
            {
                data[i][0] = 255;
                data[i][1] = 255;
                data[i][2] = 255;
            }
            else if ((j / square_size_) == 0 || (j / square_size_ == (row_corners_num_ + 2)))
            {
                data[i][0] = 255;
                data[i][1] = 255;
                data[i][2] = 255;
            }
            else if (((i / square_size_ + j / square_size_) % 2) == 1)
            {
                data[i][0] = 255;
                data[i][1] = 255;
                data[i][2] = 255;
            }
        }
    }
}
