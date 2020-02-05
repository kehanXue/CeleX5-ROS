##
# The celex5_calibration ROS package,
# used to calibrate camera parameters for CeleX5-MIPI Event-based Camera.
#
# Copyright (C) 2020  Kehan.Xue<kehan.xue@gmail.com>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

cmake_minimum_required(VERSION 2.8.3)
project(pattern_generator)

add_compile_options(-std=c++14)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        ddynamic_reconfigure)

find_package(OpenCV 3 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})

add_executable(pattern_generator_node
        pattern_generator_node.cpp
        pattern_generator.h
        pattern_generator.cpp)

target_link_libraries(pattern_generator_node
        ${OpenCV_LIBS}
        ${catkin_LIBRARIES})
