##
# The ROS package for CeleX^TM CeleX5-MIPI Dynamic Vision Sensor.
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

cmake_minimum_required(VERSION 3.2)
project(celex5_sdk)

set(CMAKE_CXX_STANDARD 14)

#set(CMAKE_BUILD_TYPE "Release")
#set(DEFINES -DCELEX_LIBRARY)
set(CMAKE_C_FLAGS "-pipe -O2 -Wall -W -D_REENTRANT -fPIC")
set(CMAKE_CXX_FLAGS "-pipe -O2 -std=gnu++0x -Wall -W -D_REENTRANT -fPIC")
set(CMAKE_SHARED_LINKER_FLAGS "-Wl,-O1 -Wl,-rpath, -shared -Wl,-soname,libCeleX.so")

find_package(OpenCV 3 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})


include_directories(base)
include_directories(configproc)
include_directories(configproc/tinyxml)
include_directories(cx3driver)
include_directories(cx3driver/include)
include_directories(eventproc)
include_directories(include)
include_directories(include/celex5)

add_library(CeleX5 SHARED
        base/dataqueue.cpp
        base/dataqueue.h
        base/filedirectory.cpp
        base/filedirectory.h
        base/xthread.cpp
        base/xthread.h
        configproc/tinyxml/tinystr.cpp
        configproc/tinyxml/tinystr.h
        configproc/tinyxml/tinyxml.cpp
        configproc/tinyxml/tinyxml.h
        configproc/tinyxml/tinyxmlerror.cpp
        configproc/tinyxml/tinyxmlparser.cpp
        configproc/celex5cfgmgr.cpp
        configproc/celex5cfgmgr.h
        configproc/wireincommand.cpp
        configproc/wireincommand.h
        configproc/xmlreader.cpp
        configproc/xmlreader.h
        cx3driver/include/libusb.h
        cx3driver/bulktransfer.cpp
        cx3driver/bulktransfer.h
        cx3driver/celedriver.cpp
        cx3driver/celedriver.h
        cx3driver/cypress.cpp
        cx3driver/cypress.h
        cx3driver/package.cpp
        cx3driver/package.h
        cx3driver/usbinterface.cpp
        cx3driver/usbinterface.h
        eventproc/celex5.cpp
        eventproc/celex5dataprocessor.cpp
        eventproc/celex5dataprocessor.h
        eventproc/celex5processeddata.cpp
        eventproc/dataprocessthread.cpp
        eventproc/dataprocessthread.h
        eventproc/datarecorder.cpp
        eventproc/datarecorder.h
        include/celex5/celex5.h
        include/celex5/celex5datamanager.h
        include/celex5/celex5processeddata.h
        include/celextypes.h)

target_link_libraries(CeleX5
        -lpthread
        -L/usr/lib/x86_64-linux-gnu/
        -lusb-1.0
        ${OpenCV_LIBS}
        )
