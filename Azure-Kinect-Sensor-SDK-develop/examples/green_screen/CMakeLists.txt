# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License.

cmake_minimum_required( VERSION 3.6 )

project (green_screen)

set( CMAKE_CXX_STANDARD 11 )
set( CMAKE_CXX_STANDARD_REQUIRED ON )
set( CMAKE_CXX_EXTENSIONS OFF )

#set_property( DIRECTORY PROPERTY VS_STARTUP_PROJECT "K4ABT" )

set( CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}" ${CMAKE_MODULE_PATH} )
find_package( k4a REQUIRED )

add_executable(green_screen main.cpp)
target_include_directories( green_screen PRIVATE ${OpenCV_INCLUDE_DIRS} )
target_link_libraries(green_screen PRIVATE k4a::k4a ${OpenCV_LIBS})

include_directories("C:\\Users\\Abby.Eustace\\Desktop\\Kinect Azure\\Azure Kinect SDK v1.3.0\\sdk\\include"
"C:\\opencv\\build\\include")