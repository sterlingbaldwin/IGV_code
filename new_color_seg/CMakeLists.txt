cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(new_seg)

find_package(PCL 1.5 REQUIRED)

set(CMAKE_BUILD_TYPE Debug)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (new_seg region_growing_rgb_segmentation.cpp)
target_link_libraries (new_seg ${PCL_LIBRARIES})
