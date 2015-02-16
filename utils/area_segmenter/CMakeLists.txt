cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(area_segmenter)

find_package(PCL 1.2 REQUIRED)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (area_segmenter area_seg.cpp)
target_link_libraries (area_segmenter ${PCL_LIBRARIES})
