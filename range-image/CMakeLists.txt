cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(range_image)

find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (creation range_image_creation.cpp)
target_link_libraries (creation ${PCL_LIBRARIES})

add_executable (border_extraction range_image_border_extraction.cpp)
target_link_libraries (border_extraction ${PCL_LIBRARIES})
