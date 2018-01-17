cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(proj)

find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (trans transform.cpp)
target_link_libraries (trans ${PCL_LIBRARIES})
