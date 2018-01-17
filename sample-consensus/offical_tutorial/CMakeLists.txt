cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(RANSAC)
find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(rsc random_sample_consensus.cpp)
target_link_libraries(rsc ${PCL_LIBRARIES})

