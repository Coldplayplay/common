cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(filter_exp)
find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (arrester exp_arrester.cpp)
target_link_libraries(arrester ${PCL_LIBRARIES})




