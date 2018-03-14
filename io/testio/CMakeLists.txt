cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(proj)

find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
add_executable (iotest iotest.cpp)
target_link_libraries (iotest ${PCL_LIBRARIES})

add_executable (obj_io obj_io.cpp)
target_link_libraries (obj_io ${PCL_LIBRARIES})
