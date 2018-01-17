cmake_minimum_required (VERSION 2.6 FATAL_ERROR)

project      (pcl-visualizer)

find_package (PCL 1.7.1 REQUIRED)

include_directories (${PCL_INCLUDE_DIRS})
link_directories    (${PCL_LIBRARY_DIRS})
add_definitions     (${PCL_DEFINITIONS})


set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
ADD_EXECUTABLE  (main add_cube.cpp)

TARGET_LINK_LIBRARIES (main ${PCL_LIBRARIES} )

