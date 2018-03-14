cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(cluster_extraction)

find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

#add_executable (super           example_supervoxels.cpp)
add_executable (lccp            example_lccp_segmentation.cpp)
#add_executable (extract_label   extract_label.cpp)

#target_link_libraries(super             ${PCL_LIBRARIES})
target_link_libraries(lccp              ${PCL_LIBRARIES})
#target_link_libraries(extract_label     ${PCL_LIBRARIES})