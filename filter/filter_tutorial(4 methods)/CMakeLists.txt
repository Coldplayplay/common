cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pcl-filter)

find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#add_executable (passthrough passthrough.cpp)
add_executable (voxelgrid voxelgrid.cpp)
add_executable (sor statistical_outlier_removal.cpp)
#add_executable (ror radius_outlier_removal.cpp)
#add_executable (extract_indices extract_indices.cpp)

#target_link_libraries (passthrough ${PCL_LIBRARIES})
target_link_libraries (voxelgrid ${PCL_LIBRARIES})
target_link_libraries (sor ${PCL_LIBRARIES})
#target_link_libraries (ror ${PCL_LIBRARIES})
#target_link_libraries (extract_indices ${PCL_LIBRARIES})

