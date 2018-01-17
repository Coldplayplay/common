cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(cluster_extraction)

find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

add_executable (cluster_extraction 		Euclidean_cluster_extraction.cpp)
#add_executable (planar_segmentation 		planar_segmentation.cpp)
#add_executable (cylinder_segmentation 		cylinder_segmentation.cpp)
#add_executable(region_growing_segmentation 	region_growing_segmentation.cpp)
add_executable(region_growing_rgb_segmentation	region_growing_rgb_segmentation.cpp)
add_executable(min_cut_segmentation 		min_cut_segmentation.cpp)
#add_executable(conditional_euclidean_seg 	conditional_euclidean_clustering.cpp)
#add_executable(don_segmentation 		don_segmentation.cpp)
add_executable(supervoxel_clustering 		supervoxel_clustering.cpp)
#add_executable(pmf_segmentation 		pmf_segmentation.cpp)

target_link_libraries (cluster_extraction ${PCL_LIBRARIES})
#target_link_libraries (planar_segmentation ${PCL_LIBRARIES})
#target_link_libraries (cylinder_segmentation ${PCL_LIBRARIES})
#target_link_libraries (region_growing_segmentation ${PCL_LIBRARIES})
target_link_libraries (region_growing_rgb_segmentation ${PCL_LIBRARIES})
target_link_libraries (min_cut_segmentation ${PCL_LIBRARIES})
#target_link_libraries(conditional_euclidean_seg ${PCL_LIBRARIES})
#target_link_libraries(don_segmentation ${PCL_LIBRARIES})
target_link_libraries(supervoxel_clustering ${PCL_LIBRARIES})
#target_link_libraries(pmf_segmentation ${PCL_LIBRARIES})
