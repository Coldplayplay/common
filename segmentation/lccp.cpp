#include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/segmentation/lccp_segmentation.h>

//VTK include needed for drawing graph lines
#include <vtkPolyLine.h>

// Types
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointNCloudT;
typedef pcl::PointXYZL PointLT;
typedef pcl::PointCloud<PointLT> PointLCloudT;


//设定结晶参数
  float voxel_resolution = 0.008f;
  float seed_resolution = 0.1f;
  float color_importance = 0.2f;
  float spatial_importance = 0.4f;
  float normal_importance = 1.0f;
  
  //生成结晶器
  pcl::SupervoxelClustering<PointT> super (voxel_resolution, seed_resolution);
  //和点云形式有关
  if (disable_transform)
    super.setUseSingleCameraTransform (false);
  //输入点云及结晶参数
  super.setInputCloud (cloud);
  super.setColorImportance (color_importance);
  super.setSpatialImportance (spatial_importance);
  super.setNormalImportance (normal_importance);
  //输出结晶分割结果：结果是一个映射表
  std::map <uint32_t, pcl::Supervoxel<PointT>::Ptr > supervoxel_clusters;
  super.extract (supervoxel_clusters);
  std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
  super.getSupervoxelAdjacency (supervoxel_adjacency);
  
  //生成LCCP分割器
pcl::LCCPSegmentation<PointT>::LCCPSegmentation LCCPseg;
//输入超体聚类结果
seg.setInputSupervoxels(supervoxel_clusters,supervoxel_adjacency);
//CC效验beta值
seg.setConcavityToleranceThreshold (concavity_tolerance_threshold);
//CC效验的k邻点
seg.setKFactor (k_factor_arg)
//
seg.setSmoothnessCheck (bool_use_smoothness_check_arg,voxel_res_arg,seed_res_arg,smoothness_threshold_arg = 0.1);
//SC效验
seg.setSanityCheck (bool_use_sanity_criterion_arg);
//最小分割尺寸
seg.setMinSegmentSize (min_segment_size_arg)

seg.segment();
seg.relabelCloud (pcl::PointCloud<pcl::PointXYZL> &labeled_cloud_arg);


boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);

  PointCloudT::Ptr voxel_centroid_cloud = super.getVoxelCentroidCloud ();
  viewer->addPointCloud (voxel_centroid_cloud, "voxel centroids");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2.0, "voxel centroids");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY,0.95, "voxel centroids");

  PointLCloudT::Ptr labeled_voxel_cloud = super.getLabeledVoxelCloud ();
  viewer->addPointCloud (labeled_voxel_cloud, "labeled voxels");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY,0.8, "labeled voxels");




