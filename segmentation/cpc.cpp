






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
  //生成CPC分割器
pcl::CPCSegmentation<PointT>::CPCSegmentation seg;
//输入超体聚类结果
seg.setInputSupervoxels(supervoxel_clusters,supervoxel_adjacency);
//设置分割参数
setCutting (max_cuts = 20,
		 cutting_min_segments = 0,
		 cutting_min_score = 0.16,
		 locally_constrained = true,
		 directed_cutting = true,
		 clean_cutting = false)；
seg.setRANSACIterations (ransac_iterations);
seg.segment();
seg.relabelCloud (pcl::PointCloud<pcl::PointXYZL> &labeled_cloud_arg);
