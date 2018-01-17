#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>

//该程序的主要流程是
//1.体素滤波，降采样
//2.分割出平面
//3.提取出分割出的符合某个平面的内点和非内点
//4.在非内点中，迭代2和3

using namespace std;
using pcl::visualization::PointCloudColorHandlerCustom;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

void viewPair(PointCloud::Ptr cloudin,PointCloud::Ptr cloudout)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer("3D Viewer"));
  int v1(0),v2(0);
  viewer->createViewPort(0, 0, 0.5, 1.0, v1);
  viewer->setBackgroundColor(0, 0, 0, v1);
  viewer->addText("平面提取前：", 10, 10, "v1_text", v1);
  //PointCloudColorHandlerCustom<PointT> red(cloudin,255,0,0,0);
  viewer->addPointCloud(cloudin, "v1", v1);
  
  viewer->createViewPort(0.5, 0, 1.0, 1.0, v2);
  viewer->setBackgroundColor(0, 0, 0, v2);
  viewer->addText("平面提取后：",10, 10, "v2_text", v2);
  viewer->addPointCloud(cloudout, "v2", v2);
  viewer->spin();
}

int main (int argc, char** argv)
{
  pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2), cloud_filtered_blob (new pcl::PCLPointCloud2);
  PointCloud::Ptr cloud_filtered (new PointCloud), cloud_p (new PointCloud), cloud_f (new PointCloud);

  // Fill in the cloud data
  pcl::PCDReader reader;
  reader.read (argv[1], *cloud_blob);

  std::cout << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points." << std::endl;

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud_blob);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter (*cloud_filtered_blob);

  // Convert to the templated PointCloud
  pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_filtered);

  std::cout << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;
  
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  
  pcl::SACSegmentation<PointT> seg;// Create the segmentation object  
  seg.setOptimizeCoefficients (true);// Optional  
  seg.setModelType (pcl::SACMODEL_PLANE);// Mandatory
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.005);

  // Create the filtering object
  pcl::ExtractIndices<PointT> extract;
  int i = 0, nr_points = (int) cloud_filtered->points.size ();  
  while (cloud_filtered->points.size () > 0.1 * nr_points)// While 30% of the original cloud is still there
  {    
    seg.setInputCloud (cloud_filtered);// Segment the largest planar component from the remaining cloud
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the inliers
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_p);
    std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;
     
    extract.setNegative (true);
    extract.filter (*cloud_f);

    viewPair(cloud_filtered,cloud_f);

/*
    std::stringstream ss;
    ss << "table_scene_lms400_plane_" << i << ".pcd";
    writer.write<pcl::PointXYZ> (ss.str (), *cloud_p, false);
*/


    cloud_filtered.swap (cloud_f);
    i++;

  }

  return (0);
}
