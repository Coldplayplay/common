#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

//There are two main differences in the color-based algorithm. 
//The first one is that it uses color instead of normals. 
//The second is that it uses the merging algorithm for over- and under- segmentation control. 
//Let’s take a look at how it is done. 
//After the segmentation, an attempt for merging clusters with close colors is made. 
//Two neighbouring clusters with a small difference between average color are merged together.
//Then the second merging step takes place. 
//During this step every single cluster is verified by the number of points that it contains.
//If this number is less than the user-defined value than current cluster is merged with the closest neighbouring cluster.
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
using namespace std;


int main (int argc, char** argv)
{
  //pcl::search::Search <pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> > (new pcl::search::KdTree<pcl::PointXYZRGB>);
  
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
  
  

  PointCloud::Ptr cloud (new PointCloud);
  if ( pcl::io::loadPCDFile <PointT> (argv[1], *cloud) == -1 )//official pcd is region_growing_rgb_tutorial
  {
    std::cout << "Cloud reading failed." << std::endl;
    return (-1);
  }

  pcl::IndicesPtr indices (new std::vector <int>);
  pcl::PassThrough<PointT> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.2);
  pass.filter (*indices);

  pcl::RegionGrowingRGB<PointT> reg;

  reg.setInputCloud (cloud);
  reg.setIndices (indices);

  reg.setSearchMethod (tree);
  reg.setDistanceThreshold (20);//参数是点的个数还是距离？  determine whether the point is neighbouring or not.

  reg.setPointColorThreshold (10);//判断是不是一类，点云颜色之差的阈值
  reg.setRegionColorThreshold (25);//判断两类能不能合并的点云颜色之差的阈值
  reg.setMinClusterSize (300);

  std::vector <pcl::PointIndices> clusters;
  reg.extract (clusters);
  cout<<"1"<<endl;
  pcl::PointCloud <PointT>::Ptr colored_cloud = reg.getColoredCloud ();
  pcl::visualization::CloudViewer viewer ("Cluster viewer");
  viewer.showCloud (colored_cloud);

  pcl::PCDWriter writer;
  cout<<"2"<<endl;
  if(colored_cloud->points.size()!=0)
  {
    writer.write("rgb_cluster_result.pcd",*colored_cloud);
   std::cout<<"has finished rgb clustering.please see the result.pcd."<<endl;
  }
  else
   cout<<"Failure in rgb clustering."<<endl;

  while (!viewer.wasStopped ())
  {
    boost::this_thread::sleep (boost::posix_time::microseconds (100));
  }

  return (0);
}

