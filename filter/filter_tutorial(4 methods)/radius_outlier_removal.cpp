#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>

int
 main (int argc, char** argv)
{
  if (argc != 2)
  {
    std::cerr << "please specify command line arg '-r' or '-c'" << std::endl;
    exit(0);
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZ>);
  // Fill in the cloud data
  cloud->width  = 5;
  cloud->height = 1;
  cloud->points.resize (cloud->width * cloud->height);
  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
    cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  }
 
/*
   // Fill in the cloud data
  pcl::PCDReader reader;
  reader.read<pcl::PointXYZ> ("test.pcd", *cloud1);

  std::cerr << "Cloud before filtering: " << std::endl;
  std::cerr << *cloud1 << std::endl;

  pcl::PCDWriter writer;
  pcl::PCDWriter writer1;
  */
  if (strcmp(argv[1], "-r") == 0){
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    outrem.setInputCloud(cloud);
    outrem.setRadiusSearch(0.8);
    outrem.setMinNeighborsInRadius (1);
    outrem.filter (*cloud_filtered);
   // writer.write<pcl::PointXYZ> ("table_scene_lms400_r.pcd", *cloud_filtered, false);
  }
  else if (strcmp(argv[1], "-c") == 0){
    pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new
      pcl::ConditionAnd<pcl::PointXYZ> ());
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
      pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, 0.0)));
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
      pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, 1.4)));
 
    pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
    condrem.setCondition (range_cond);
    condrem.setInputCloud (cloud);
    condrem.setKeepOrganized(true);
    condrem.filter (*cloud_filtered);
    std::cerr << "Cloud after filtering: " << std::endl;
    std::cerr << *cloud_filtered << std::endl;
    //writer1.write<pcl::PointXYZ> ("table_scene_lms400_c.pcd", *cloud_filtered, false);
  }
  else{
    std::cerr << "please specify command line arg '-r' or '-c'" << std::endl;
    exit(0);
  }
  
  //display pointcloud before filtering
  std::cerr << "Cloud before filtering: " << std::endl;
  std::cerr << *cloud1 << std::endl;
  for (size_t i = 0; i < cloud1->points.size (); ++i)
    std::cerr << "    " << cloud->points[i].x << " "
                        << cloud->points[i].y << " "
                        << cloud->points[i].z << std::endl;
  // display pointcloud after filtering
  std::cerr << "Cloud after filtering: " << std::endl;
  std::cerr << *cloud_filtered << std::endl;
  for (size_t i = 0; i < cloud_filtered->points.size (); ++i)
    std::cerr << "    " << cloud_filtered->points[i].x << " "
                        << cloud_filtered->points[i].y << " "
                        << cloud_filtered->points[i].z << std::endl;

  
  
  return (0);
}