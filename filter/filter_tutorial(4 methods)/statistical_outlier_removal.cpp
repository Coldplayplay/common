#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include<pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
using namespace std;
using pcl::visualization::PointCloudColorHandlerRGBField;
void viewPair(PointCloud::Ptr cloud1, PointCloud::Ptr cloud1al)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));    
  viewer->initCameraParameters();
	int v1(0), v2(0);
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer->setBackgroundColor(0.1, 0.1, 0.1, v1);
	viewer->addText("Before remove outliers", 10, 10, "v1 text", v1);

	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer->setBackgroundColor(0.1, 0.1, 0.1, v2);
	viewer->addText("After remove outliers", 10, 10, "v2 text", v2);

  viewer->addPointCloud<PointT>(cloud1, "v1", v1);	
  viewer->addPointCloud<PointT>(cloud1al,"v2", v2); 
	
  viewer->spin();   

}

int main (int argc, char** argv)
{
  PointCloud::Ptr cloud (new PointCloud);
  PointCloud::Ptr cloud_filtered (new PointCloud);
  //PointCloud::Ptr result (new PointCloud);
 
  pcl::PCDReader reader;
  pcl::io::loadPCDFile (argv[1], *cloud);
  
  pcl::StatisticalOutlierRemoval<PointT> sor;
  sor.setInputCloud (cloud);
  sor.setMeanK (50);
  sor.setStddevMulThresh (0.4);
  sor.filter (*cloud_filtered);

  std::cout<<"离群点去除前，点云数目："<<cloud->points.size()<<endl;
  std::cout<<"离群点去除后，点云数目："<<cloud_filtered->points.size()<<endl;

  //string ldname = argv[1];
  
  std::stringstream ss1;
  ss1 << argv[1];    

  char* name = strchr(argv[1], '.pcd');
  size_t namelen = name-argv[1];
  string loadname = ss1.str().substr(0,namelen-3);
  std::stringstream ss;
  ss << loadname <<"_no_outlier.pcd";      
        
  pcl::io::savePCDFileBinary(ss.str (), *cloud_filtered);
  cout<<"文件写入了"<<ss.str()<<endl;
  viewPair(cloud,cloud_filtered);
  

  return (0);
}