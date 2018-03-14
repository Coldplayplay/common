 #include <iostream>
 #include <pcl/io/pcd_io.h>
 #include <pcl/point_types.h>
 #include <pcl/filters/filter.h>
 #include<pcl/visualization/cloud_viewer.h>
 using namespace std;
 typedef pcl::PointXYZRGB PointT;
 int main (int argc, char** argv)   
 {
      pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
      if (pcl::io::loadPCDFile<PointT> (argv[1], *cloud) == -1) 
      {
        PCL_ERROR ("Couldn't read pcd file \n");
        return (-1);
      }
      std::cout << "Loaded "
                << cloud->width * cloud->height
                << " data points from test_pcd.pcd with the following fields: "
                << std::endl;
            
      vector<int> indices;
      pcl::removeNaNFromPointCloud(*cloud,*cloud,indices);
      cout<<"left size:"<<cloud->size()<<endl;
      pcl::io::savePCDFile("(no_nan)cloud0.pcd",*cloud);
      
      pcl::visualization::CloudViewer viewer("test");
      viewer.showCloud(cloud);
      while (!viewer.wasStopped()){ }
      return (0);
 }
