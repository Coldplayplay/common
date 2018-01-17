 #include <iostream>
 #include <pcl/io/pcd_io.h>
 #include <pcl/point_types.h>
 #include <pcl/filters/filter.h>
 #include<pcl/visualization/cloud_viewer.h>
 int main (int argc, char** argv)   
 {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3 (new pcl::PointCloud<pcl::PointXYZ>);
      if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *cloud1) == -1) 
      {
        PCL_ERROR ("Couldn't read pcd file \n");
        return (-1);
      }
      std::vector<int> indices;
      pcl::removeNaNFromPointCloud(*cloud1,*cloud1, indices);
      std::cout << "Loaded "<< cloud1->points.size()<< " data points "  << std::endl;
      
      if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[2], *cloud2) == -1) 
      {
        PCL_ERROR ("Couldn't read pcd file \n");
        return (-1);
      }
      std::cout << "Loaded "<< cloud2->points.size()<< " data points "  << std::endl;
      
      if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[3], *cloud3) == -1) 
      {
        PCL_ERROR ("Couldn't read pcd file \n");
        return (-1);
      }
      std::cout << "Loaded "<< cloud3->points.size()<< " data points "  << std::endl;
      
      pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud (new pcl::PointCloud<pcl::PointXYZ>);
      //*final_cloud=*cloud1+*cloud2+*cloud3;
      *final_cloud+=*cloud1;
      *final_cloud+=*cloud2;
      *final_cloud+=*cloud3;
      pcl::io::savePCDFile("result.pcd",*final_cloud,true);
      
      pcl::visualization::CloudViewer viewer("show result");
      viewer.showCloud(final_cloud);
      while (!viewer.wasStopped()){ }
      return (0);
 }
