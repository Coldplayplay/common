#include <pcl/visualization/pcl_visualizer.h> 
#include <iostream>

int 
main (int argc, char *argv[]) 
{ 
  pcl::visualization::PCLVisualizer viewer ("PCL visualizer"); 
  viewer.addCube (16.2, 18.2, -7.9, -5.9, 0.21, 2.21, 1.0, 0, 0, "cube", 0);   
  viewer.addCoordinateSystem (2.5, "axis", 0); 

  while (!viewer.wasStopped ()) 
  { 
    viewer.spinOnce (); 
  } 
  return (0); 
}
