
/*
程序实现
读入原始点云，滤出避雷器
*/
#include <iostream>
#include <string>
//#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/impl/io.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
//#include<pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>


using namespace std;
//using namespace cv;

typedef pcl::PointXYZRGBA PointT;
//typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

PointCloud::Ptr cloud (new PointCloud);
PointCloud::Ptr cloud_transformed (new PointCloud);
PointCloud::Ptr cloud_voxelfilter (new PointCloud);
PointCloud::Ptr cloud_conditionfilter (new PointCloud);
PointCloud::Ptr cloud_static (new PointCloud);
PointCloud::Ptr cloud_radiusfilter (new PointCloud);


int main(int argc, char** argv)
{
   
  if (pcl::io::loadPCDFile<PointT> (argv[1], *cloud) == -1) 
    {
        PCL_ERROR ("Couldn't read file this pcd \n");
        return (-1);
    }
    
    cout<<"raw point cloud size:  "<<cloud->points.size()<<"data points."<<endl;

    

    //1.条件滤波
    pcl::ConditionAnd<PointT>::Ptr range_cond (new pcl::ConditionAnd<PointT>());

    range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new
        pcl::FieldComparison<PointT> ("x", pcl::ComparisonOps::GT, -0.5)));
    range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new
        pcl::FieldComparison<PointT> ("x", pcl::ComparisonOps::LT, 0.5)));

    range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new
        pcl::FieldComparison<PointT> ("y", pcl::ComparisonOps::GT, -0.4)));
    range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new
        pcl::FieldComparison<PointT> ("y", pcl::ComparisonOps::LT, 0.25)));

    range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new
        pcl::FieldComparison<PointT> ("z", pcl::ComparisonOps::GT, 0.0)));
    range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new
        pcl::FieldComparison<PointT> ("z", pcl::ComparisonOps::LT, 1.5 )));

    pcl::ConditionalRemoval<PointT> condrem;
    condrem.setCondition (range_cond);
    condrem.setInputCloud (cloud);
    condrem.setKeepOrganized(true);
    condrem.filter (*cloud_conditionfilter);

    vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_conditionfilter,*cloud_conditionfilter,indices);
    cout << "after conditional filtering: " 
         <<cloud_conditionfilter->size()<<"data points."<<endl;
    
    
    pcl::io::savePCDFileASCII ("conditionFiltered.pcd", *cloud_conditionfilter);  
     
    //2.降采样(体素滤波器)
    pcl::VoxelGrid<PointT> filter_VG;
    filter_VG.setInputCloud(cloud_conditionfilter);
    filter_VG.setLeafSize(0.001f,0.001f,0.001f);
    filter_VG.filter(*cloud_voxelfilter);

    cout <<"after VoxelGrid filtering,"
         << cloud_voxelfilter->size()<< " data points." << std::endl;
    pcl::io::savePCDFileASCII ("voxelfilterd.pcd", *cloud_voxelfilter);

/*
    //3.半径滤波
    pcl::RadiusOutlierRemoval<PointT> outrem;
    outrem.setInputCloud(cloud_conditionfilter);
    outrem.setRadiusSearch(0.5);
    outrem.setMinNeighborsInRadius(200);
    outrem.filter(*cloud_radiusfilter);

    cout <<"after Radius_Outlier_Removal filtering,"
         << cloud_radiusfilter->size()<< " data points."<< std::endl;
    pcl::io::savePCDFileASCII ("radiusfilter.pcd", *cloud_radiusfilter); 
*/

//3.统计滤波器        
    pcl::StatisticalOutlierRemoval<PointT> filter_SOR;
    filter_SOR.setInputCloud(cloud_voxelfilter);
    filter_SOR.setMeanK(50);
    filter_SOR.setStddevMulThresh(1.0);
    filter_SOR.filter(*cloud_static);

    cout <<"after Statistical_Outlier_Removal filtering,"
        << cloud_static->size()<< " data points."  << std::endl;
    pcl::io::savePCDFileASCII("final_filtered.pcd",*cloud_static);  
    
    // Visualization
  pcl::visualization::PCLVisualizer viewer ("Arrester");
  // Create two verticaly separated viewports
  int v1 (0);
  int v2 (1);
  viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
  viewer.createViewPort (0.5, 0.0, 1.0, 1.0, v2);

  // The color we will be using
  float bckgr_gray_level = 0.0;  // Black
  float txt_gray_lvl = 1.0 - bckgr_gray_level;

  // Original point cloud is white
  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h (cloud, (int) 255 * txt_gray_lvl, (int) 255 * txt_gray_lvl, (int) 255 * txt_gray_lvl);
  viewer.addPointCloud (cloud, cloud_in_color_h, "cloud_in_v1", v1);
  // Filtered point cloud is green
  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tr_color_h (cloud_static, 20, 180, 20);
  viewer.addPointCloud (cloud_static, cloud_tr_color_h, "cloud_tr_v1", v2);
  // Adding text descriptions in each viewport
  viewer.addText ("White: Original point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
  viewer.addText ("Green: Filtered point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);  
  // Set background color
  viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
  viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);
  // Set camera position and orientation
  //viewer.setCameraPosition (-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
  viewer.setSize (1280, 1024);  // Visualiser window size

  // Display the visualiser
  while (!viewer.wasStopped ())
  {
    viewer.spinOnce ();
  }
    return 0;


}
