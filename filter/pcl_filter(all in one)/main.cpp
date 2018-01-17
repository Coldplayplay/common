#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
 #include<pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>


int main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_PT (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_VG (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_SOR (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_ROR (new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *cloud) == -1) 
      {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return (-1);
      }
    std::cout << "Loaded cloud. \n"
            <<"before filtering,"
            << cloud->width * cloud->height
            << " data points."
            << std::endl;

    //直通滤波器
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0,1.0);
    pass.filter(*cloud_filtered_PT);

    std::cout <<"after passthrough filtering,"
            << cloud_filtered_PT->width * cloud_filtered_PT->height
            << " data points."
            << std::endl;

    //体素滤波器
    pcl::VoxelGrid<pcl::PointXYZ> filter_VG;
    filter_VG.setInputCloud(cloud);
    filter_VG.setLeafSize(0.01f,0.01f,0.01f);
    filter_VG.filter(*cloud_filtered_VG);

    std::cout <<"after VoxelGrid filtering,"
            << cloud_filtered_VG->width * cloud_filtered_VG->height
            << " data points."
            << std::endl;

    //统计滤波器
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> filter_SOR;
    filter_SOR.setInputCloud(cloud);
    filter_SOR.setMeanK(50);
    filter_SOR.setStddevMulThresh(1.0);
    filter_SOR.filter(*cloud_filtered_SOR);

    std::cout <<"after Statistical_Outlier_Removal filtering,"
            << cloud_filtered_SOR->width * cloud_filtered_SOR->height
            << " data points."
            << std::endl;

    //半径滤波器
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    outrem.setInputCloud(cloud);
    outrem.setRadiusSearch(0.8);
    outrem.setMinNeighborsInRadius(2);
    outrem.filter(*cloud_filtered_ROR);

    std::cout <<"after Radius_Outlier_Removal filtering,"
            << cloud_filtered_ROR->width * cloud_filtered_ROR->height
            << " data points."
            << std::endl;
    
    pcl::visualization::CloudViewer viewer1("test");
    viewer1.showCloud(cloud);
    //while (!viewer1.wasStopped()){ }

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer("3D viewer"));
    int v1(0);
    int v2(0);
    int v3(0);
    int v4(0);
   
    viewer->createViewPort(0.0,0.0,0.5,0.5,v1);
    viewer->setBackgroundColor(0,0,0,v1);
    
    viewer->createViewPort(0.5,0.0,1.0,0.5,v2);
    viewer->setBackgroundColor(0,0,0,v2);
    
    viewer->createViewPort(0.0,0.5,0.5,1.0,v3);
    viewer->setBackgroundColor(0,0,0,v3);

    viewer->createViewPort(0.5,0.5,1.0,1.0,v4);
    viewer->setBackgroundColor(0,0,0,v4);

    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    viewer->addPointCloud<pcl::PointXYZ>(cloud_filtered_PT,"pt",v1);
    viewer->addPointCloud<pcl::PointXYZ>(cloud_filtered_VG,"vg",v2);
    viewer->addPointCloud<pcl::PointXYZ>(cloud_filtered_SOR,"statistic_outlier_removal",v3);
    viewer->addPointCloud<pcl::PointXYZ>(cloud_filtered_ROR,"radius_outlier_removal",v4);

    while (!viewer1.wasStopped())
    {
    if(!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));

       
    }
    }
    return 0;
}
