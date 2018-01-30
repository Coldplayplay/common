#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

using namespace std;
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

//获取法向量
pcl::PointCloud<pcl::Normal>::Ptr getNormals(PointCloudT::Ptr cloud, float radius)
{
   pcl::NormalEstimation<PointT,pcl::Normal> ne;
   pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
   pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
   ne.setInputCloud(cloud);
   ne.setSearchMethod(tree);
   ne.setRadiusSearch(radius);
   ne.compute(*normals);
   return normals;
}

int main(int argc, char** argv)
{
   

    PointCloudT::Ptr cloud (new PointCloudT);   
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>); 
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer
    (new pcl::visualization::PCLVisualizer("3D Viewer"));
    int v1(0); int v2(0);

/*
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viewer->setBackgroundColor(0, 0, 0, v1);
    viewer->setBackgroundColor(0, 0, 0, v2);
*/
    
    if (argc != 4)
    {
    cout<<"Usage: ./main <xxx.pcd> -r <tree_radius>"<<endl;
    return -1;
    }
    
    if(pcl::io::loadPCDFile<PointT>(argv[1], *cloud)==-1)
    {
        PCL_ERROR("wrong in reading pcd file.\n");
        return -1;
    }
   
    float radius = 0.03;//默认值为0.03
    pcl::console::parse(argc, argv, "-r", radius);

    normals = getNormals(cloud, radius);
//四种获得normal具体信息的方式
    std::cout<<normals->points[100]<<std::endl;
    std::cout<<"["<<normals->points[100].normal_x<<" "
                  <<normals->points[100].normal_y<<" "
                  <<normals->points[100].normal_z<<" "
                  <<normals->points[100].curvature<<"]"<<endl;

    std::cout<<"["<<normals->points[100].normal[0]<<" "
                  <<normals->points[100].normal[1]<<" "
                  <<normals->points[100].normal[2]<<" "
                  <<normals->points[100].curvature<<"]"<<endl;

    std::cout<<"["<<normals->points[100].data_n[0]<<" "
                  <<normals->points[100].data_n[1]<<" "
                  <<normals->points[100].data_n[2]<<" "
                  <<normals->points[100].curvature<<"]"<<endl;
   

    viewer->addPointCloud(cloud,"raw", v1);
    viewer->addPointCloudNormals<PointT, pcl::Normal>(cloud, normals, 10, 0.05, "pointnormals", v2);
    
    while (!viewer->wasStopped ()) 
    { 
        viewer->spinOnce (); 
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    } 
    return (0); 

}