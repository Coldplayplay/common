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

struct callback_args
{
    // structure used to pass arguments to the callback function
    int count;
    PointCloudT::Ptr cloud;
    pcl::PointCloud<pcl::Normal>::Ptr normals;    
    PointCloudT::Ptr clicked_points_3d;
    pcl::search::KdTree<PointT> search;
    pcl::visualization::PCLVisualizer::Ptr viewerPtr;
};

//鼠标回调函数
void pp_callback(const pcl::visualization::PointPickingEvent& event, void* args)
{
    struct callback_args* data = (struct callback_args *)args;
    int idx = event.getPointIndex();
    if (idx == -1)
        return;
    PointT current_point;
    PointT normal_point;
    std::vector<int> indices(1);
    std::vector<float> distances(1);
    event.getPoint(current_point.x, current_point.y, current_point.z);
    data->search.nearestKSearch(current_point, 1, indices, distances);
    PCL_INFO ("1 Point_voxel index picked: %d [%f, %f, %f]\n", 
             idx, current_point.x, current_point.y, current_point.z);    
    
    PCL_INFO("2 Point index nearestSearch: %d [%f, %f, %f] [%d, %d, %d]\n",
             indices[0], 
             data->cloud->points[indices[0]].x, data->cloud->points[indices[0]].y, data->cloud->points[indices[0]].z 
           , data->cloud->points[indices[0]].r, data->cloud->points[indices[0]].g, data->cloud->points[indices[0]].b);
        
    normal_point.x = data->normals->points[indices[0]].normal[0];
    normal_point.y = data->normals->points[indices[0]].normal[1];
    normal_point.z = data->normals->points[indices[0]].normal[2];

    data->clicked_points_3d->points.push_back(current_point);
    data->clicked_points_3d->points.push_back(normal_point);

    cout<<"当前进入回调函数的次数"<<data->count<<endl;
    //cout<<"当前clicked_points一共有"<<data->clicked_points_3d->size()<<endl;    

    // Draw clicked points in red:
    pcl::visualization::PointCloudColorHandlerCustom<PointT> color(data->clicked_points_3d, 0, 0, 255);
    data->viewerPtr->removePointCloud("clicked_points");    
    data->viewerPtr->addPointCloud(data->clicked_points_3d, color, "clicked_points");
    data->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");
    
    //data->viewerPtr->removeShape("arrow");
    stringstream ss;
    ss<<"arrow"<<data->count;
    data->viewerPtr->addArrow(normal_point, current_point, 255, 0, 0, false, ss.str());
    data->count++;
}

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
    bool use_voxel = false;
    boost::mutex cloud_mutex;
    int count = 1;
    PointCloudT::Ptr cloud (new PointCloudT);
    PointCloudT::Ptr clicked_points_3d(new PointCloudT);
    pcl::PointCloud<PointT>::Ptr cloud_voxel(new pcl::PointCloud<PointT>());
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>); 

    pcl::search::KdTree<PointT> search;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer
    (new pcl::visualization::PCLVisualizer("3D Viewer"));
    int v1(0); int v2(0);
    struct callback_args cb_args;
    
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
    std::cout <<"降体素前："<< cloud->points.size() << std::endl;   

    cloud_mutex.lock(); 

    float radius = 0.03;//默认值为0.03
    pcl::console::parse(argc, argv, "-r", radius);

    if(use_voxel)
    {
        //降体素
        pcl::VoxelGrid<PointT> sor;
        sor.setInputCloud (cloud);
        sor.setLeafSize (0.01f, 0.01f, 0.1f);
        sor.filter (*cloud_voxel);
        std::cout <<"降体素后"<< cloud_voxel->points.size() << std::endl;    
        //pcl::io::savePCDFileASCII("voxel.pcd",*cloud_voxel);            
        //pcl::visualization::PointCloudColorHandlerCustom<PointT> color(cloud_voxel, 255, 0, 0);
        normals = getNormals(cloud_voxel, radius);
        viewer->addPointCloud(cloud_voxel,"raw", v1);
        viewer->addPointCloudNormals<PointT, pcl::Normal>(cloud_voxel, normals, 10, 0.05, "pointnormals", v2);
        search.setInputCloud(cloud_voxel);
        cb_args.cloud = cloud_voxel;
    }
    else
    {
        //pcl::visualization::PointCloudColorHandlerCustom<PointT> color(cloud, 255, 0, 0);
        normals = getNormals(cloud, radius);
        viewer->addPointCloud(cloud,"raw", v1);
        viewer->addPointCloudNormals<PointT, pcl::Normal>(cloud, normals, 10, 0.05, "pointnormals", v2);
        search.setInputCloud(cloud);
        cb_args.cloud = cloud;
    }    
        
    // Add point picking callback to viewer:    
    cb_args.count = count;
    //cb_args.cloud = cloud;   
    cb_args.normals = normals;
    cb_args.clicked_points_3d = clicked_points_3d;
    cb_args.search = search;
    cb_args.viewerPtr = pcl::visualization::PCLVisualizer::Ptr(viewer);
    viewer->registerPointPickingCallback(pp_callback, (void*)&cb_args);
    std::cout << "Shift+click on three floor points, then press 'Q'..." << std::endl;
    
    // Spin until 'Q' is pressed:
    viewer->spin();
    std::cout << "done." << std::endl;

    cloud_mutex.unlock();
    
    while (!viewer->wasStopped ()) 
    { 
        viewer->spinOnce (); 
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    } 
    return (0); 

}