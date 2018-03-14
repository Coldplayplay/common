#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

/*
点云过于稠密，不方便鼠标选点，于是进行降采样，但是又要获取该点在raw点云中的标号及相关信息
所以pclvisualizer显示的是降采样的点云，鼠标选好后，对改点在raw点云中进行最近邻搜索，反馈最近点在raw点云中的标号及所有信息。
*/

// Mutex: //
boost::mutex cloud_mutex;

struct callback_args
{
    // structure used to pass arguments to the callback function
    PointCloudT::Ptr cloud;    
    PointCloudT::Ptr clicked_points_3d;
    pcl::search::KdTree<PointT> search;
    pcl::visualization::PCLVisualizer::Ptr viewerPtr;
};

void pp_callback(const pcl::visualization::PointPickingEvent& event, void* args)
{
    struct callback_args* data = (struct callback_args *)args;
    int idx = event.getPointIndex();
    if (idx == -1)
        return;
    PointT current_point;
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
    
    data->clicked_points_3d->points.push_back(current_point);

    // Draw clicked points in red:
    pcl::visualization::PointCloudColorHandlerCustom<PointT> color(data->clicked_points_3d, 0, 0, 255);
    data->viewerPtr->removePointCloud("clicked_points");
    data->viewerPtr->addPointCloud(data->clicked_points_3d, color, "clicked_points");
    data->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");
    
    
}

int main(int argc, char** argv)
{
    //visualizer
    bool use_voxel = false;
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    pcl::PointCloud<PointT>::Ptr cloud_voxel(new pcl::PointCloud<PointT>());
    if (pcl::io::loadPCDFile(argv[1], *cloud))
    {
        std::cerr << "ERROR: Cannot open file " << argv[1] << "! Aborting..." << std::endl;
        return -1;
    }
    std::cout <<"降体素前："<< cloud->points.size() << std::endl;   

    cloud_mutex.lock();    // for not overwriting the point cloud

    PointCloudT::Ptr clicked_points_3d(new PointCloudT);
    pcl::search::KdTree<PointT> search;    
    search.setInputCloud(cloud);

    // Display pointcloud:
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("viewer"));
    if(use_voxel)
    {
        //降体素
        pcl::VoxelGrid<PointT> sor;
        sor.setInputCloud (cloud);
        sor.setLeafSize (0.01f, 0.01f, 0.1f);
        sor.filter (*cloud_voxel);
        std::cout <<"降体素后"<< cloud_voxel->points.size() << std::endl;    
        pcl::io::savePCDFileASCII("voxel.pcd",*cloud_voxel);
            
        pcl::visualization::PointCloudColorHandlerCustom<PointT> color(cloud_voxel, 255, 0, 0);
        viewer->addPointCloud(cloud_voxel, color, "scene");
    }
    else
    {
        pcl::visualization::PointCloudColorHandlerCustom<PointT> color(cloud, 255, 0, 0);
        viewer->addPointCloud(cloud, color, "scene");
    }
    

    viewer->setCameraPosition(0, -2, -2, 0, 0, 0, 0, 0, 0, 0);

    // Add point picking callback to viewer:
    struct callback_args cb_args;
    cb_args.cloud = cloud;   
    cb_args.clicked_points_3d = clicked_points_3d;
    cb_args.search = search;
    cb_args.viewerPtr = pcl::visualization::PCLVisualizer::Ptr(viewer);
    viewer->registerPointPickingCallback(pp_callback, (void*)&cb_args);
    std::cout << "Shift+click on three floor points, then press 'Q'..." << std::endl;

    // Spin until 'Q' is pressed:
    viewer->spin();
    std::cout << "done." << std::endl;

    cloud_mutex.unlock();

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
}
