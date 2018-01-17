#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
using namespace std;
using pcl::visualization::PointCloudColorHandlerCustom;

PointT searchPoint;
PointCloudT::Ptr cloud (new PointCloudT);
PointCloudT::Ptr neighs (new PointCloudT);
PointCloudT::Ptr radius_neighs (new PointCloudT);
//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));  

void searchneighs();
void pointpickEventOccurred(const pcl::visualization::PointPickingEvent& event, void* view_void)
{
	cout<<"start picking points..."<<endl;
	int index = event.getPointIndex();
	if(index == -1)
		return;
	event.getPoint(searchPoint.x, searchPoint.y, searchPoint.z);
	PCL_INFO ("Clicked point %d with X:%f Y:%f Z:%f\n", index, searchPoint.x, searchPoint.y, searchPoint.z);
	searchneighs();
}

void searchneighs()
{
	neighs->clear();
	radius_neighs->clear();
	pcl::KdTreeFLANN<PointT> kdtree;
	kdtree.setInputCloud(cloud);
//k个最近邻
	int k = 20;
	vector<int> indices;
	vector<float> distances;
	kdtree.nearestKSearch(searchPoint, k, indices, distances);
//固定半径内的邻居
	float radius = 0.1;	
	vector<int> radius_indices;
	vector<float> radius_distances;	
	kdtree.radiusSearch(searchPoint, radius, radius_indices, radius_distances);
//储存选中的邻居
	for(int i=0; i<indices.size(); i++)
	{
		neighs->push_back(cloud->points[indices[i]]);
	}
	for(int i=0; i<radius_indices.size(); i++)
	{
		radius_neighs->push_back(cloud->points[radius_indices[i]]);
	}	
}
int main(int argc, char** argv)
{
	pcl::io::loadPCDFile(argv[1], *cloud);
	searchPoint.x = 0.1;
	searchPoint.y = 0.1;
	searchPoint.z = 0.1;

	searchneighs();
	//viewPair(cloud, neighs, radius_neighs);
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));  
	viewer->initCameraParameters();
	viewer->registerPointPickingCallback(&pointpickEventOccurred, (void*)viewer.get());//没有取地址符号！！！debug了半天
	int v1(0), v2(0);
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addText("Before Alignment", 10, 10, "v1 text");
	PointCloudColorHandlerCustom<PointT> green(cloud, 0, 255, 0);
	PointCloudColorHandlerCustom<PointT> red(neighs, 255, 0, 0);
	viewer->addPointCloud(cloud, green, "v1_target");
	viewer->addPointCloud(neighs, red, "v1_source");

	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer->setBackgroundColor(0, 0, 0, v2);
	viewer->addText("After Alignment", 10, 10, "v2 text", v2);	
	PointCloudColorHandlerCustom<PointT> red2(radius_neighs, 255, 0, 0);
	viewer->addPointCloud(cloud, green, "v2_target", v2);
	viewer->addPointCloud(radius_neighs, red2, "v2_source", v2);

	while(!viewer->wasStopped())
	{
		viewer->updatePointCloud(neighs, red, "v1_source");
		viewer->updatePointCloud(radius_neighs, red2, "v2_source");
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(10000));
	}

	return 0;
}

