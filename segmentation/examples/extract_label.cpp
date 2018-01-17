#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;

void viewPair(PointCloud::Ptr cloud1, PointCloud::Ptr cloud1al)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    //pcl::visualization::PCLVisualizer viewer("3D viewer");
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

int main(int argc, char ** argv)
{
    pcl::PointCloud<pcl::PointXYZL>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZL>);
    PointCloud::Ptr cloud2 (new PointCloud);
    PointCloud::Ptr cloudout (new PointCloud);
    pcl::io::loadPCDFile("2_kinect_outlier_remove_out.pcd",*cloud1);
    pcl::io::loadPCDFile("2_kinect_outlier_remove.pcd",*cloud2);


    std::cout<<"全部点的个数："<<cloud1->points.size()<<endl;
    std::cout<<"全部点的个数："<<cloud2->points.size()<<endl;
    
    int count = 0;

    for (size_t i=0; i<cloud1->points.size(); i++)
    {
        if(cloud1->points[i].label==3)
        {
            ++count;
            cloudout->push_back(cloud2->points[i]);
        }  
            
 
        
    }
    std::cout<<"label的点的个数："<<count<<endl;   

    pcl::io::savePCDFile("seg_raw_2.pcd",*cloudout);
    viewPair(cloud2,cloudout);


    return 0;

}