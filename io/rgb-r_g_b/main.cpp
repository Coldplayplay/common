#include <stdio.h>
#include <stdlib.h>
#include <iostream>  
#include <pcl/io/pcd_io.h>  
#include <pcl/point_types.h>  
#include <pcl/filters/filter.h>
 
using namespace std;
int main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  if(pcl::io::loadPCDFile<pcl::PointXYZRGB>(argv[1], *cloud)==-1)
  {
    PCL_ERROR("error in reading the pcd file");
    return -1;
  }
  cout<<"原始cloud大小："<<cloud->size()<<std::endl;
  
  vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud,*cloud,indices);
  std::cout<<"删除NAN的cloud大小："<<cloud->size()<<std::endl;

  ofstream out("xyzrgb0.txt");
  if(out.is_open())
  {
    for(int i=0; i<cloud->size(); i++)
    {      
      /*
      int rgb = cloud->points[i].rgb;
      int r = (rgb>>16)&0x0000ff;
      int g = (rgb>>8)&0x0000ff;
      int b = (rgb)&0x0000ff;
      */

      float x = cloud->points[i].x;
      float y = cloud->points[i].y;
      float z = cloud->points[i].z;      
      int r = cloud->points[i].r;
      int g = cloud->points[i].g;
      int b = cloud->points[i].b;//困惑是为什么不能直接用cout输出，非给强行转换成int类型才行

      out<<x<<" ";
      out<<y<<" ";
      out<<z<<" ";
      out<<r<<" ";
      out<<g<<" ";
      out<<b<<" ";
      
      out<<"\n";

    }
    out.close();
  }

  unsigned long rgb = *reinterpret_cast<int*>(&cloud->points[19555].rgb);
  int r = (rgb>>16)&0x0000ff;
  int g = (rgb>>8)&0x0000ff;
  int b = (rgb)&0x0000ff;
  PCL_INFO("第19555个点rgb是%lu",rgb);//%lu for unsigned long//这个数值搞不懂
  cout<<endl;

  PCL_INFO("第19555个点r,g,b是[%d, %d, %d]",r,g,b);
  cout<<endl;

  PCL_INFO("第19555个点的r,g,b是[%d, %d, %d]",cloud->points[19555].r,cloud->points[19555].g,cloud->points[19555].b);
  cout<<endl;


  
  return 0;

} 

