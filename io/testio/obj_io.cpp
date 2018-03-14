#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;


void split(const string& src, const string& delim, vector<string>& dest)
{  
    string str = src;  
    string::size_type start = 0, index;  
    string substr;  
  
    index = str.find_first_of(delim, start);    //在str中查找(起始：start) delim的任意字符的第一次出现的位置  
    while(index != string::npos)  
    {  
        substr = str.substr(start, index-start);  
        dest.push_back(substr);  
        start = str.find_first_not_of(delim, index);    //在str中查找(起始：index) 第一个不属于delim的字符出现的位置  
        if(start == string::npos) return;  
  
        index = str.find_first_of(delim, start);  
    } 
    substr = str.substr(start, index-start);  
    dest.push_back(substr);  
}  
 


int main (int argc, char** argv)   
{
    /*
    pcl::TextureMesh mesh;
    pcl::io::loadOBJFile(argv[1] , mesh);  

    pcl::visualization::PCLVisualizer viewer("viewer");
    viewer.addTextureMesh(mesh, "texture", 0);      
    viewer.spin();
    return 0;    
    */
    
    PointCloudT::Ptr cloud (new PointCloudT);
    PointT point;

    ifstream infile(argv[1], ios::in);      
    string delim(" ");  
    string textline;  

    if(infile.good())  
    {  
        while(!infile.fail())  
        {  
            vector<string> line;
            getline(infile, textline);  
            split(textline, delim, line);             

            if(line.size()>6)
            {
                point.x = stod(line[1]);
                point.y = stod(line[2]);
                point.z = stod(line[3]);
                point.r = stoi(line[4]);
                point.g = stoi(line[5]);
                point.b = stoi(line[6]);
                cloud->push_back(point);
            }
           

            // if(line.size()>1)
            // {
            //     cout<<line[1]<<" "<<line[2]<<" "<<line[3]<<" "<<line[4]<<" "<<line[5]<<" "<<line[6]<<endl;
            // }
            

        }  
    }  
    infile.close();  
    
    pcl::visualization::CloudViewer viewer("test");
    viewer.showCloud(cloud);
    while (!viewer.wasStopped()){ }
    
  
    return 0;  




}


