#include <cv.h>
#include <iostream>

#include <vector>
#include <Eigen/Geometry>
#include <Eigen/StdVector> 
 using namespace std;
 using namespace cv;
 using namespace Eigen;
 
 EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Affine3d)
 
 class mat2eigen
 {
 public:
 	vector<Mat> A1;
 	vector<Eigen::Affine3d> B1;
 	
 	mat2eigen()
 	{}
 	~mat2eigen()
 	{}
 	
 	
 	Eigen::Affine3d Mat2Affine3d(Mat &A)
	{
	    Eigen::Affine3d B;
	    for(int i=0; i<4; i++)
		for(int j=0; j<4; j++)
		    B(i,j) = A.at<float>(i,j);
	    return B;
	}
	
	void vector_Mat2Affine3d()
	{
	    //vector<Eigen::Affine3d> B1;
	    for(int i=0; i<A1.size(); i++)
		B1.push_back(Mat2Affine3d(A1[i]));
	    //return B1;	
	}
	
	void printAffine3d(Eigen::Affine3d A)
	    {
		for(int i=0; i<4; i++)
		    {
		    for(int j=0; j<4; j++)
		        std::cout<<A(i,j);
		    std::cout<<endl;
		    }
		std::cout<<endl;
	    }
 
 
 
 
 
 };
 /*
 Eigen::Affine3d Mat2Affine3d(Mat &A)
	{
	    Eigen::Affine3d B;
	    for(int i=0; i<4; i++)
		for(int j=0; j<4; j++)
		    B(i,j) = A.at<float>(i,j);
	    return B;
	}
	
	vector<Eigen::Affine3d> vector_Mat2Affine3d(vector<Mat>& A1)
	{
	    vector<Eigen::Affine3d> B1;
	    for(int i=0; i<A1.size(); i++)
		B1.push_back(Mat2Affine3d(A1[i]));
	    return B1;	
	}
	
	void printAffine3d(Eigen::Affine3d A)
	    {
		for(int i=0; i<4; i++)
		    {
		    for(int j=0; j<4; j++)
		        std::cout<<A(i,j);
		    std::cout<<endl;
		    }
		std::cout<<endl;
	    }
*/

int main(int argc, char**argv)
{
	mat2eigen hhh;
	vector<Mat> AA;
 	vector<Eigen::Affine3d> BB;
	Mat A(4,4,CV_32FC1,1);
	Mat B(4,4,CV_32FC1,2);
	AA.push_back(A);
	AA.push_back(B);
	//Eigen::Affine3d B;
	hhh.A1 = AA;
	hhh.vector_Mat2Affine3d();
	hhh.printAffine3d(hhh.B1[1]);
	return 0;
}
