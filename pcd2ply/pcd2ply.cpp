#include <pcl/io/pcd_io.h>  
#include <pcl/io/ply_io.h>  
#include<pcl/PCLPointCloud2.h>  
#include<iostream> 
#include<string>   
using namespace pcl;  
using namespace pcl::io; using namespace std;    
int PCDtoPLYconvertor(string & input_filename ,string& output_filename) 
{
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
	//pcl::PCLPointCloud2 cloud;  
	 pcl::PointCloud<pcl::PointXYZRGB> cloud;
	//loadPCDFile(input_filename , cloud
	if ( pcl::io::loadPCDFile("001_1.pcd", cloud) <  0)
	{  		
		cout << "Error: cannot load the PCD file!!!"<< endl;  		
		return -1; 
	}  	
	PLYWriter writer;  	
	writer.write(output_filename, cloud);  	
	//writer.write(output_filename, cloud, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), false, true);
	return 0; 
}   
int main()
 { 
	string input_filename = "001_1.pcd"; 	
	string output_filename = "001_1.ply";  	
	PCDtoPLYconvertor(input_filename , output_filename);  	
	return 0; 
}