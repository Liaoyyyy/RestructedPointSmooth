 #include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/vtk_io.h>
#include <vtkPolyData.h>

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/vtk_io.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <pcl/visualization/cloud_viewer.h>  

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

int main()
{
	pcl::PCLPointCloud2 clod;
	//pcl::PLYReader reader;
	pcl::PCDReader pcdReader;
	pcdReader.read("cloud_37.pcd", clod);
	//reader.read("r.ply", clod);
	//pcl::PCDWriter writer;
	//writer.writeASCII("r.pcd", clod);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::io::loadPCDFile("cloud_37.pcd", *cloud);
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewe(new pcl::visualization::PCLVisualizer("ss"));
	viewe->setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> color(cloud);
	viewe->addPointCloud<pcl::PointXYZRGB>(cloud, color, "cloud");
	while (!viewe->wasStopped()) {
		viewe->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	return 0;
}