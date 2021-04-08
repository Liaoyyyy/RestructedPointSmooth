
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
int
main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
	// 填入点云数据
	pcl::PCDReader reader;
	// 把路径改为自己存放文件的路径
	reader.read<pcl::PointXYZRGB>("002.pcd", *cloud);//note：此处是PCD文件的绝对路径，最好放在此工程同一目录下
	std::cerr << "Cloud before filtering: " << std::endl;
	std::cerr << *cloud << std::endl;
	// 创建滤波器对象
	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
	sor.setInputCloud(cloud);//设置待滤波的点云
	sor.setMeanK(32);//设置在进行统计时考虑查询点邻近点数
	sor.setStddevMulThresh(2.0f);//设置判断是否为离群的的阈值 设置标准差的系数, 值越大，丢掉的点越少
	sor.filter(*cloud_filtered);//执行滤波处理，保存内点到cloud_filtered
	std::cerr << "Cloud after filtering: " << std::endl;
	std::cerr << *cloud_filtered << std::endl;
	//剩下的数据（内部点）将被存入磁盘
	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZRGB>("002_1.pcd", *cloud_filtered, false);
	//使用同样的参数，再次调用该滤波器，但是使用函数setNegative设置输出取外点，以获取离群点数据
	sor.setNegative(true);
	sor.filter(*cloud_filtered);
	writer.write<pcl::PointXYZRGB>("002_2.pcd", *cloud_filtered, false);//将数据写回到磁盘
	system("pause");
	return (0);

}
