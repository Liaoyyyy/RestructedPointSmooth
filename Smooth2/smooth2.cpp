/****************************
 * 给定一个融合后的点云，对其进行下采样和滤波。
 * 再进行平滑（输出结果），然后计算法线，并讲法线显示在平滑后的点云上。
****************************/

#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d.h>

typedef pcl::PointXYZRGB PointT;

int main(int argc, char** argv)
{

	// Load input file
	//pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downSampled(new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_smoothed(new pcl::PointCloud<pcl::PointXYZRGB>);
	//int NormalEst = 0;
	//int viewer = 0;
	if (pcl::io::loadPCDFile("r.pcd", *cloud_filtered) == -1)
	{
		cout << "点云数据读取失败！" << endl;
	}

	//std::cout << "Orginal points number: " << cloud_filtered->points.size() << std::endl;

	// 下采样，同时保持点云形状特征
	//pcl::VoxelGrid<PointT> downSampled;  //创建滤波对象
	//downSampled.setInputCloud(cloud);            //设置需要过滤的点云给滤波对象
	//downSampled.setLeafSize(0.01f, 0.01f, 0.01f);  //设置滤波时创建的体素体积为1cm的立方体
	//downSampled.filter(*cloud_downSampled);           //执行滤波处理，存储输出

	//pcl::io::savePCDFile("downsampledPC.pcd", *cloud_downSampled);

	// 统计滤波
	//pcl::StatisticalOutlierRemoval<PointT> statisOutlierRemoval;       //创建滤波器对象
	//statisOutlierRemoval.setInputCloud(cloud_downSampled);            //设置待滤波的点云
	//statisOutlierRemoval.setMeanK(50);                                //设置在进行统计时考虑查询点临近点数
	//statisOutlierRemoval.setStddevMulThresh(6.0);                     //设置判断是否为离群点的阀值:均值+1.0*标准差
	//statisOutlierRemoval.filter(*cloud_filtered);                     //滤波结果存储到cloud_filtered

	//pcl::io::savePCDFile("filteredPC.pcd", *cloud_filtered);
	// ----------------------开始你的代码--------------------------//
	// 请参考PCL官网实现以下功能
	// 对点云重采样
	pcl::search::KdTree<PointT>::Ptr treeSampling(new pcl::search::KdTree<PointT>);// 创建用于最近邻搜索的KD-Tree
	pcl::PointCloud<PointT> mls_point;    //输出MLS
	pcl::MovingLeastSquares<PointT, PointT> mls; // 定义最小二乘实现的对象mls
	mls.setComputeNormals(false);  //设置在最小二乘计算中是否需要存储计算的法线
	mls.setInputCloud(cloud_filtered);         //设置待处理点云
	mls.setPolynomialOrder(2);            // 拟合2阶多项式拟合
	mls.setPolynomialFit(false);     // 设置为false可以 加速 smooth
	mls.setSearchMethod(treeSampling);         // 设置KD-Tree作为搜索方法
	mls.setSearchRadius(0.16);           // 单位m.设置用于拟合的K近邻半径
	mls.process(mls_point);                 //输出

	// 输出重采样结果
	cloud_smoothed = mls_point.makeShared();
	std::cout << "cloud_smoothed: " << cloud_smoothed->size() << std::endl;

	//save cloud_smoothed
	pcl::io::savePCDFileASCII("cloud_smoothed.pcd", *cloud_smoothed);


		//// 法线估计
		//pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normalEstimation;             //创建法线估计的对象
		//normalEstimation.setInputCloud(cloud_smoothed);                         //输入点云
		//pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);// 创建用于最近邻搜索的KD-Tree
		//normalEstimation.setSearchMethod(tree);
		//pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>); // 定义输出的点云法线
		//// K近邻确定方法，使用k个最近点，或者确定一个以r为半径的圆内的点集来确定都可以，两者选1即可
		//normalEstimation.setKSearch(10);// 使用当前点周围最近的10个点
		////normalEstimation.setRadiusSearch(0.03);            //对于每一个点都用半径为3cm的近邻搜索方式
		//normalEstimation.compute(*normals);               //计算法线

		/*std::cout << "normals: " << normals->size() << ", " << "normals fields: " << pcl::getFieldsList(*normals) << std::endl;
		pcl::io::savePCDFileASCII("normals.pcd", *normals);
*/

		//// 显示结果
		////pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> color(cloud);
		//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("PCL Viewer"));
		//pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> color(cloud);
		//viewer->setBackgroundColor(0, 0, 0);
		//pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_smoothed);
		//viewer->addPointCloud<pcl::PointXYZRGB>(cloud_smoothed, rgb, "smooth cloud");
		//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "smooth cloud");
		//viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud_smoothed, normals, 20, 0.05, "normals");

		//viewer->initCameraParameters();

		//while (!viewer->wasStopped())
		//{
		//	viewer->spinOnce(100);
		//	boost::this_thread::sleep(boost::posix_time::microseconds(100000));
		//}


	return 1;
}
