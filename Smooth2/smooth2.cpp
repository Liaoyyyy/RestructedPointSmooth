/****************************
 * ����һ���ںϺ�ĵ��ƣ���������²������˲���
 * �ٽ���ƽ��������������Ȼ����㷨�ߣ�����������ʾ��ƽ����ĵ����ϡ�
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
		cout << "�������ݶ�ȡʧ�ܣ�" << endl;
	}

	//std::cout << "Orginal points number: " << cloud_filtered->points.size() << std::endl;

	// �²�����ͬʱ���ֵ�����״����
	//pcl::VoxelGrid<PointT> downSampled;  //�����˲�����
	//downSampled.setInputCloud(cloud);            //������Ҫ���˵ĵ��Ƹ��˲�����
	//downSampled.setLeafSize(0.01f, 0.01f, 0.01f);  //�����˲�ʱ�������������Ϊ1cm��������
	//downSampled.filter(*cloud_downSampled);           //ִ���˲������洢���

	//pcl::io::savePCDFile("downsampledPC.pcd", *cloud_downSampled);

	// ͳ���˲�
	//pcl::StatisticalOutlierRemoval<PointT> statisOutlierRemoval;       //�����˲�������
	//statisOutlierRemoval.setInputCloud(cloud_downSampled);            //���ô��˲��ĵ���
	//statisOutlierRemoval.setMeanK(50);                                //�����ڽ���ͳ��ʱ���ǲ�ѯ���ٽ�����
	//statisOutlierRemoval.setStddevMulThresh(6.0);                     //�����ж��Ƿ�Ϊ��Ⱥ��ķ�ֵ:��ֵ+1.0*��׼��
	//statisOutlierRemoval.filter(*cloud_filtered);                     //�˲�����洢��cloud_filtered

	//pcl::io::savePCDFile("filteredPC.pcd", *cloud_filtered);
	// ----------------------��ʼ��Ĵ���--------------------------//
	// ��ο�PCL����ʵ�����¹���
	// �Ե����ز���
	pcl::search::KdTree<PointT>::Ptr treeSampling(new pcl::search::KdTree<PointT>);// �������������������KD-Tree
	pcl::PointCloud<PointT> mls_point;    //���MLS
	pcl::MovingLeastSquares<PointT, PointT> mls; // ������С����ʵ�ֵĶ���mls
	mls.setComputeNormals(false);  //��������С���˼������Ƿ���Ҫ�洢����ķ���
	mls.setInputCloud(cloud_filtered);         //���ô��������
	mls.setPolynomialOrder(2);            // ���2�׶���ʽ���
	mls.setPolynomialFit(false);     // ����Ϊfalse���� ���� smooth
	mls.setSearchMethod(treeSampling);         // ����KD-Tree��Ϊ��������
	mls.setSearchRadius(0.16);           // ��λm.����������ϵ�K���ڰ뾶
	mls.process(mls_point);                 //���

	// ����ز������
	cloud_smoothed = mls_point.makeShared();
	std::cout << "cloud_smoothed: " << cloud_smoothed->size() << std::endl;

	//save cloud_smoothed
	pcl::io::savePCDFileASCII("cloud_smoothed.pcd", *cloud_smoothed);


		//// ���߹���
		//pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normalEstimation;             //�������߹��ƵĶ���
		//normalEstimation.setInputCloud(cloud_smoothed);                         //�������
		//pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);// �������������������KD-Tree
		//normalEstimation.setSearchMethod(tree);
		//pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>); // ��������ĵ��Ʒ���
		//// K����ȷ��������ʹ��k������㣬����ȷ��һ����rΪ�뾶��Բ�ڵĵ㼯��ȷ�������ԣ�����ѡ1����
		//normalEstimation.setKSearch(10);// ʹ�õ�ǰ����Χ�����10����
		////normalEstimation.setRadiusSearch(0.03);            //����ÿһ���㶼�ð뾶Ϊ3cm�Ľ���������ʽ
		//normalEstimation.compute(*normals);               //���㷨��

		/*std::cout << "normals: " << normals->size() << ", " << "normals fields: " << pcl::getFieldsList(*normals) << std::endl;
		pcl::io::savePCDFileASCII("normals.pcd", *normals);
*/

		//// ��ʾ���
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
