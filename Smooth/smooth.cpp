
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
int
main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
	// �����������
	pcl::PCDReader reader;
	// ��·����Ϊ�Լ�����ļ���·��
	reader.read<pcl::PointXYZRGB>("002.pcd", *cloud);//note���˴���PCD�ļ��ľ���·������÷��ڴ˹���ͬһĿ¼��
	std::cerr << "Cloud before filtering: " << std::endl;
	std::cerr << *cloud << std::endl;
	// �����˲�������
	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
	sor.setInputCloud(cloud);//���ô��˲��ĵ���
	sor.setMeanK(32);//�����ڽ���ͳ��ʱ���ǲ�ѯ���ڽ�����
	sor.setStddevMulThresh(2.0f);//�����ж��Ƿ�Ϊ��Ⱥ�ĵ���ֵ ���ñ�׼���ϵ��, ֵԽ�󣬶����ĵ�Խ��
	sor.filter(*cloud_filtered);//ִ���˲����������ڵ㵽cloud_filtered
	std::cerr << "Cloud after filtering: " << std::endl;
	std::cerr << *cloud_filtered << std::endl;
	//ʣ�µ����ݣ��ڲ��㣩�����������
	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZRGB>("002_1.pcd", *cloud_filtered, false);
	//ʹ��ͬ���Ĳ������ٴε��ø��˲���������ʹ�ú���setNegative�������ȡ��㣬�Ի�ȡ��Ⱥ������
	sor.setNegative(true);
	sor.filter(*cloud_filtered);
	writer.write<pcl::PointXYZRGB>("002_2.pcd", *cloud_filtered, false);//������д�ص�����
	system("pause");
	return (0);

}
