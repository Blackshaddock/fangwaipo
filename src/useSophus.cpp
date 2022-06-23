#include <iostream>
#include <cmath>
#include "param.h"
#include "io.h"
#include "util.hpp"
#include <pcl/io/pcd_io.h>

//using namespace std;


int main( int argc, char** argv )
{
    //1. read the config file
	if (argc != 2)
	{
		cout << "Please input the params_camera path!" << endl;
		return 0;
	}
	std::string config_file(argv[1]);
	if (config_file.empty())
	{
		std::cout << "Please Input The Config File Path!" << std::endl;
		return 0;
	}
	
	readParameters(config_file);

	//1. read the points
	pcl::PointCloud<pcl::PointXYZINormal> refer_cloud, cur_cloud, first_cloud;
	readtxtpcl(refer_file, refer_cloud);
	readtxtpcl(cur_file, cur_cloud);
	readtxtpcl(first_file, first_cloud);
	CLOUD_PTR refer_cloud_ptr(new CLOUD());
	CLOUD_PTR cur_cloud_ptr(new CLOUD());
	CLOUD_PTR first_cloud_ptr(new CLOUD());
	VoxelGridFilter(refer_cloud.makeShared(), refer_cloud_ptr, 0.2);
	VoxelGridFilter(cur_cloud.makeShared(), cur_cloud_ptr, 0.2);
	VoxelGridFilter(first_cloud.makeShared(), first_cloud_ptr, 0.2);
	// kd ËÑË÷
	
	pcl::KdTreeFLANN<PointType> kdTree0;
	CLOUD laserOut, curtmp, outlier;
	curtmp = *cur_cloud_ptr;
	vector<int> idx;
	vector<float> dist;
	kdTree0.setInputCloud(refer_cloud_ptr);
	for (auto i : curtmp)
	{
		kdTree0.radiusSearch(i, 0.20, idx, dist);
		if (idx.size() < 1)
		{
			laserOut.push_back(i);
		}
	}
	if (laserOut.empty())
	{
		cout << "can not find the outlier cloud to the referencloud! " << endl;
	}
	else
	{
		pcl::io::savePCDFileBinary("D:\\code\\19-mine\\refercloud.pcd", laserOut);
		kdTree0.setInputCloud(first_cloud_ptr);
		idx.clear();
		dist.clear();
		for (auto i : laserOut)
		{
			kdTree0.radiusSearch(i, 0.20, idx, dist);
			if (idx.size() < 1)
			{
				outlier.push_back(i);
			}
		}
		if (!outlier.empty())
		{
			pcl::io::savePCDFileBinary("D:\\code\\19-mine\\firstcloud.pcd", outlier);
		}
		else {
			cout << "can not find the outlier cloud to the firstcloud! " << endl;
		}
	}
    return 0;
}