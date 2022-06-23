#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <fstream>

void readtxtpcl(std::string curPath, pcl::PointCloud<pcl::PointXYZINormal> &cloud);