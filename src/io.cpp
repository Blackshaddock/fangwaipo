#include "io.h"
void readtxtpcl(std::string curPath, pcl::PointCloud<pcl::PointXYZINormal> &cloud)
{
	std::ifstream input(curPath.c_str());
	if (!input.good())
		throw "Could not open file: " + curPath;

	cloud.clear();
	std::string line;
	//std::setlocale(LC_NUMERIC, "en_US.UTF-8");
	pcl::PointXYZINormal curPt;
	while (std::getline(input, line))  // ±£´æÊý¾Ý;
	{
		float Tangle, x, y, z, nx, ny, nz, inten;
		sscanf(line.c_str(), "%f, %f, %f, %f", &x, &y, &z, &nx, &ny, &nz, &inten);
		curPt.x = x;
		curPt.y = y;
		curPt.z = z;
		curPt.normal_x = nx;
		curPt.normal_y = ny;
		curPt.normal_z = nz;
		curPt.intensity = inten;
		cloud.push_back(curPt);
	}
}