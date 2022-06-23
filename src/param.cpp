#include "param.h"
#include <direct.h>
std::string refer_file;
std::string cur_file;
std::string first_file;
void readParameters(std::string &config_file)
{
	cout << config_file << endl;
	
	cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);

	//ÊäÈëÍ¼Æ¬ºÍÍ¼Æ¬±£´æ
	fsSettings["refer_file"] >> refer_file;
	fsSettings["cur_file"] >> cur_file;
	fsSettings["first_file"] >> first_file;
	
	fsSettings.release();
	//sleep(1);
}

