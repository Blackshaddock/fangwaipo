#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv/cv.h>
#include <opencv2/ccalib/omnidir.hpp>
#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>
#include <omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/impl/voxel_grid.hpp>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/segment_differences.h>
using namespace std;
//using namespace cv;
typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> CLOUD;
typedef CLOUD::Ptr CLOUD_PTR;
//pcl::NormalEstimationOMP<Point_T, Point_T> ne;

extern int ROW;
extern int COL;
extern int FOCAL_LENGTH;
extern int IMAGE_COUNT;
extern cv::Size BOARD_SIZE;
extern float SQUARE_SIZE;
extern std::ofstream FOUT;

extern std::string refer_file;
extern std::string cur_file;
extern std::string first_file;
extern std::string CORNER_FILEPATH;
extern std::string CORRECT_FILEPATH;
extern std::string PROCESS_TYPE;
extern std::string SAVE_FILEPATH;
extern std::string VIDEO_FILEPATH;
void readParameters(std::string &config_file);
