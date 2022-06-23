#pragma once
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <liblas/liblas.hpp>
#include <pcl/common/common.h>
#include <iostream>
#include <fstream>
#include <param.h>
#include <pcl/filters/voxel_grid.h>





bool WriteLasResult(std::string path,pcl::PointCloud<pcl::PointXYZINormal>& cloud,Eigen::Vector3d offset)
{
	liblas::Header curheader_;
	std::ofstream outPt_;
	
//	if(cloud.size()==0)
//		return false;
		
	curheader_.SetVersionMajor(1);
	curheader_.SetVersionMinor(2);
	curheader_.SetDataFormatId(liblas::PointFormatName::ePointFormat3);
	curheader_.SetScale(0.001,0.001,0.001);
	curheader_.SetOffset(offset[0],offset[1],offset[2]);
	
	curheader_.SetCompressed(false);
	curheader_.SetSystemId("GreenValley");
	curheader_.SetSoftwareId("GreenValley");
	
	time_t systemtime;
	time(&systemtime);
	
	tm* p=localtime(&systemtime);
	curheader_.SetCreationYear(p->tm_year+1900);
	curheader_.SetCreationDOY(p->tm_yday+1);
	
	outPt_.open(path.c_str(),std::ios::out | std::ios::binary);
	if(!outPt_.is_open())
		return false;
		
	Eigen::Vector4f min_pt,max_pt;
	pcl::getMinMax3D(cloud,min_pt,max_pt);
	
	
	
	curheader_.SetMax(max_pt.x()+offset[0],max_pt.y()+offset[1],max_pt.z()+offset[2]);
	curheader_.SetMin(min_pt.x()+offset[0],min_pt.y()+offset[1],min_pt.z()+offset[2]);
	
	
	liblas::Writer writer_(outPt_,curheader_);
	liblas::Point wrtpoint(&curheader_);
	liblas::Color rgb;
	std::cout<<"write las:"<<std::endl;
	for(int i=0;i<cloud.size();i++)
	{
		wrtpoint.SetCoordinates(cloud.points[i].x+offset[0],cloud.points[i].y+offset[1],cloud.points[i].z+offset[2]);
		
		wrtpoint.SetIntensity(cloud.points[i].intensity);
		
		rgb.SetRed(cloud.points[i].normal_x);
		rgb.SetGreen(cloud.points[i].normal_y);
		rgb.SetBlue(cloud.points[i].normal_z);
		if(i<10)
			std::cout<<cloud.points[i].normal_x<<" "<<cloud.points[i].normal_y<<" "<<cloud.points[i].normal_z<<std::endl;
		wrtpoint.SetColor(rgb);
		writer_.WritePoint(wrtpoint);
	}
	

	
	
	
	return true;
}


void VoxelGridFilter(CLOUD_PTR &cloudIn, CLOUD_PTR &cloudOut, float gridSize)
{
	float inverse_leaf_size = 1.0 / gridSize;
	// Copy the header (and thus the frame_id) + allocate enough space for points
	cloudOut->height = 1;                    // downsampling breaks the organized structure
	cloudOut->is_dense = true;                 // we filter out invalid points

	Eigen::Vector4f min_p, max_p;
	boost::shared_ptr <std::vector<int> > indices_;

	indices_.reset(new std::vector<int>);
	try
	{
		indices_->resize(cloudIn->points.size());
	}
	catch (const std::bad_alloc&)
	{
		cout << "[initCompute] Failed to allocate %lu indices.\n", cloudIn->points.size();
	}
	for (size_t i = 0; i < indices_->size(); ++i) { (*indices_)[i] = static_cast<int>(i); }


	pcl::getMinMax3D<PointType>(*cloudIn, *indices_, min_p, max_p);

	// Check that the leaf size is not too small, given the size of the data
	int64_t dx = static_cast<int64_t>((max_p[0] - min_p[0]) * inverse_leaf_size) + 1;
	int64_t dy = static_cast<int64_t>((max_p[1] - min_p[1]) * inverse_leaf_size) + 1;
	int64_t dz = static_cast<int64_t>((max_p[2] - min_p[2]) * inverse_leaf_size) + 1;

	if ((dx*dy*dz) > static_cast<int64_t>(std::numeric_limits<int64_t>::max()))
	{
		cout << " Leaf size is too small for the input dataset. Integer indices would overflow.\n";
		cloudOut = cloudIn;
		return;
	}

	// Compute the minimum and maximum bounding box values
	Eigen::Vector4i min_b_, max_b_, div_b_, divb_mul_;
	min_b_[0] = static_cast<int> (floor(min_p[0] * inverse_leaf_size));
	max_b_[0] = static_cast<int> (floor(max_p[0] * inverse_leaf_size));
	min_b_[1] = static_cast<int> (floor(min_p[1] * inverse_leaf_size));
	max_b_[1] = static_cast<int> (floor(max_p[1] * inverse_leaf_size));
	min_b_[2] = static_cast<int> (floor(min_p[2] * inverse_leaf_size));
	max_b_[2] = static_cast<int> (floor(max_p[2] * inverse_leaf_size));

	// Compute the number of divisions needed along all axis
	div_b_ = max_b_ - min_b_ + Eigen::Vector4i::Ones();
	div_b_[3] = 0;

	// Set up the division multiplier
	divb_mul_ = Eigen::Vector4i(1, div_b_[0], div_b_[0] * div_b_[1], 0);

	// Storage for mapping leaf and pointcloud indexes
	std::vector<cloud_point_index_idx> index_vector;
	index_vector.reserve(indices_->size());


	// First pass: go over all points and insert them into the index_vector vector
	// with calculated idx. Points with the same idx value will contribute to the
	// same point of resulting CloudPoint
	for (std::vector<int>::const_iterator it = indices_->begin(); it != indices_->end(); ++it)
	{
		if (!cloudIn->is_dense)
			// Check if the point is invalid
			if (!pcl_isfinite(cloudIn->points[*it].x) ||
				!pcl_isfinite(cloudIn->points[*it].y) ||
				!pcl_isfinite(cloudIn->points[*it].z))
				continue;

		int ijk0 = static_cast<int> (floor(cloudIn->points[*it].x * inverse_leaf_size) - static_cast<float> (min_b_[0]));
		int ijk1 = static_cast<int> (floor(cloudIn->points[*it].y * inverse_leaf_size) - static_cast<float> (min_b_[1]));
		int ijk2 = static_cast<int> (floor(cloudIn->points[*it].z * inverse_leaf_size) - static_cast<float> (min_b_[2]));

		// Compute the centroid leaf index
		int idx = ijk0 * divb_mul_[0] + ijk1 * divb_mul_[1] + ijk2 * divb_mul_[2];
		index_vector.push_back(cloud_point_index_idx(static_cast<unsigned int> (idx), *it));
	}


	// Second pass: sort the index_vector vector using value representing target cell as index
	// in effect all points belonging to the same output cell will be next to each other
	std::sort(index_vector.begin(), index_vector.end(), std::less<cloud_point_index_idx>());

	// Third pass: count output cells
	// we need to skip all the same, adjacenent idx values
	unsigned int total = 0;
	unsigned int index = 0;
	// first_and_last_indices_vector[i] represents the index in index_vector of the first point in
	// index_vector belonging to the voxel which corresponds to the i-th output point,
	// and of the first point not belonging to.
	std::vector<std::pair<unsigned int, unsigned int> > first_and_last_indices_vector;
	// Worst case size
	first_and_last_indices_vector.reserve(index_vector.size());
	while (index < index_vector.size())
	{
		unsigned int i = index + 1;
		while (i < index_vector.size() && index_vector[i].idx == index_vector[index].idx)
			++i;
		if (i - index >= 0)
		{
			++total;
			first_and_last_indices_vector.push_back(std::pair<unsigned int, unsigned int>(index, i));
		}
		index = i;
	}

	// Fourth pass: compute centroids, insert them into their final position
	cloudOut->points.resize(total);

	index = 0;
	for (unsigned int cp = 0; cp < first_and_last_indices_vector.size(); ++cp)
	{
		// calculate centroid - sum values from all input points, that have the same idx value in index_vector array
		unsigned int first_index = first_and_last_indices_vector[cp].first;
		unsigned int last_index = first_and_last_indices_vector[cp].second;

		//Limit downsampling to coords

		pcl::CentroidPoint<PointType> centroid;

		// fill in the accumulator with leaf points
		for (unsigned int li = first_index; li < last_index; ++li)
			centroid.add(cloudIn->points[index_vector[li].cloud_point_index]);

		centroid.get(cloudOut->points[index]);

		cloudOut->points[index].normal_x = cloudIn->points[index_vector[first_index].cloud_point_index].normal_x;
		cloudOut->points[index].normal_y = cloudIn->points[index_vector[first_index].cloud_point_index].normal_y;
		cloudOut->points[index].normal_z = cloudIn->points[index_vector[first_index].cloud_point_index].normal_z;

		++index;
	}
	cloudOut->width = static_cast<uint32_t> (cloudOut->points.size());
}

