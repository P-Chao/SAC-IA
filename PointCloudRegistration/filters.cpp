#include <iostream>
#include <time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_types.h>
#include <pcl/features/pfh.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/cloud_viewer.h>
#include <limits>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include "pcl/point_cloud.h" 
#include "pcl/kdtree/kdtree_flann.h" 
#include "pcl/filters/passthrough.h" 
#include "pcl/filters/voxel_grid.h" 
#include "pcl/features/fpfh.h" 

using namespace pcl;
using namespace std;

void voxelFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out, float gridsize){
	VoxelGrid<PointXYZ> vox_grid;
	vox_grid.setLeafSize(gridsize, gridsize, gridsize);
	vox_grid.setInputCloud(cloud_in);
	vox_grid.filter(*cloud_out);
	return;
}

void passFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr pc, double filter_limit){
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(pc);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(0, filter_limit);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(0, filter_limit);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0, filter_limit);
	pass.filter(*pc);
	return;
}


