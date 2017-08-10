#include <iostream>
#include <time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
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

Eigen::Matrix4f icpNonLinear(PointCloud<PointNormal>::Ptr src, PointCloud<PointNormal>::Ptr tgt){

	pcl::IterativeClosestPointNonLinear<PointNormal, PointNormal> reg;
	reg.setTransformationEpsilon(1e-6);

	reg.setMaxCorrespondenceDistance(0.1);
	//reg.setPointRepresentation();
	reg.setInputCloud(src);
	reg.setInputTarget(tgt);
	reg.setMaximumIterations(2);

	Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), prev;
	PointCloud<PointNormal>::Ptr reg_result = src;

	for (int i = 0; i < 30; ++i){
		src = reg_result;
		reg.setInputCloud(src);
		reg.align(*reg_result);
		Ti = reg.getFinalTransformation() * Ti;
		if (fabs((reg.getLastIncrementalTransformation() - prev).sum()) 
			< reg.getTransformationEpsilon()){
			reg.setMaxCorrespondenceDistance(reg.getMaxCorrespondenceDistance() - 0.001);
		}
		prev = reg.getLastIncrementalTransformation();
	}

	Eigen::Matrix4f targetToSource = Ti.inverse();

	return Ti;
}