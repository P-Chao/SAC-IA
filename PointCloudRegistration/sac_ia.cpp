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

SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33>

align(PointCloud<PointXYZ>::Ptr cloud1, PointCloud<PointXYZ>::Ptr cloud2,
PointCloud<FPFHSignature33>::Ptr features1, PointCloud<FPFHSignature33>::Ptr features2,
int max_sacia_iterations, double min_correspondence_dist, double max_correspondence_dist) {

	SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia;
	Eigen::Matrix4f final_transformation;
	sac_ia.setInputCloud(cloud2);
	sac_ia.setSourceFeatures(features2);
	sac_ia.setInputTarget(cloud1);
	sac_ia.setTargetFeatures(features1);
	sac_ia.setMaximumIterations(max_sacia_iterations);
	sac_ia.setMinSampleDistance(min_correspondence_dist);
	sac_ia.setMaxCorrespondenceDistance(max_correspondence_dist);
	PointCloud<PointXYZ> finalcloud;
	sac_ia.align(finalcloud);
	sac_ia.getCorrespondenceRandomness();
	return sac_ia;
}