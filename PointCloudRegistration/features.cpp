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

PointCloud<FPFHSignature33>::Ptr getFeatures(PointCloud<PointXYZ>::Ptr cloud, PointCloud<Normal>::Ptr normals, double feature_radius) {

	PointCloud<FPFHSignature33>::Ptr features = PointCloud<FPFHSignature33>::Ptr(new PointCloud<FPFHSignature33>);
	search::KdTree<PointXYZ>::Ptr search_method_ptr = search::KdTree<PointXYZ>::Ptr(new search::KdTree<PointXYZ>);
	FPFHEstimation<PointXYZ, Normal, FPFHSignature33> fpfh_est;
	fpfh_est.setInputCloud(cloud);
	fpfh_est.setInputNormals(normals);
	fpfh_est.setSearchMethod(search_method_ptr);
	fpfh_est.setRadiusSearch(feature_radius);
	fpfh_est.compute(*features);
	return features;
}

PointCloud<Normal>::Ptr getNormals(PointCloud<PointXYZ>::Ptr incloud, double normals_radius) {

	PointCloud<Normal>::Ptr normalsPtr = PointCloud<Normal>::Ptr(new PointCloud<Normal>);
	NormalEstimation<PointXYZ, Normal> norm_est;
	norm_est.setInputCloud(incloud);
	norm_est.setRadiusSearch(normals_radius);
	norm_est.compute(*normalsPtr);
	return normalsPtr;
}

PointCloud<PointNormal>::Ptr getPointNormals(PointCloud<PointXYZ>::Ptr incloud, int k){

	PointCloud<PointNormal>::Ptr points_with_normals = PointCloud<PointNormal>::Ptr(new PointCloud<PointNormal>);
	NormalEstimation<PointXYZ, PointNormal> norm_est;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	norm_est.setSearchMethod(tree);
	norm_est.setKSearch(k);
	norm_est.setInputCloud(incloud);
	norm_est.compute(*points_with_normals);
	pcl::copyPointCloud(*incloud, *points_with_normals);
	return points_with_normals;
}