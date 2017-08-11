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

#include "filters.h"
#include "features.h"
#include "registration.h"
#include "sac_ia.h"
#include "visualization.h"

using namespace std;

const double FILTER_LIMIT = 1000.0;
const int MAX_SACIA_ITERATIONS = 1000; // 2000

const float VOXEL_GRID_SIZE = 3;
const double NORMALS_RADIUS = 20;
const double FEATURES_RADIUS = 50; // 50
const double SAC_MAX_CORRESPONDENCE_DIST = 1000; // 2000
const double SAC_MIN_CORRESPONDENCE_DIST = 3;

int main(int argc, char *argv[]){
	string filename1, filename2;
	if (argc == 3){
		filename1 = argv[1];
		filename2 = argv[2];
	} else{
		filename1 = "pcd/a3.pcd";
		filename2 = "pcd/a4.pcd";
	}

	time_t starttime = time(NULL);
	cout << "Loading clouds...\n";
	cout.flush();

	// open the clouds
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
	
	pcl::io::loadPCDFile(filename1, *cloud1);
	pcl::io::loadPCDFile(filename2, *cloud2);

	// downsample the clouds
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1ds(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2ds(new pcl::PointCloud<pcl::PointXYZ>);

	voxelFilter(cloud1, cloud1ds, VOXEL_GRID_SIZE);
	voxelFilter(cloud2, cloud2ds, VOXEL_GRID_SIZE);

	// compute normals
	pcl::PointCloud<pcl::Normal>::Ptr normals1 = getNormals(cloud1ds, NORMALS_RADIUS);
	pcl::PointCloud<pcl::Normal>::Ptr normals2 = getNormals(cloud2ds, NORMALS_RADIUS);

	// compute local features
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr features1 = getFeatures(cloud1ds, normals1, FEATURES_RADIUS);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr features2 = getFeatures(cloud2ds, normals2, FEATURES_RADIUS);

	// 
	auto sac_ia = align(cloud1ds, cloud2ds, features1, features2, 
		MAX_SACIA_ITERATIONS, SAC_MIN_CORRESPONDENCE_DIST, SAC_MAX_CORRESPONDENCE_DIST);
	
	Eigen::Matrix4f init_transform = sac_ia.getFinalTransformation();
	pcl::transformPointCloud(*cloud2, *cloud2, init_transform);
	pcl::PointCloud<pcl::PointXYZ> final = *cloud1;
	final += *cloud2;

	cout << init_transform << endl;
	cout << "done. Time elapsed: " << time(NULL) - starttime << " seconds\n";
	cout.flush();

	pcl::io::savePCDFile("result.pcd", final);
	viewPair(cloud1ds, cloud2ds, cloud1, cloud2);
	//view(final);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr output = coloredMerge(cloud1, cloud2);
	pcl::io::savePCDFile("output.pcd", *output);

	// copy source data
	pcl::transformPointCloud(*cloud2ds, *cloud2ds, init_transform);

	pcl::PointCloud<pcl::PointXYZ>::Ptr src(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr tgt(new pcl::PointCloud<pcl::PointXYZ>);

	src = cloud1;
	tgt = cloud2;

	// compute normals
	pcl::PointCloud<pcl::PointNormal>::Ptr points_with_normals_src = getPointNormals(src, 30);
	pcl::PointCloud<pcl::PointNormal>::Ptr points_with_normals_tgt = getPointNormals(tgt, 30);

	// ICP + LM (Non Linear ICP)
	pcl::IterativeClosestPointNonLinear<pcl::PointNormal, pcl::PointNormal> reg;
	Eigen::Matrix4f Ti = icpNonLinear(points_with_normals_src, points_with_normals_tgt, 2, 2, 0.1);
	Eigen::Matrix4f Tiv = Ti.inverse();
	cout << Tiv << endl;
	pcl::transformPointCloud(*tgt, *tgt, Tiv);
	viewPair(cloud1, cloud2, src, tgt);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr icpout = coloredMerge(src, tgt);
	pcl::io::savePCDFile("icpout.pcd", *icpout);

	return 0;
}