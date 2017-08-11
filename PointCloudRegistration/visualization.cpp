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
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <limits>
#include <fstream>
#include <vector>
#include <Eigen/Core>

using namespace pcl;
using namespace std;
using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;

void view(pcl::PointCloud<pcl::PointXYZ> &cloud)
{
	pcl::visualization::CloudViewer viewer1("Cloud Viewer");
	viewer1.showCloud(cloud.makeShared());
	while (!viewer1.wasStopped());
	return;
}

void viewPair(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2,
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1al, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2al){

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->initCameraParameters();

	int v1(0), v2(0);
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer->setBackgroundColor(0, 0, 0, v1);
	viewer->addText("Before Alignment", 10, 10, "v1 text", v1);
	PointCloudColorHandlerCustom<PointXYZ> green(cloud1, 0, 255, 0);
	PointCloudColorHandlerCustom<PointXYZ> red(cloud2, 255, 0, 0);
	viewer->addPointCloud(cloud1, green, "v1_target", v1);
	viewer->addPointCloud(cloud2, red, "v1_sourse", v1);

	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer->setBackgroundColor(0, 0, 0, v2);
	viewer->addText("After Alignment", 10, 10, "v2 text", v2);
	PointCloudColorHandlerCustom<PointXYZ> green2(cloud1al, 0, 255, 0);
	PointCloudColorHandlerCustom<PointXYZ> red2(cloud2al, 255, 0, 0);
	viewer->addPointCloud(cloud1al, green2, "v2_target", v2);
	viewer->addPointCloud(cloud2al, red2, "v2_sourse", v2);
	viewer->spin();

	return;
}

PointCloud<pcl::PointXYZRGB>::Ptr coloredMerge(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2){

	PointCloud<pcl::PointXYZRGB>::Ptr cloud1color(new pcl::PointCloud<pcl::PointXYZRGB>);
	PointCloud<pcl::PointXYZRGB>::Ptr cloud2color(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::copyPointCloud(*cloud1, *cloud1color);
	pcl::copyPointCloud(*cloud2, *cloud2color);
	for (size_t i = 0; i < cloud1color->points.size(); ++i){
		cloud1color->points[i].r = 255;
		cloud1color->points[i].g = 0;
		cloud1color->points[i].b = 0;
	}
	for (size_t i = 0; i < cloud2color->points.size(); ++i){
		cloud2color->points[i].r = 0;
		cloud2color->points[i].g = 0;
		cloud2color->points[i].b = 255;
	}
	*cloud1color += *cloud2color;
	return cloud1color;
}

