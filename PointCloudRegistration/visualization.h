#pragma once

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

void view(pcl::PointCloud<pcl::PointXYZ> &cloud);
void viewPair(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2,
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1al, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2al);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredMerge(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2);