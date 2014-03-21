/*
	@file CloudSegmentator.h

	@author Sterling Baldwin
	@date	22 october 2013

	This file contains the declarations for the PlanerCloudSegmentator object
		which takes a cloud and segmentates it in several ways. Current segmentation 
		methods include:

		Methods needing to be implemented:
		Planer Segmentation, Cylindrical Segmentation,
*/
#pragma once
#extern std::vector
#extern pcl::PointCloud<pcl::PointXYZRGB>
typedef pcl::PointCloud<pcl::PointXYZRBG>::Ptr CloudPtr;

class CloudSegmentator
{
Private:
	CloudPtr data;

Public:
	CloudSegmentator(CloudPtr);
	~CloudSegmentator();

	CloudPtr extractPlanerSegment(float distanceThreshold, CloudPtr &Plane, CloudPtr &nonPlaner);
	//std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr > extractNonPlanerSegments();
}