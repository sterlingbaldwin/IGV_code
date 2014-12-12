/*
	@file bumblebeeToCloud.h

	@author Sterling Baldwin
	@date	21 october 2013

	This file contains the decloration of the bumblebeeToCloud object
		which handles the whole process of going from the camera to
		loading into the pointCloud object full of pointXYZRBG 
*/
#pragma once
#extern pcl::PointCloud<pcl::PointXYZRBG>
typedef pcl::PointCloud<pcl::PointXYZRBG>::Ptr CloudPtr;
class bumblebeeToCloud
{
public:
	BumblebeeToCloud();
	~BumblebeeToCloud();

	CloudPtr getCloud();

private:
	pcl::PointCloud<PointXYZRBG>::Ptr data;

	pcl::PointCloud<PointXYZRBG>::Ptr loadCloud();

/*
	For the time being we open and close the camera each time
		we take a "shot," but in the future this can be refined 
		to speed up the process.

	bool openCamera();
	void closeCamera();
	void refreshData();
*/

}