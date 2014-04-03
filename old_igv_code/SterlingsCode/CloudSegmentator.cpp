/*
	@file CloudSegmentator.cpp

	@author Sterling Baldwin
	@date	22 october 2013

	This file contains the implementation of the PlanerCloudSegmentator object
		which takes a cloud and segmentates it in several ways. Current segmentation 
		methods include:
		Planer Segmentation, 

		Methods needing to be implemented:
		Cylindrical Segmentation, ColorRegionGrowing Segmentation
*/
//=============================================================================
// System Includes
//=============================================================================
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>


//=============================================================================
// PCL Includes
//=============================================================================
#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/extract_indices.h>



//=============================================================================
// Project Includes
//=============================================================================
#include "CloudSegmentator.h"
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr CloudPtr;


CloudSegmentator::CloudSegmentator(pcl::PointCloud<pcl::PointXYZRGB>::Ptr newData)
{
	data = new pcl::PointXYZRGB;
	*data = newData;
}

CloudSegmentator::~CloudSegmentator()
{
	delete data;
}


/*
	DistanceThreshold for smooth flat surface is approx 0.01
		The correct Threshold value for the grassy field will change dependent on the length
		of the grass
*/

void CloudSegmentator::extractPlanerSegment(float distanceThreshold, CloudPtr &Plane, CloudPtr &nonPlaner)
{
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  	// Create the segmentation object
  	pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  	// Optional
  	seg.setOptimizeCoefficients (true);
	// Mandatory
  	seg.setModelType (pcl::SACMODEL_PLANE);
  	seg.setMethodType (pcl::SAC_RANSAC);
  	//==========================================================================================
  	// This is what is going to need to be changed for the grass
  	seg.setDistanceThreshold (distanceThreshold);

  	seg.setInputCloud (data.makeShared ()); // data is the private member data for the cloudSegemntator object
  	seg.segment (*inliers, *coefficients);

  	pcl::ExtractIndices<pcl::PointXYZRGB> extrator;
  	extrator.setInputCloud(data);
  	extrator.setIndices(inliers);
  	extrator.setNegative(false);

  	//extract the plane from the cloud
  	extractor.filter(*plane);
  	cout << "Extracted " << plane->points.size() << " points as part of the ground.\n";

  	//extract non-planer elements
  	extract.setNegative(true);
  	extract.filter(*nonPlaner);
  	
}


void CloudSegmentator::extractEuclideanClusters(CloudPtr &inputCloud)
{
	pcl::search::Kdtree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::Kdtree<pcl::PointXYZRGB>);
	tree->setInputCloud(inputCloud);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
	ec.setClusterTolerance(0.02);
	ec.setMinClusterSize(10);
	ec.setMaxClusterSize(25000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(inputCloud);

	ec.extract(cluster_indices);


	int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++){
			cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
		}
  		cloud_cluster->width = cloud_cluster->points.size ();
  		cloud_cluster->height = 1;
  		cloud_cluster->is_dense = true;

  		std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
 		std::stringstream ss;
  		ss << "cloud_cluster_" << j << ".pcd";
   		writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
    	j++;
	}



}








