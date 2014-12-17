
//System includes
#include <iostream>
#include <vector>
#include <sstream>

//pcl includes
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

//i hate myself for this but its easy
using namespace std;

int
main (int argc, char** argv)
{
  float r_threshold = 1.0;
  float p_threshold = 1.0;
  int neighbors = 50;
  float d_thresh = 0.2;
  int max_cluster = 10000;
  int min_cluster = 1000;
  if(argc == 4)
  {
    std::stringstream ss(argv[2]);
    ss >> min_cluster;
    ss.str("");
    ss << argv[3];
    ss >> max_cluster;
  }
      cout << "Setting min_cluster to "  << min_cluster << "\nMax cluster to " << max_cluster << '\n';

  if(argc == 5)
  {
    std::stringstream ss(argv[1]);
    ss >> p_threshold;
    ss.str("");
    ss << argv[2];
    ss >> neighbors;
    ss.str("");
    ss << argv[3];
    ss >> r_threshold;
    ss.str("");
    ss << argv[4];
    ss >> d_thresh;
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  if ( pcl::io::loadPCDFile <pcl::PointXYZRGB> (argv[1], *cloud) == -1)
  {
    std::cout << "Cloud reading failed." << std::endl;
    return (-1);
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr greycloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  double total_lumin = 0;
  double max = 0;
  double min = 9999;
  int count = 0;
  int num_removed = 0;
  double lumin;
  for(int i = 0; i < cloud->points.size(); i++)
  {
    int r = cloud->points.at(i).r;
    int g = cloud->points.at(i).g;
    int b = cloud->points.at(i).b;
    lumin = 0.00001*r + 0.00001*g + 10.0*b;
 
    total_lumin += lumin;
    count++;
    if(lumin > max) max = lumin;
    else if(lumin < min) min = lumin;
    // std::cout << "-------------------------------\nr = " << r << '\n'
    //   << "g = " << g << '\n'
    //   << "b = " << b << "\n\n";
  }
  double ave = total_lumin/count;
  for(int i = 0; i < cloud->points.size(); i++ )
  {
    int r = cloud->points.at(i).r;
    int g = cloud->points.at(i).g;
    int b = cloud->points.at(i).b;
    lumin = 0.00001*r + 0.00001*g + 10.0*b;
    float z = 0.0;
    if(lumin > (max - max*0.1)) //remove everything but the points within 10% of the max
    {
       pcl::PointXYZRGB point = cloud->points.at(i);
       point.z = z;
       greycloud->points.push_back(point);
       num_removed++;
    }

  }

  std::cout << "Removed: " << count <<"\nMax: " << max << "\nMin: " << min << "\nAve: " << ave << '\n';
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr postRemover (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> remover;
  remover.setInputCloud(greycloud);
  remover.setMeanK(30);
  remover.setStddevMulThresh(0.5);
  remover.filter(*postRemover);

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(postRemover);
  viewer->addPointCloud<pcl::PointXYZRGB> (postRemover, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  //viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals, 10, 0.05, "normals");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();


   //--------------------
  // -----Main loop-----
  //--------------------
  while (!viewer->wasStopped ())
  {
  viewer->spinOnce (100);
  boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
  return 0;



  


  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);


  // // Create the filtering object: downsample the dataset using a leaf size of 1cm
  // pcl::VoxelGrid<pcl::PointXYZRGB> vg;
  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
  // vg.setInputCloud (postRemover);
  // vg.setLeafSize (0.01f, 0.01f, 0.01f);
  // vg.filter (*cloud_filtered);
  // std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*

  // // Create the segmentation object for the planar model and set all the parameters
  
  // pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  // pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ());
  // pcl::PCDWriter writer;
  // seg.setOptimizeCoefficients (true);
  // seg.setModelType (pcl::SACMODEL_PLANE);
  // seg.setMethodType (pcl::SAC_RANSAC);
  // seg.setMaxIterations (100);
  // seg.setDistanceThreshold (0.2);

  // int i=0, nr_points = (int) cloud_filtered->points.size ();
  // while (cloud_filtered->points.size () > 0.3 * nr_points)
  // {
  //   // Segment the largest planar component from the remaining cloud
  //   seg.setInputCloud (cloud_filtered);
  //   seg.segment (*inliers, *coefficients);
  //   if (inliers->indices.size () == 0)
  //   {
  //     std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
  //     break;
  //   }

  //   // Extract the planar inliers from the input cloud
  //   pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  //   extract.setInputCloud (cloud_filtered);
  //   extract.setIndices (inliers);
  //   extract.setNegative (false);

  //   // Get the points associated with the planar surface
  //   extract.filter (*cloud_plane);
  //   std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

  //   // Remove the planar inliers, extract the rest
  //   //extract.setNegative (true);
  //   //extract.filter (*cloud_f);
  //   //*cloud_filtered = *cloud_f;
  // }

  // // Creating the KdTree object for the search method of the extraction
  // pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  // tree->setInputCloud (cloud_filtered);

  // std::vector<pcl::PointIndices> cluster_indices;
  // pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  // ec.setClusterTolerance (0.02); // 2cm
  // ec.setMinClusterSize (100);
  // ec.setMaxClusterSize (25000);
  // ec.setSearchMethod (tree);
  // ec.setInputCloud (cloud_filtered);
  // ec.extract (cluster_indices);

  // int j = 0;
  // for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  // {
  //   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
  //   for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
  //     cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
  //   cloud_cluster->width = cloud_cluster->points.size ();
  //   cloud_cluster->height = 1;
  //   cloud_cluster->is_dense = true;

  //   std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
  //   std::stringstream ss;
  //   ss << "cloud_cluster_" << j << ".pcd";
  //   writer.write<pcl::PointXYZRGB> (ss.str (), *cloud_cluster, false); //*
  //   j++;
  // }

  // return (0);














/*
  pcl::IndicesPtr indices (new std::vector <int>);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (greycloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.0);
  pass.filter (*indices);

*/

  




}
