#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <sstream>

int
main (int argc, char** argv)
{
  float r_threshold = 1.0;
  float p_threshold = 1.0;
  int neighbors = 50;
  float d_thresh = 0.2;
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

  pcl::search::Search<pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> > (new pcl::search::KdTree<pcl::PointXYZRGB>);
  pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod (tree);
  normal_estimator.setInputCloud (cloud);
  normal_estimator.setKSearch (50);
  normal_estimator.compute (*normals);

/*
  pcl::IndicesPtr indices (new std::vector <int>);
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.0);
  pass.filter (*indices);
*/
  pcl::RegionGrowingRGB<pcl::PointXYZRGB, pcl::Normal> reg;
    std::cout << "Distance threshold: " << reg.getDistanceThreshold() << std::endl
      << "Point Threshold: " << reg.getPointColorThreshold() << "\n"
      << "Region Threshold: " << reg.getRegionColorThreshold() << "\n";

  reg.setMinClusterSize (100);
  reg.setMaxClusterSize (1000000);
  reg.setSearchMethod (tree);
  reg.setNumberOfNeighbours (neighbors);
  reg.setInputCloud (cloud);
  //reg.setIndices (indices);
  reg.setInputNormals (normals);
  reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
  reg.setPointColorThreshold(p_threshold);
  reg.setRegionColorThreshold(r_threshold);
  reg.setCurvatureTestFlag(false);
  reg.setCurvatureThreshold (100.0);
  reg.setDistanceThreshold(100.0);

  std::vector <pcl::PointIndices> clusters;
  reg.extract (clusters);

  /*std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
  std::cout << "First cluster has " << clusters[0].indices.size () << " points." << endl;
  std::cout << "These are the indices of the points of the initial" <<
  std::endl << "cloud that belong to the first cluster:" << std::endl;

  int counter = 0;
  while (counter < clusters[0].indices.size ())
  {
    std::cout << clusters[0].indices[counter] << ", ";
    counter++;
    if (counter % 10 == 0)
      std::cout << std::endl;
  }
  std::cout << std::endl;
  */

  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
  pcl::visualization::CloudViewer viewer ("Cluster viewer");
  viewer.showCloud(colored_cloud);
  while (!viewer.wasStopped ())
  {
  }

  return (0);
}