#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
using namespace std;

int main()
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	
	if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("test_pcd.pcd", *cloud) == -1) //* load the file
	{
		PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
		return (-1);
	}
	std::cout << "Loaded "
	        << cloud->width * cloud->height
	        << " data points from test_pcd.pcd" << endl;

	pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
	viewer.showCloud (cloud);
	while (!viewer.wasStopped ())
	{
	}
}