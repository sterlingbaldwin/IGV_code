// pcdViewer
// Author Sterling Baldwin



#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>



pcl::PointCloud<pcl::PointXYZRGB> area_seg(float startx, float stopx, float starty, float stopy, pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud) {
	pcl::PointCloud<pcl::PointXYZRGB> outCloud;
	for(int i = 0; i < inCloud->points.size(); i++) {
		std::cout << inCloud->points[i].x  << ' ' << inCloud->points[i].y << '\n';
		if (inCloud->points[i].x >= startx 
			&& inCloud->points[i].x <= stopx
			&& inCloud->points[i].y >= starty
			&& inCloud->points[i].y <= stopy) {
			outCloud.points.push_back(inCloud->points[i]);
		}
	}
	return outCloud;
}

int main(int argc, char** argv)
{

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>() );
	if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (argv[1], *cloud) == -1) 
	{
		PCL_ERROR ("Couldn't read file the input file \n");
		return (-1);
	}


	pcl::PointCloud<pcl::PointXYZRGB> m_cloud(area_seg(0.5, 0.6, 0.2, 0.9, cloud));
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr newCloud(&m_cloud);



	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(newCloud);
	viewer->addPointCloud<pcl::PointXYZRGB> (newCloud, rgb, "sample cloud");
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
}