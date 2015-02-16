// pcdViewer
// Author Sterling Baldwin



#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>



pcl::PointCloud<pcl::PointXYZRGB> area_seg(float startx, float stopx, float starty, float stopy, pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud) {
	pcl::PointCloud<pcl::PointXYZRGB> outCloud;
	for(int i = 0; i < inCloud->points.size(); i++) {
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

	return 0;
}