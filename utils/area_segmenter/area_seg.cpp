// Author Sterling Baldwin and Ryan Beal

#include <iostream>
#include <sstream>
#include <string>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>


pcl::PointCloud<pcl::PointXYZRGB> area_seg(float startx, float stopx, float starty, float stopy, float startz, float stopz, pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud, std::string outname) ;

float * seg_info(pcl::PointCloud<pcl::PointXYZRGB>::Ptr incloud) ;

void passMap(pcl::PointCloud<pcl::PointXYZRGB>::Ptr incloud);

int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>() );
	if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (argv[1], *cloud) == -1)
	{
		PCL_ERROR ("Couldn't read the input file \n");
		return (-1);
	}
	//std::string outname="tiny_cloud.pcd";
	//pcl::PointCloud<pcl::PointXYZRGB> m_cloud(area_seg(-1.2, 0, -1.3, 0 ,1.9, 2.0, cloud, outname));
	//pcl::PointCloud<pcl::PointXYZRGB> m_cloud(area_seg(cloud, outname));

	passMap(cloud);
	

	return 0;
}

pcl::PointCloud<pcl::PointXYZRGB> area_seg(float startx, float stopx, float starty, float stopy, float startz, float stopz, pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud, std::string outname) {

	std::cout << "point 1" <<std::endl;
	pcl::PointCloud<pcl::PointXYZRGB> outCloud;
	FILE             * pPointFile;

	for(int i = 0; i < inCloud->points.size(); i++) {
		if (inCloud->points[i].x >= startx
				&& inCloud->points[i].x <= stopx
				&& inCloud->points[i].y >= starty
				&& inCloud->points[i].y <= stopy 
				&& inCloud->points[i].z >= startz
				&& inCloud->points[i].z <= stopz) 
		{
			std::cout << "File "<<std::endl;
			outCloud.points.push_back(inCloud->points[i]);
			//hacks to get around width*height error
				outCloud.width = 1;
				outCloud.height = outCloud.points.size();

		}
	}
		std::cout << "point height:" << outCloud.height << std::endl;
		std::cout << "point width:" << outCloud.width << std::endl;
    // Save points to disk
    pPointFile = fopen( outname.c_str(), "w+" );
    if ( pPointFile != NULL ) {
			//printf("Opening output file %s\n", outname);
			std::cout << "point 4:" << std::endl;
		//	pcl::io::savePCDFile (outname.c_str(), outCloud);
			//std::cout << "File " << outname << " written sucessfully"<<std::endl;
			std::cout << "File " << outname << "not written sucessfully"<<std::endl;
    }
    else {
			//printf("Error opening output file %s\n", outname);
			std::cout<<"Error opening output file"<< outname << std::endl;
	}

	return outCloud;
}

float * seg_info(pcl::PointCloud<pcl::PointXYZRGB>::Ptr incloud) {
	std::cout<< "Seg Info" << std::endl;
	float highest_y=0, highest_x=0,highest_z=0, lowest_x=100, lowest_y=100, lowest_z=100;
	float * array_pointer;
	float info_array[6];
	//for cloud
	for(int i =0; i<incloud->points.size(); i++){
		//get highest and lowest
		if ( incloud->points[i].x < lowest_x) { 
			lowest_x= incloud->points[i].x; 
			info_array[0]=lowest_x; }
		if ( incloud->points[i].x > highest_x) {
			highest_x= incloud->points[i].x;
			info_array[1]=highest_x; }
		if ( incloud->points[i].y < lowest_y) { 
			lowest_y= incloud->points[i].y;
			info_array[2]=lowest_y; }
		if ( incloud->points[i].y > highest_y) {
			highest_y= incloud->points[i].y; 
			info_array[3]=highest_y; }
		if ( incloud->points[i].z < lowest_z) { 
			lowest_z= incloud->points[i].z;
			info_array[4]=lowest_z; }
		if ( incloud->points[i].z > highest_z) {
			highest_z= incloud->points[i].z;
			info_array[5]=highest_z; 
		}

	}
	array_pointer = info_array;
	//print highest and lowest
	std::cout  << " lowest x:" << lowest_x << " highest x:" << highest_x << std::endl;
	std::cout  << " lowest y:" << lowest_y << " highest y:" << highest_y << std::endl;
	std::cout  << " lowest z:" << lowest_z << " highest z:" << highest_z << std::endl;
	std::cout  << "total points in area: " << incloud->points.size() << std::endl;
	std::cout  << "info end" << std::endl;

	return array_pointer;
}
void passMap(pcl::PointCloud<pcl::PointXYZRGB>::Ptr incloud) {

	uint8_t r = 255, g = 0, b = 0;
	bool checkered=true;

	float * info_array = seg_info(incloud);
	//array constants for floor boundries
	int minBound_x = 0;
	int maxBound_x = 1;
	int minBound_z = 4;
	int maxBound_z = 5;
	int finalSize = 6;
	//height //TODO: change for up to 15 degree arc
	float floor_min = -1;
	float floor_max = 1;

	pcl::PointCloud<pcl::PointXYZRGB> finalCloud;
	pcl::PointCloud<pcl::PointXYZRGB> z_row_cloud;
	pcl::PointCloud<pcl::PointXYZRGB> x_partFrom_z_row;

	//get bounds for next line
	info_array = seg_info(incloud);
	for (float z=info_array[minBound_z]; z=info_array[maxBound_z]; z+=0.35){
		//set outname
			std::ostringstream ss;
			ss << "z_row_" <<std::setprecision(4)<< z;
			std::string outname = ss.str();
			std::cout << "Z Segment outname: " << outname << std::endl;
		//segment for z row cloud
		z_row_cloud =  area_seg(info_array[minBound_x], info_array[maxBound_x], floor_min, floor_max, z, z+.35, incloud, outname) ;
		std::cout<< "second fails" <<std::endl;

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr z_row_pointer(&z_row_cloud);
		//get bounds for new seg with x 
		info_array = seg_info(z_row_pointer);
		//separate z into x segments
		for (float xStep=info_array[minBound_x]; xStep=info_array[maxBound_x]; xStep+=0.35){
			std::ostringstream ss;
			ss << "x_row_"  <<xStep;
			std::cout << "X Segment outname: " << ss.str() << std::endl;

		  x_partFrom_z_row = area_seg(xStep, xStep+0.35, floor_min, floor_max, z, z+0.35, z_row_pointer, outname) ;
			std::cout<<"x part from z row size"<< x_partFrom_z_row.size() <<std::endl;
			if(x_partFrom_z_row.size()>800) {
			//area has enough points to be safe
				r=0; b=255;
			}
			else { 
				r=255; b=0;
			}
			int32_t rgb = (r << 16) | (g << 8) | b; 
			for(int i =0; i<x_partFrom_z_row.points.size(); i++){
				x_partFrom_z_row.points[i].rgba = *(float *)(&rgb); //colors the point 
				finalCloud += x_partFrom_z_row;
			}
		}
	}
}
