
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

void test_view(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
void passMap(pcl::PointCloud<pcl::PointXYZRGB>::Ptr incloud);

int main(int argc, char** argv)
{
	if (argv[1]==NULL){
		std::cout << "needs a file name to segment" <<std::endl;
	}
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

	std::cout << "area_seg begin" << std::endl
	<< "x, x2, y, y, z, z2:" << startx<<", " << stopx <<", " << starty <<", " << stopy <<", " << startz <<", " << stopz << std::endl;
	pcl::PointCloud<pcl::PointXYZRGB> outCloud;
	FILE             * pPointFile;

	//std::cout << "inCloud height:" << inCloud->height << std::endl;
	//std::cout << "inCloud width:" << inCloud->width << std::endl;
	int denied_counter = 0;
	int accepted_counter = 0;
	for(int i = 0; i < inCloud->points.size(); i++) {

		if (inCloud->points[i].x >= startx
				&& inCloud->points[i].x <= stopx
				&& inCloud->points[i].y >= starty
				&& inCloud->points[i].y <= stopy 
				&& inCloud->points[i].z >= startz
				&& inCloud->points[i].z <= stopz) 
		{
			//std::cout << "point accepted "<<std::endl;
			outCloud.points.push_back(inCloud->points[i]);
		  accepted_counter++;
		}
		else
		{
		  denied_counter++;
			//std::cout << "point[i].x: " << inCloud->points[i].x << " startx: " << startx << " stopx: " << stopx << std::endl;
			//std::cout << "point[i].y: " << inCloud->points[i].y << " starty: " << starty << " stopy: " << stopy << std::endl;
			//std::cout << "point[i].z: " << inCloud->points[i].z << " startz: " << startz << " stopz: " << stopz << std::endl;
		}
	}
			std::cout << "points accepted: " << accepted_counter
			 << " points denied: " << denied_counter
			 << " ratio: " << (float)accepted_counter/(float)denied_counter*100 << "%" <<std::endl;
	//hacks to get around width*height error 
		outCloud.width = 1;
		outCloud.height = accepted_counter;
	// Save points to disk
	pPointFile = fopen( outname.c_str(), "w+" );

	int cursorLocation  = ftell(pPointFile);
	fseek(pPointFile, 0L, SEEK_END);
	int fileSize =ftell(pPointFile);
	fseek(pPointFile, 0L, cursorLocation);

	if ( pPointFile != NULL &&  fileSize>1) {
		//printf("Opening output file %s\n", outname);
		std::cout << "***saving*** " << outCloud.points.size() << " points" << std::endl;
			pcl::io::savePCDFile (outname.c_str(), outCloud);
		std::cout << "File " << outname << " written sucessfully"<<std::endl;
	}
	else {
		//printf("Error opening output file %s\n", outname);
		std::cout<<"Error opening output file"<< outname << std::endl;
	}
	fclose(pPointFile);
	return outCloud;
}

float * seg_info(pcl::PointCloud<pcl::PointXYZRGB>::Ptr incloud) {
	std::cout<< "Seg Info" << std::endl;
	float highest_y=-100, highest_x=-100, highest_z=-100, lowest_x=100, lowest_y=100, lowest_z=100;
	//float info_array[6];
	float * info_array = new float [6];
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
	//print highest and lowest
	std::cout  << " lowest x:" << lowest_x << " highest x:" << highest_x << std::endl;
	std::cout  << " lowest y:" << lowest_y << " highest y:" << highest_y << std::endl;
	std::cout  << " lowest z:" << lowest_z << " highest z:" << highest_z << std::endl;
	std::cout  << "total points in area: " << incloud->points.size() << std::endl;
	std::cout  << "info end" << std::endl;

	return info_array;
}
void passMap(pcl::PointCloud<pcl::PointXYZRGB>::Ptr incloud) {

	uint8_t r = 255, g = 0, b = 0;
	bool checkered=true;

	float * info_array ;
	//array constants for floor boundries
	int minBound_x = 0; int maxBound_x = 1;
	int minBound_z = 4; int maxBound_z = 5;
	//these values could be anything (so longs as they grab all points)  since it will be reading a pcd that has already been segmented as the floor
	float floor_min = -5; float floor_max = 5;
	float area = 0.7;
	float x_min, x_max, z_min, z_max;

	pcl::PointCloud<pcl::PointXYZRGB> finalCloud;
	pcl::PointCloud<pcl::PointXYZRGB> z_row_cloud;
	pcl::PointCloud<pcl::PointXYZRGB> x_partFrom_z_row;

	//get bounds for next line
			std::cout << "Original Cloud" << std::endl;
	info_array = seg_info(incloud);
	 x_min = info_array[minBound_x];
	 x_max = info_array[maxBound_x];
	 z_min = info_array[minBound_z];
	 z_max = info_array[maxBound_z];

	//std::copy(std::begin(0), std::end(5), std::begin(0));
	/*
		std::cout << "before loop" <<std::endl;
		std::cout << "info_array[minBound_z] " << info_array[minBound_z]  << " max: " << info_array[maxBound_z] <<std::endl;
		std::cout << "info_array address " << &info_array  <<std::endl;
		std::cout << "minBound " << minBound_z  <<std::endl;
		*/

	for (float z=z_min; z<=z_max; z+=area){
		//set outname 
			std::ostringstream ss;
			//strange problem using std::string.to_string(float) : error says string is not a part of std, and/or to_string is defined in bb2...
			ss << "z_row_" << std::setprecision(3)<< z << ".pcd";
			std::string outname = ss.str();
			std::cout << "Z Segment outname: " << outname << std::endl;
		//segment for z row cloud
		info_array = seg_info(incloud);

		//rename confusing variables
		 x_min = info_array[minBound_x]; x_max = info_array[maxBound_x];
		 z_min = info_array[minBound_z]; z_max = info_array[maxBound_z];

		z_row_cloud =  area_seg(x_min, x_max, floor_min, floor_max, z, z+area, incloud, outname);

//open the cloud viewer
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr z_row_pointer(&z_row_cloud);
		test_view(z_row_pointer);
		//get bounds for new seg with x 
			std::cout << "after segment Z row info" << std::endl;
			info_array = seg_info(z_row_pointer);
		 x_min = info_array[minBound_x]; x_max = info_array[maxBound_x];
		//separate z into x segments

		for (float x=x_min; x<=x_max-area; x+=area){
			std::ostringstream ss;
			ss << "x_row_"  << std::setprecision(3) << x<< ".pcd";
			std::cout << "X Segment outname: " << ss.str() << std::endl;

//there is seg fault here:
		x_partFrom_z_row = area_seg(x, x+area, floor_min, floor_max, z, z+area, z_row_pointer, ss.str()) ;

		//open cloud viewer
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr test_pointer(&x_partFrom_z_row);
			test_view(test_pointer);

			std::cout<< "x part from z row size: " << x_partFrom_z_row.size() <<std::endl;
			//eliminate data that is too small for a cloud.
			if(x_partFrom_z_row.size()<100) {
				if(x_partFrom_z_row.size()>800) {
outname					//area has enough points to be safe
						r=0; b=255;
				}
				else { 
					r=255; b=0;
				}
				int32_t rgb = (r << 16) | (g << 8) | b; 
				for(int i =0; i<x_partFrom_z_row.points.size(); i++){
					x_partFrom_z_row.points[i].rgba = *(float *)(&rgb); //colors the point 
				}
				finalCloud += x_partFrom_z_row;
			}
		}
	}
}
void test_view(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
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
}
