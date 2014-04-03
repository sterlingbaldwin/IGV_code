#include <pcl/io/io.h>
#include <iostream>
#include "stdlib.h"
using namespace std;

int PCTConvert(string pct_file_path)
{
	PCDwriter writer;
	ifstream pct_stream(pct_file_path.c_str());
	ofstream pcd_stream;
	pcl::PointCloud<pcl::PointXYZRGB> cloud;
	pcl::PointXYZRGB point;
	stringstream point_atributes;
	string s;


	while(getline(pct_stream, s))
	{	
		point_atributes("");
		point_atributes(s);
		point_atributes >> s;
		point.x = atoi(s);
		point_atributes >> s;
		point.y = atoi(s);
		point_atributes >> s;
		point.z = atoi(s);
		point_atributes >> s;
		point.r = atoi(s);
		point_atributes >> s;
		point.g = atoi(s);
		point_atributes >> s;
		point.b = atoi(s);

		cloud.points.push_back(point);
	}

}