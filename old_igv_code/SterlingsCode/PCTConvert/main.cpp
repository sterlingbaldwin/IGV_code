// system includes
#include <iostream>
#include <fstream>
#include <stdlib.h>
// pcl includes
#include <pcl/io/pcd_io.h>
#include <pcl/io/impl/pcd_io.hpp>
#include <pcl/point_types.h>

using namespace std;

int main()
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointXYZRGB point;
    string s;
    string filePath;
    fstream inStream;


    cout << "Please give the path to the pct file:";
    cin >> filePath;
    inStream.open(filePath.c_str());
    if(!inStream.is_open()) cout << "Invalid file path.";

    while(getline(inStream, s))
    {
        stringstream lineStream(s);
        lineStream >> s;
        point.x = atoi(s.c_str());
        lineStream >> s;
        point.y = atoi(s.c_str());
        lineStream >> s;
        point.z = atoi(s.c_str());
        lineStream >> s;
        point.r = atoi(s.c_str());
        lineStream >> s;
        point.g = atoi(s.c_str());
        lineStream >> s;
        point.b = atoi(s.c_str());

        cloud->points.push_back(point);
    }

    size_t found = filePath.find('.');
    if(found != string::npos)
    {
        filePath = filePath.substr(0, found);
        filePath = filePath + ".pcd";
    }

    pcl::PCDWriter writer;
    writer.writeBinary<pcl::PointXYZRGB> (filePath, *cloud);
    //pcl::io::savePCDFileASCII(filePath, *cloud);
    return 0;
}
