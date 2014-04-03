//
//  main.cpp
//  IGV_BRAINS
//
//  Created by Sterling A. Baldwin on 12/21/13.
//  Copyright (c) 2013 Sterling A. Baldwin. All rights reserved.
//

#include <iostream>
#include "Bumblebee.h"
//=============================================================================
// PCL Includes
//=============================================================================

#include "/Users/sterling/Downloads/pcl-master/common/include/pcl/pcl_base.h"
#include "/Users/sterling/Downloads/pcl-master/common/include/pcl/point_cloud.h"
#include "/Users/sterling/Downloads/pcl-master/io/include/pcl/io/pcd_io.h"
#include </Users/sterling/Downloads/pcl-master/common/include/pcl/point_types.h"
using namespace std;

int main(int argc, const char * argv[])
{
    pcl::PointCloud<pcl::PointXYZRGB>* cloud;
    Bumblebee bee;
    
    bee >> cloud;
    
    
    ///um ya s o o o o o o o
    return 0;
}

