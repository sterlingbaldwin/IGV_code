//
//  Bumblebee.h
//  IGV_BRAINS
//
//  Created by Sterling A. Baldwin on 12/21/13.
//  Copyright (c) 2013 Sterling A. Baldwin. All rights reserved.
//

#ifndef __IGV_BRAINS__Bumblebee__
#define __IGV_BRAINS__Bumblebee__

#include <iostream>

#include "/Users/sterling/Downloads/pcl-master/common/include/pcl/pcl_base.h"
#include "/Users/sterling/Downloads/pcl-master/common/include/pcl/point_cloud.h"
#include "/Users/sterling/Downloads/pcl-master/io/include/pcl/io/pcd_io.h"
#include </Users/sterling/Downloads/pcl-master/common/include/pcl/point_types.h"

//=============================================================================
// PGR Includes
//*********These have to change when going to the actualy system***************
//=============================================================================
#include "triclops.h"
#include "pgrflycapture.h"
#include "pgrflycapturestereo.h"
#include "pnmutils.h"

class Bumblebee
{
public:
    Bumblebee();
    ~Bumblebee();
    pcl::PointCloud<pcl::PointXYZRGB>* operator>>(){return cloud};
    void initialize_cloud();

private:
    pcl::PointCloud<pcl::PointXYZRGB>* cloud;
    
};
#endif /* defined(__IGV_BRAINS__Bumblebee__) */
