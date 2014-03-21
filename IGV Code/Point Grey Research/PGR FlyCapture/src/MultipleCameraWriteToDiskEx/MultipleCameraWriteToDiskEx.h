//=============================================================================
// Copyright © 2007 Point Grey Research, Inc. All Rights Reserved.
// 
// This software is the confidential and proprietary information of Point
// Grey Research, Inc. ("Confidential Information").  You shall not
// disclose such Confidential Information and shall use it only in
// accordance with the terms of the license agreement you entered into
// with Point Grey Research, Inc. (PGR).
// 
// PGR MAKES NO REPRESENTATIONS OR WARRANTIES ABOUT THE SUITABILITY OF THE
// SOFTWARE, EITHER EXPRESSED OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
// PURPOSE, OR NON-INFRINGEMENT. PGR SHALL NOT BE LIABLE FOR ANY DAMAGES
// SUFFERED BY LICENSEE AS A RESULT OF USING, MODIFYING OR DISTRIBUTING
// THIS SOFTWARE OR ITS DERIVATIVES.
//=============================================================================
//=============================================================================
// $Id: MultipleCameraWriteToDiskEx.h,v 1.3 2008/03/28 18:49:51 soowei Exp $
//=============================================================================
#if !defined(AFX_MULTIPLECAMERASSPEEDTEST_H__7FCB6F73_4E70_43B9_B434_DA39CEEE9487__INCLUDED_)
#define AFX_MULTIPLECAMERASSPEEDTEST_H__7FCB6F73_4E70_43B9_B434_DA39CEEE9487__INCLUDED_
//=============================================================================
// System Includes
//=============================================================================
//=============================================================================
// PGR Includes
//=============================================================================
//=============================================================================
// Project Includes
//=============================================================================
#include "resource.h"

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

//=============================================================================
// Macro Definitions 
//=============================================================================

//
// Helper code to handle a FlyCapture error.
//
#define _HANDLE_ERROR( error, function ) \
   if( error != FLYCAPTURE_OK ) \
{ \
   printf( "%s: %s\n", function, flycaptureErrorToString( error ) ); \
   if ( g_bAllocated ) deallocateBuffers(); \
   return -1; \
} \

// Video mode to use. If you want to test a particular video mode, change
// this to the mode that is to be tested.
#define _VIDEOMODE	   FLYCAPTURE_VIDEOMODE_ANY

// Frame rate to use. If you want to test a particular frame rate, change
// this to the frame rate that is to be tested.
#define _FRAMERATE	   FLYCAPTURE_FRAMERATE_ANY

// Maximum size of expected (raw) image.
#define _MAX_IMAGE_SIZE	   1600*1200*3

// Images to grab.
#define _IMAGES		   100

// Number of buffers in memory per camera. Increasing this number may
// help performance on systems with slow disk writes.
#define _BUFFERS	   10

// Maximum cameras on the bus. (Maximum devices allowed on a 1394 bus is 64).
#define _MAX_CAMERAS	   64

// Register defines
#define INITIALIZE         0x0000
#define CAMERA_POWER       0x0610
#define FRAME_INFO	   0x12F8

// Directory to save data to
#define _DESTINATION_DIRECTORY	 "c:\\tmp\\"

// Color processing method used for raw bayer tile images
#define _COLOR_PROCESSING_METHOD    FLYCAPTURE_NEAREST_NEIGHBOR

// File format and file extension to use when saving individual images
#define _SAVE_FILE_FORMAT	    FLYCAPTURE_FILEFORMAT_BMP
#define _SAVE_FILE_EXT		    "bmp"

// Uncomment the line below to report information for every image.
// The large amount of text being outputted may affect performance on
// slower machines.
//#define _VERBOSE 

//=============================================================================
// Global Variables 
//=============================================================================

FlyCaptureContext    g_arContext[ _MAX_CAMERAS ];
FlyCaptureImagePlus  g_arImageplus[ _MAX_CAMERAS ];
FlyCaptureInfoEx     g_arInfo[ _MAX_CAMERAS ];
FlyCaptureImage	     g_arImageTemplate[ _MAX_CAMERAS ];

// Number of cameras
unsigned int         g_uiNumCameras = _MAX_CAMERAS;

// Image buffers to use.
unsigned char**	     g_arpBuffers[ _MAX_CAMERAS ];

// Number of images to grab passed in
int		     g_iNumImagesToGrab = _IMAGES;

// Sync success?
bool		     g_bSyncSuccess = false;

// Have the buffers been allocated?
bool		     g_bAllocated = false;

// Buffers used for color-processing (BGR pixel format)
unsigned char	     g_srcBuffer[_MAX_IMAGE_SIZE];
unsigned char	     g_dstBuffer[_MAX_IMAGE_SIZE];

// Bayer tile pattern used by each camera
FlyCaptureStippledFormat g_arBayerTile[_MAX_CAMERAS];


//=============================================================================
// Function prototypes 
//=============================================================================

// Echo usage
void printUsage();

// Echo disk information
void printDiskInfo();

// Begins the test
int startTest();

// Allocates the buffers for the images
void allocateBuffers();

// Deallocates the buffers for the images
void deallocateBuffers();

// Grab and test loop
int doGrabLoop();

// Splits and color-process
int splitImages();

// Helper functions
int createContexts();
int initCameras();
int startCameras();
int stopCameras();
int destroyContexts();



#endif // !defined(AFX_MULTIPLECAMERASSPEEDTEST_H__7FCB6F73_4E70_43B9_B434_DA39CEEE9487__INCLUDED_)
