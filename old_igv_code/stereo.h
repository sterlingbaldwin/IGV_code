#pragma once
#ifndef STEREO_H
#define STEREO_H

#include "globals.h"

// This file is a modified version of:
// F:\Program Files\Point Grey Research\Triclops Stereo Vision SDK\src\examples\win32\stereoto3dpoints\stereoto3dpoints.cpp

//=============================================================================
// Copyright © 2004 Point Grey Research, Inc. All Rights Reserved.
//
// This software is the confidential and proprietary information of Point
// Grey Research, Inc. ("Confidential Information").  You shall not
// disclose such Confidential Information and shall use it only in
// accordance with the terms of the license agreement you entered into
// with Point Grey Research, Inc. (PGR).
//
// PGR MAKES NO REPRESENTATIONS OR WARRANTIES ABOUT THE SUITABILITY OF THE
// SOFTWARE, EITHER EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
// PURPOSE, OR NON-INFRINGEMENT. PGR SHALL NOT BE LIABLE FOR ANY DAMAGES
// SUFFERED BY LICENSEE AS A RESULT OF USING, MODIFYING OR DISTRIBUTING
// THIS SOFTWARE OR ITS DERIVATIVES.
//=============================================================================
//=============================================================================
// $Id: stereoto3dpoints.cpp,v 1.9 2007/05/28 22:30:26 soowei Exp $
//=============================================================================
//=============================================================================
// stereoto3dpoints
//
// Takes input from a Bumblebee and performs subpixel
// interpolation to create a 16-bit disparity image, which is saved.
// The disparity data is then converted to 3-dimensional X/Y/Z
// coordinates which is written to a file.
//
// This point file can be viewed with PGRView under windows.
//
//=============================================================================

//=============================================================================
// System Includes
//=============================================================================
//#include <stdio.h>
//#include <stdlib.h>

//=============================================================================
// PGR Includes
//=============================================================================

#include "triclops.h"
#include "pgrflycapture.h"
#include "pgrflycapturestereo.h"
#include "pnmutils.h"

//=============================================================================
// Project Includes
//=============================================================================

//
// Macro to check, report on, and handle Triclops API error codes.
//
#define _HANDLE_TRICLOPS_ERROR( function, error ) \
{ \
   if( error != TriclopsErrorOk ) \
   { \
      printf( \
	 "ERROR: %s reported %s.\n", \
	 function, \
	 triclopsErrorToString( error ) ); \
      exit( 1 ); \
   } \
} \

//
// Macro to check, report on, and handle Flycapture API error codes.
//
#define _HANDLE_FLYCAPTURE_ERROR( function, error ) \
{ \
   if( error != FLYCAPTURE_OK ) \
   { \
      printf( \
	 "ERROR: %s reported %s.\n", \
	 function, \
	 flycaptureErrorToString( error ) ); \
      exit( 1 ); \
   } \
} \


void RunStereo()
{
	printf("hStereo successfully started.\n");
	TriclopsInput		stereoData;
	TriclopsInput		colorData;
	TriclopsImage16		depthImage16;
	TriclopsImage		monoImage;
	TriclopsColorImage	colorImage;
	TriclopsContext		triclops;
	TriclopsROI*		pRois;
	int					nMaxRois;
	TriclopsError		te;

	float				d, x, y, z;
	int					pixelinc ;
	int					i, j, k;
	unsigned short*		row;
	unsigned short		disparity;

	FlyCaptureContext		flycapture;
	FlyCaptureImage			flycaptureImage;
	FlyCaptureInfoEx		pInfo;
	FlyCapturePixelFormat	pixelFormat;
	FlyCaptureError			fe;

	int iMaxCols = 0;
	int iMaxRows = 0;

	char* szCalFile;


	// Open the camera
	fe = flycaptureCreateContext( &flycapture );
	_HANDLE_FLYCAPTURE_ERROR( "flycaptureCreateContext()", fe );

	// Initialize the Flycapture context
	fe = flycaptureInitialize( flycapture, 0 );
	_HANDLE_FLYCAPTURE_ERROR( "flycaptureInitialize()", fe );

	// Save the camera's calibration file, and return the path
	fe = flycaptureGetCalibrationFileFromCamera( flycapture, &szCalFile );
	_HANDLE_FLYCAPTURE_ERROR( "flycaptureGetCalibrationFileFromCamera()", fe );

	// Create a Triclops context from the cameras calibration file
	te = triclopsGetDefaultContextFromFile( &triclops, szCalFile );
	_HANDLE_TRICLOPS_ERROR( "triclopsGetDefaultContextFromFile()", te );



	// Get camera information
	fe = flycaptureGetCameraInfo( flycapture, &pInfo );
	_HANDLE_FLYCAPTURE_ERROR( "flycatpureGetCameraInfo()", fe );

	if (pInfo.CameraType == FLYCAPTURE_COLOR)
	{
	  pixelFormat = FLYCAPTURE_RAW16;
	}
	else
	{
	  pixelFormat = FLYCAPTURE_MONO16;
	}

	switch (pInfo.CameraModel)
	{
	case FLYCAPTURE_BUMBLEBEE2:
	  {
	 unsigned long ulValue;
	 flycaptureGetCameraRegister( flycapture, 0x1F28, &ulValue );

	 if ( ( ulValue & 0x2 ) == 0 )
	 {
		// Hi-res BB2
		iMaxCols = 1024;
		iMaxRows = 768;
	 }
	 else
	 {
		// Low-res BB2
		iMaxCols = 640;
		iMaxRows = 480;
	 }
	  }
	  break;

	case FLYCAPTURE_BUMBLEBEEXB3:
	  iMaxCols = 1280;
	  iMaxRows = 960;
	  break;

	default:
	  te = TriclopsErrorInvalidCamera;
	  _HANDLE_TRICLOPS_ERROR( "triclopsCheckCameraModel()", te );
	  break;
	}


	// Start transferring images from the camera to the computer
	fe = flycaptureStartCustomImage(
	  flycapture, 3, 0, 0, iMaxCols, iMaxRows, 100, pixelFormat);
	_HANDLE_FLYCAPTURE_ERROR( "flycaptureStart()", fe );

	// Set up some stereo parameters:
	// Set to 320x240 output images
	te = triclopsSetResolution( triclops, 240, 320 );
	_HANDLE_TRICLOPS_ERROR( "triclopsSetResolution()", te );

	// Set disparity range
	te = triclopsSetDisparity( triclops, 0, 50);
	_HANDLE_TRICLOPS_ERROR( "triclopsSetDisparity()", te );
	  te = triclopsSetEdgeCorrelation( triclops, 1);
	_HANDLE_TRICLOPS_ERROR( "triclopsSetDisparity()", te );
	te = triclopsSetEdgeMask( triclops, 7);
	_HANDLE_TRICLOPS_ERROR( "triclopsSetDisparity()", te );

	// Set surface validation
	te = triclopsSetSurfaceValidation( triclops, 1 );
	_HANDLE_TRICLOPS_ERROR( "triclopsSetSurfaceValidation()", te );
	te = triclopsSetSurfaceValidationSize( triclops, 100 );
	_HANDLE_TRICLOPS_ERROR( "triclopsSetSurfaceValidationSize()", te );
	te = triclopsSetSurfaceValidationDifference( triclops, 3.0 );
	_HANDLE_TRICLOPS_ERROR( "triclopsSetSurfaceValidationDifference()",	te );

	// Set Backforth validation
	te = triclopsSetBackForthValidation( triclops, 1);
	_HANDLE_TRICLOPS_ERROR( "triclopsBackForthValidation()",	te );

	// Lets turn off all validation except subpixel and surface
	// This works quite well
	te = triclopsSetTextureValidation( triclops, 0 );
	_HANDLE_TRICLOPS_ERROR( "triclopsSetTextureValidation()", te );
	te = triclopsSetUniquenessValidation( triclops, 0 );
	_HANDLE_TRICLOPS_ERROR( "triclopsSetUniquenessValidation()", te );

	// Turn on sub-pixel interpolation
	te = triclopsSetSubpixelInterpolation( triclops, 1);
	_HANDLE_TRICLOPS_ERROR( "triclopsSetSubpixelInterpolation()", te );

/* ADDING THIS CODE CAUSES MIDDLE REGION OF IMAGE TO DISAPPEAR
	   // Get the pointer to the regions of interest array
   te = triclopsGetROIs( triclops, &pRois, &nMaxRois );
   _HANDLE_TRICLOPS_ERROR( "triclopsGetROIs()", te );

	pRois[0].row   = 0;
	pRois[0].col   = 0;
	pRois[0].nrows = 240;
	pRois[0].ncols = 320;

	// Tell the TriclopsContext how many ROIs we want to process
	te = triclopsSetNumberOfROIs( triclops, 1 );
	_HANDLE_TRICLOPS_ERROR( "triclopsSetNumberOfROIs()", te );
*/
	// Grab an image from the camera
	fe = flycaptureGrabImage2( flycapture, &flycaptureImage );
	_HANDLE_FLYCAPTURE_ERROR( "flycaptureGrabImage()", fe );

	// Extract information from the FlycaptureImage
	int imageCols = flycaptureImage.iCols;
	int imageRows = flycaptureImage.iRows;
	int imageRowInc = flycaptureImage.iRowInc;
	int iSideBySideImages = flycaptureImage.iNumImages;
	unsigned long timeStampSeconds = flycaptureImage.timeStamp.ulSeconds;
	unsigned long timeStampMicroSeconds = flycaptureImage.timeStamp.ulMicroSeconds;
	printf("sidebysideimages: %d\n", iSideBySideImages);

	// Create buffers for holding the color and mono images
	unsigned char* rowIntColor =
	new unsigned char[ imageCols * imageRows * iSideBySideImages * 4];
	unsigned char* rowIntMono =
	new unsigned char[ imageCols * imageRows * iSideBySideImages ];

	// Pointers to positions in the color buffer that correspond to the beginning
	// of the red, green and blue sections
	unsigned char* redColor = NULL;
	unsigned char* greenColor = NULL;
	unsigned char* blueColor = NULL;

	// Pointers to positions in the mono buffer that correspond to the beginning
	// of the red, green and blue sections
	unsigned char* redMono = NULL;
	unsigned char* greenMono = NULL;
	unsigned char* blueMono = NULL;

	while (!GetAsyncKeyState(VK_ESCAPE))
	{

		// Grab an image from the camera
		fe = flycaptureGrabImage2( flycapture, &flycaptureImage );
		_HANDLE_FLYCAPTURE_ERROR( "flycaptureGrabImage()", fe );

		/*
		// Extract information from the FlycaptureImage
		imageCols = flycaptureImage.iCols;
		imageRows = flycaptureImage.iRows;
		imageRowInc = flycaptureImage.iRowInc;
		iSideBySideImages = flycaptureImage.iNumImages;
		timeStampSeconds = flycaptureImage.timeStamp.ulSeconds;
		timeStampMicroSeconds = flycaptureImage.timeStamp.ulMicroSeconds;
		*/

		// Create a temporary FlyCaptureImage for preparing the stereo image
		FlyCaptureImage tempColorImage;
		FlyCaptureImage tempMonoImage;

		tempColorImage.pData = rowIntColor;
		tempMonoImage.pData = rowIntMono;

		// Convert the pixel interleaved raw data to row interleaved format
		fe = flycapturePrepareStereoImage( flycapture, flycaptureImage, &tempMonoImage, &tempColorImage  );
		_HANDLE_FLYCAPTURE_ERROR( "flycapturePrepareStereoImage()", fe );

		// Pointers to positions in the color buffer that correspond to the beginning
		// of the red, green and blue sections

		redColor = rowIntColor/*+4*imageCols*/;
		if (flycaptureImage.iNumImages == 2)
		{
		greenColor = redColor + ( 4 * imageCols );
		blueColor = redColor + ( 4 * imageCols );
		}

		if (flycaptureImage.iNumImages == 3)
		{
		greenColor = redColor + ( 4 * imageCols );
		blueColor = redColor + ( 2 * 4 * imageCols );
		}


		// Pointers to positions in the mono buffer that correspond to the beginning
		// of the red, green and blue sections
		redMono = rowIntMono;
		if (flycaptureImage.iNumImages == 2)
		{
		greenMono = redMono + imageCols;
		blueMono = redMono + imageCols;
		}

		if (flycaptureImage.iNumImages == 3)
		{
		greenMono = redMono + imageCols;
		blueMono = redMono + ( 2 * imageCols );
		}

		// Use the row interleaved images to build up a packed TriclopsInput.
		// A packed triclops input will contain a single image with 32 bpp.
		te = triclopsBuildPackedTriclopsInput(
		imageCols,
		imageRows,
		imageRowInc * 4,
		timeStampSeconds,
		timeStampMicroSeconds,
		redColor,
		&colorData );
		_HANDLE_TRICLOPS_ERROR( "triclopsBuildPackedTriclopsInput()", te );

		// Use the row interleaved images to build up an RGB TriclopsInput.
		// An RGB triclops input will contain the 3 raw images (1 from each camera).
		te = triclopsBuildRGBTriclopsInput(
		imageCols,
		imageRows,
		imageRowInc,
		timeStampSeconds,
		timeStampMicroSeconds,
		redMono,
		greenMono,
		blueMono,
		&stereoData);
		_HANDLE_TRICLOPS_ERROR( "triclopsBuildRGBTriclopsInput()", te );


		// Preprocessing the images
		te = triclopsRectify( triclops, &stereoData );
		_HANDLE_TRICLOPS_ERROR( "triclopsRectify()", te );

		// Stereo processing
		te = triclopsStereo( triclops ) ;
		_HANDLE_TRICLOPS_ERROR( "triclopsStereo()", te );

		// Retrieve the interpolated depth image from the context
		te = triclopsGetImage16( triclops,
				TriImg16_DISPARITY,
				TriCam_REFERENCE,
				&depthImage16 );
		_HANDLE_TRICLOPS_ERROR( "triclopsGetImage16()", te );

		// Rectify the color image if applicable
		if ( pixelFormat == FLYCAPTURE_RAW16 )
		{
			te = triclopsRectifyColorImage( triclops,
			TriCam_REFERENCE,
			&colorData,
			&colorImage );
			_HANDLE_TRICLOPS_ERROR( "triclopsRectifyColorImage()", te );
		}
		else
		{
			te = triclopsGetImage( triclops,
			TriImg_RECTIFIED,
			TriCam_REFERENCE,
			&monoImage );
			_HANDLE_TRICLOPS_ERROR( "triclopsGetImage()", te );
		}
		// ...
		// Determine the number of pixels spacing per row
		//  pixelinc = depthImage16.rowinc/2;
		// for ( i = 0, k = 0; i < depthImage16.nrows; i++ )
		//  {
		//     row     = depthImage16.data + i * pixelinc;
		//     for ( j = 0; j < depthImage16.ncols; j++, k++ )
		//printf("%d\n", depthImage16.ncols);
		pixelinc = depthImage16.rowinc/2;
		//printf("i: %d\n", i);
		//FILE * pFile;
		//pFile = fopen("output.csv","w");
		//fprintf(pFile, "i,j,disp,x,y,z\n");
		//printf("i\tj\tdisp\tx\ty\tz\n");
		for (i = 0, k = 0; i < depthImage16.nrows; i++)
		{
			//printf("-------------------\n");
			row= depthImage16.data + i * pixelinc;
			for (j = 0; j < depthImage16.ncols; j++, k++)
			{
				disparity = row[j];
				temp1RGBimage[i][j].red = colorImage.red[k];
				temp1RGBimage[i][j].green = colorImage.green[k];
				temp1RGBimage[i][j].blue = colorImage.blue[k];
				//printf("%d\t%d\t%d\n", temp1RGBimage[j][i].red, temp1RGBimage[j][i].green, temp1RGBimage[j][i].blue);
				// do not save invalid points


//				if ((i > 60) && (i < 180) && (j == 0))
//					printf("disparity: %d\n", disparity);

				if (( disparity < 0xFF00 ) && (disparity > 0))
				{
					// convert the 16 bit disparity value to floating point x,y,z
					//triclopsRCD16ToWorldXYZ( triclops, i, j, disparity, &x, &y, &z );
					triclopsRCD16ToWorldXYZ( triclops, i, j, disparity, &x, &y, &z );
					//Sample Point
					//if ((i == 128) && (j == 157))
					//	printf("x: %f\ty: %f\t z: %d\n", x, y, z);
					//printf("i: %d\tj: %d\n", i, j);
					//printf("ptgrey: x: %f\ty: %f\tz: %f\n", x, y, z);
					//fprintf(pFile, "%d,%d,%d,%f,%f,%f\n", i, j, disparity, x, y, z);
					//printf("%d\t%d\t%d\t%f\t%f\t%f\n", i, j, disparity, x, y, z);

					// This line converts disparity to centimeters.
					// This is some arbitary value specified by Point Grey.
					d = 8185.0/disparity;

					//float tempy = y;
					//printf("%d, %f\n", disparity, d);
					//if (((i == 318) && (j == 238)) || ((i == 100) && (j == 100)))
					//if ((i == 239) && (j == 319))
					//printf("ptgrey: thetax: %f\tthetay: %f\n", atan(x/z), atan(y/z));
				//xz = d*cos(xarray[j]);
				//y = -xz*sin(yarray[i]+CAMANG*TORAD);
				//x = d*cos(yarray[i]+CAMANG*TORAD)*sin(xarray[j]);
				//z = xz*cos(yarray[i]+CAMANG*TORAD);
				//y = sqrt(d*d - xz*xz);
					//printf("deltay: %f\n", tempy-y);
					//mdelay(100);
					//printf("error: %f\n", xz-sqrt(x*x+z*z));
					//printf("brian: x: %f\ty: %f\tz: %f\n", x, y, z);
					//printf("brian: thetax: %f\tthetay: %f\n", atan(x/z), atan(y/z));
					//if ((i == 0) && (j == 0))
					//printf("brian: thetax: %f\tthetay: %f\n\n", xarray[j], yarray[i]);

					// This code translates the coordinate system relative to the titled camera,
					// x, y, and z value are relative to the vehicle.

					temp1XYZimage[i][j].x = x;
					//temp1XYZimage[i][j].y = -xz*sin(yarray[i]+CAMANG*TORAD);
					temp1XYZimage[i][j].y = (y*cos(CAMANG*TORAD)-z*sin(CAMANG*TORAD));
					temp1XYZimage[i][j].z = z*cos(CAMANG*TORAD)+y*sin(CAMANG*TORAD);
					temp1XYZimage[i][j].xz = sqrt(sqr(x)+sqr(temp1XYZimage[i][j].z));

//					printf("i: %d, j%d, x: %f\tz: %f\txz: %f\n", i, j, temp1XYZimage[i][j].x, temp1XYZimage[i][j].z, temp1XYZimage[i][j].xz);

					/*
					temp1XYZimage[j][i].x = x;
					temp1XYZimage[j][i].y = y;
					temp1XYZimage[j][i].z = z;
					*/
					/*
					n=(int)(-temp1XYZimage[j][i].x*50)+50;
					m=(int)(temp1XYZimage[j][i].z*50)+25;
					if((0<m && m<500) && (0<n && n<100)&&(temp1XYZimage[j][i].y>-.25))
					{
						bitmap[m][n]=temp1XYZimage[j][i].y;
					}
					*/
				}

				else
				{
				    //if there is no reported data, set to -1.0 (invalid)
					temp1XYZimage[i][j].x = -1.0;
					temp1XYZimage[i][j].y = -1.0;
					temp1XYZimage[i][j].z = -1.0;
				}

			}
		}
		//printf("Entering Critical Section.\n");
		EnterCriticalSection(&RawDisparityMap);
		memcpy(RGBimage, temp1RGBimage, 320*240*sizeof(rgb));
		memcpy(XYZimage, temp1XYZimage, 320*240*sizeof(xyz));
		LeaveCriticalSection(&RawDisparityMap);
		SetEvent(NewImage);
		memset(temp1XYZimage, 0, sizeof(temp1XYZimage));
		//printf("Outside Critical Section.\n");

		//fclose(pFile);
		//printf("Done.\n");

	}

	// Close the camera
	fe = flycaptureStop( flycapture );
	_HANDLE_FLYCAPTURE_ERROR( "flycaptureStop()", fe );

	// Delete the image buffer, it is not needed once the TriclopsInput
	// has been built
	delete rowIntColor;
	redColor = NULL;
	greenColor = NULL;
	blueColor = NULL;

	delete rowIntMono;
	redMono = NULL;
	greenMono = NULL;
	blueMono = NULL;

	fe = flycaptureDestroyContext( flycapture );
	_HANDLE_FLYCAPTURE_ERROR( "flycaptureDestroyContext()", fe );

	// Destroy the Triclops context
	te = triclopsDestroyContext( triclops ) ;
	_HANDLE_TRICLOPS_ERROR( "triclopsDestroyContext()", te );
	//return 0;
}

#endif
