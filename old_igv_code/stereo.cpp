#include "stereo.h"

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

   TriclopsInput		stereoData;
   TriclopsInput		colorData;
   TriclopsImage16		depthImage16;
   TriclopsImage		monoImage;
   TriclopsColorImage	colorImage;
   TriclopsContext		triclops;
   TriclopsError		te;

   float				x, y, z;
   int					r, g, b;
   FILE*				pointFile;
   FILE*				pointFile1;
   int					nPoints = 0;
   int					pixelinc ;
   int					i, j, k;
   unsigned short*		row;
   unsigned short		disparity;

   FlyCaptureContext		flycapture;
   FlyCaptureImage			flycaptureImage;
   FlyCaptureInfoEx			pInfo;
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
   te = triclopsSetDisparity( triclops, 0, 240);
   _HANDLE_TRICLOPS_ERROR( "triclopsSetDisparity()", te );
      te = triclopsSetEdgeCorrelation( triclops, 1);
   _HANDLE_TRICLOPS_ERROR( "triclopsSetDisparity()", te );
   te = triclopsSetEdgeMask( triclops, 7);
   _HANDLE_TRICLOPS_ERROR( "triclopsSetDisparity()", te );

   // Set surface validation
	te = triclopsSetSurfaceValidation( triclops, 1 );
	_HANDLE_TRICLOPS_ERROR( "triclopsSetSurfaceValidation()", te );
	te = triclopsSetSurfaceValidationSize( triclops, 400 );
   _HANDLE_TRICLOPS_ERROR( "triclopsSetSurfaceValidationSize()", te );
	te = triclopsSetSurfaceValidationDifference( triclops, 3.0 );
   _HANDLE_TRICLOPS_ERROR( "triclopsSetSurfaceValidationDifference()",	te );

   // Set Backforth validation
	te = triclopsSetBackForthValidation( triclops, 1 );
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

   // Create buffers for holding the color and mono images
   unsigned char* rowIntColor =
      new unsigned char[ imageCols * imageRows * iSideBySideImages * 4 ];
   unsigned char* rowIntMono =
      new unsigned char[ imageCols * imageRows * iSideBySideImages ];

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
   unsigned char* redColor = NULL;
   unsigned char* greenColor = NULL;
   unsigned char* blueColor = NULL;

   redColor = rowIntColor;
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
   unsigned char* redMono = NULL;
   unsigned char* greenMono = NULL;
   unsigned char* blueMono = NULL;

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

#if 0
 //     {
	float map[500] = {0};
	int m;
	i=depthImage16.ncols-1;
	pixelinc = depthImage16.rowinc/2;
	while(i--)
	{
		row= depthImage16.data + i * pixelinc;
		j=depthImage16.nrows-1;
		while(j--)
		{
			disparity = row[j];
			// do not save invalid points
			if ( disparity < 0xFF00 )
			{
				// convert the 16 bit disparity value to floating point x,y,z
				triclopsRCD16ToXYZ( triclops, i, j, disparity, &x, &y, &z );
				//fprintf( pointFile, "x %f y %f z %f col %d row %d\n", x, y, z,i,j );

				m=(int)(x*100)+250;
				if (!map[m])
				{
					map[m] = 10;
				}
				//printf("%d\n",m);
				else if(0<m && m<500)
					{
						if((y>-.1) && (map[m]>z) && (z>0))
						map[m]=z;
						nPoints++;
					}
				else printf("%d\n",m);
			}
		}
	}
	/*for(i=0;i<500;i++)
	{
		fprintf(pointFile,"%1.2f",map[i]);
		if (i%20)
			fprintf(pointFile, " ");
		else
			fprintf(pointFile, "\n");
	}*/

#endif

	float map[500] = {0};
	int m;
	i=depthImage16.ncols-1;
	pixelinc = depthImage16.rowinc/2;
	while(i--)
	{
		row= depthImage16.data + i * pixelinc;
		j=depthImage16.nrows-1;
		while(j--)
		{
			disparity = row[j];
			// do not save invalid points
			if ( disparity < 0xFF00 )
			{
				// convert the 16 bit disparity value to floating point x,y,z
				triclopsRCD16ToXYZ( triclops, i, j, disparity, &x, &y, &z );
				//fprintf( pointFile, "x %f y %f z %f col %d row %d\n", x, y, z,i,j );
				if ((y < .4) && (y > .1))
				{
					m=(int)(x*100)+250;
					if ((z < map[m]) || (!map[m]))
					{
						map[m] = z;
					}
				}
			}
		}
	}
	for(i=0;i<500;i++)
	{
		if(map[i]==0.0)
			map[i]=10.0;
		fprintf(pointFile,"%d, %2.2f \n",i,map[i]);
		/*
		if (i%20)
			fprintf(pointFile, ", ");
		else
			fprintf(pointFile, "\n");*/
	}



///////////////////////////////////////
#if 1
	float bitmap[500][500] = {0};
	int n;
	i=depthImage16.ncols-1;
	pixelinc = depthImage16.rowinc/2;
	while(i--)
	{
		row= depthImage16.data + i * pixelinc;
		j=depthImage16.nrows-1;
		while(j--)
		{
			disparity = row[j];
			// do not save invalid points
			if ( disparity < 0xFF00 )
			{
				// convert the 16 bit disparity value to floating point x,y,z
				triclopsRCD16ToXYZ( triclops, i, j, disparity, &x, &y, &z );
				n=(int)(-x*50)+50;
				m=(int)(z*50)+25;
				if((0<m && m<500) && (0<n && n<100)&&(y>-.25))
					{
					bitmap[m][n]=y;
					}
			}
		}
	}
	for(i=0;i<500;i++)
	{
		for(j=0;j<500;j++)
		{
			if(bitmap[i][j] == 0.00)
			{
				fprintf(pointFile1,"    ");
			}
			else
			{
				fprintf(pointFile1,"%1.2f",bitmap[i][j]);
				nPoints++;
			}
			fprintf(pointFile1,",");
		}
		fprintf(pointFile1,"\r\n");
	}
#endif
//////////////////////////////////////
   fclose( pointFile );
   printf( "Points in file: %d\t", nPoints );

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
