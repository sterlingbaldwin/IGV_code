//=============================================================================
// Copyright © 2007 Point Grey Research, Inc. All Rights Reserved.
//
// This software is the confidential and proprietary information of Point
// Grey Research, Inc. ("Confidential Information").  You shall not
// disclose such Confidential Information and shall use it only in
// accordance with the terms of the license agreement you entered into
// with PGR.
//
// PGR MAKES NO REPRESENTATIONS OR WARRANTIES ABOUT THE SUITABILITY OF THE
// SOFTWARE, EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
// PURPOSE, OR NON-INFRINGEMENT. PGR SHALL NOT BE LIABLE FOR ANY DAMAGES
// SUFFERED BY LICENSEE AS A RESULT OF USING, MODIFYING OR DISTRIBUTING
// THIS SOFTWARE OR ITS DERIVATIVES.
//=============================================================================
//=============================================================================
// $Id: HighDynamicRangeEx.cpp,v 1.2 2008/01/24 23:36:33 soowei Exp $
//=============================================================================
//=============================================================================
// System Includes
//=============================================================================
#include "stdafx.h"
#include "malloc.h"
#include "windows.h"
//=============================================================================
// PGR Includes
//=============================================================================
//=============================================================================
// Project Includes
//=============================================================================
#include "pgrflycapture.h"
#include "pgrflycaptureplus.h"

/*
  1800 HDR_Ctrl:
  Bit[0] Presence
  Bit[6] On/Off
  - write 82000000 to enable [and 80000000 to disable]

  1820 HDR_Shutter[0]:
  Bit[0] Presence
  Bit[6] On/Off
  Bit[20:31] Value [same as normal shutter]

  1824 HDR_Gain[0]:
  Bit[0] Presence
  Bit[6] On/Off 
  Bit[20:31] Value [same as normal gain]
    
  1840 HDR_Shutter[1]:
  - same fmt as HDR_Shutter[0]
      
  1844 HDR_Gain[1]:
  - same fmt as HDR_Gain[0]
	
  1860 HDR_Shutter[2]:
  - same fmt as HDR_Shutter[0]
	  
  1864 HDR_Gain[2]:
  - same fmt as HDR_Gain[0]
	    
  1880 HDR_Shutter[3]:
  - same fmt as HDR_Shutter[0]
	      
  1884 HDR_Gain[3]:
  - same fmt as HDR_Gain[0]
		
  Shutter 0-864    3 steps of 120 
  Gain 0-683       3 steps of E3
		  
*/
#define HDRCTRL	     0x1800
#define HDRSHUTTER_1 0x1820
#define HDRSHUTTER_2 0x1840
#define HDRSHUTTER_3 0x1860
#define HDRSHUTTER_4 0x1880

#define HDRGAIN_1    0x1824
#define HDRGAIN_2    0x1844
#define HDRGAIN_3    0x1864
#define HDRGAIN_4    0x1884

#define HDR_ON	     0x82000000
#define HDR_OFF	     0x80000000

//=============================================================================
// Main
//=============================================================================
int 
main(int /*argc*/, char* /*argv[]*/ )
{
   FlyCaptureContext context;
   FlyCaptureError   error;
   
   //
   // Create the context and initialize the camera
   //
   error = flycaptureCreateContext( &context );
   
   //
   // Initialize the context
   //
   error = flycaptureInitialize( context, 0 );
   error = flycaptureStart(context, FLYCAPTURE_VIDEOMODE_ANY, FLYCAPTURE_FRAMERATE_ANY);
   
   //
   // Check to see if the HDR table is supported
   //
   unsigned long ulRegVal;
   error = flycaptureGetCameraRegister( context, HDRCTRL, &ulRegVal );
   if( !0x80000000 & ulRegVal )
   {
      printf("This camera does not support the HDR feature. Exiting...\n");
      
      //
      // Destroy the context
      //
      flycaptureDestroyContext(context);
      return 0;
   }
   
   printf( "\nSetting up HDR Registers\n" );
   int fcError = 0;
   
   fcError  |= flycaptureSetCameraRegister( context,  HDRSHUTTER_1, 0x0 );
   fcError  |= flycaptureSetCameraRegister( context,  HDRSHUTTER_2, 0x120 );
   fcError  |= flycaptureSetCameraRegister( context,  HDRSHUTTER_3, 0x240 );
   fcError  |= flycaptureSetCameraRegister( context,  HDRSHUTTER_4, 0x360 );
   
   if(fcError  !=0)
   {  
      printf("Failed to write shutter values to camera. Exiting...\n");
      
      // Destroy the context
      flycaptureDestroyContext(context);
      return 0;   
   }
      
   fcError  |= flycaptureSetCameraRegister( context,  HDRGAIN_1, 0x0 );
   fcError  |= flycaptureSetCameraRegister( context,  HDRGAIN_2, 0xE3 );
   fcError  |= flycaptureSetCameraRegister( context,  HDRGAIN_3, 0x1C6 );
   fcError  |= flycaptureSetCameraRegister( context,  HDRGAIN_4, 0x2A9 );
   
   if(fcError  !=0)
   {  
      printf("Failed to write gain values to camera. Exiting...\n");
      
      // Destroy the context
      flycaptureDestroyContext( context );
      return 0;   
   }
   
   FlyCaptureImage imageConverted;
   FlyCaptureImage imageRaw;
   imageConverted.pData = NULL;
   imageConverted.pixelFormat = FLYCAPTURE_BGRU;
      
   // Turn on HDR
   error = flycaptureSetCameraRegister( context, HDRCTRL, HDR_ON );
   
   for( int i = 0; i < 4; ++i )
   {
      printf("Grabbing ... \n");
      error = flycaptureGrabImage2( context, &imageRaw );

      // Allocate memory if NULL, should happen only on the first time
      // we enter the loop
      if ( imageConverted.pData == NULL )
      {
	 imageConverted.pData = 
	    new unsigned char[imageRaw.iRows * imageRaw.iCols * 4];
      }

      // Convert raw image to BGRU
      error = flycaptureConvertImage( context, &imageRaw, &imageConverted );
      
      // Create filename
      char outputfilename[64];
      sprintf( outputfilename, "HDRImage%d.ppm", i );
      
      // Save the converted images to disk. 
      printf("Saving...");
      error = flycaptureSaveImage(
	 context,
	 &imageConverted,
	 outputfilename,
	 FLYCAPTURE_FILEFORMAT_PPM);     
   }  
   
   printf("Done. Press any key to exit.");
   getchar();
   
   //
   // Destroy the context
   //
   flycaptureDestroyContext( context );
   
   delete [] imageConverted.pData;
   return 0;
}

