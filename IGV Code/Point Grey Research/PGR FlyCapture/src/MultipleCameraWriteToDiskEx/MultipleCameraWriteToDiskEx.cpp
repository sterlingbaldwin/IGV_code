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
// $Id: MultipleCameraWriteToDiskEx.cpp,v 1.5 2008/03/28 18:49:51 soowei Exp $
//=============================================================================
//=============================================================================
//
// MultipleCameraWriteToDiskEx
//
// PGR cameras that are on the same 1394 bus, running at the same frame rate
// automatically synchronize with one another. This example illustrates how 
// to grab a series of images from multiple cameras, insuring that no images are 
// missed and that all of the images are synchronized. This example also
// shows how to use the Win32 API to write images to disk as fast as possible.
// A summary of disk writing speeds is shown before the application exits.
//
// The example makes use of flycaptureLockNext() and flycaptureUnlock() commands
// to insure that no images are lost. A call to flycaptureStartLockNext() causes 
// the library to enqueue incoming images to a length not exceeding the number of 
// allocated and available (not 'locked') buffers. Once this limit is hit, the 
// oldest images start to be lost. 
// Calls to flycaptureLockNext return the oldest available image that has yet to 
// be returned. The user is guaranteed access to this image (they have it 
// 'locked') until they return it to the buffer pool using the flycaptureUnlock() 
// command.
//
// The example illustrates how users can determine if any images have been lost
// as a result of buffer overrun. This is achieved by simply examining the 
// uiSeqNum field contained in the FlyCaptureImagePlus structure.  Any pair
// of images returned from subsequent calls to flycaptureLockNext() that have 
// uiSeqNum's that differ by more than 1 is an indication that images have been
// dropped.
//
// Although the cameras are always synchronized, it is possible for any pair of 
// cameras set of buffered images to be 'misaligned'. For example, a call to
// flycaptureStartLockNext() for one camera even a single frame time after the
// same call to a different camera can result in the buffers being misaligned by
// an image. This issue is resolved with a call to flycaptureSyncForLockNext().
// This command uses a mechanism that clears all of the stored buffers and 
// insures that the buffered images are aligned with respect to time.
//
// This example attempts to start all detected cameras with "any" video mode
// and "any" frame rate with no checking for bandwidth limitations. If a
// specific video mode and frame rate is required, change the _VIDEOMODE
// and _FRAMERATE definition found in the header file to the desired values.
//
// After writing the raw images to disk, the program will extract the images 
// from each camera file and output them as individual image files. If the 
// images in the raw file are stippled, the images saved to disk will be
// color processed.
//=============================================================================
//=============================================================================
// $Id: MultipleCameraWriteToDiskEx.cpp,v 1.5 2008/03/28 18:49:51 soowei Exp $
//=============================================================================
//=============================================================================
// System Includes
//=============================================================================
#include "stdafx.h"
#include "assert.h"
//=============================================================================
// PGR Includes
//=============================================================================
#include "pgrflycapture.h"
#include "pgrflycaptureplus.h"
//=============================================================================
// Project Includes
//=============================================================================
#include "MultipleCameraWriteToDiskEx.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

CWinApp theApp;

//
// Print the disk information to the screen
//
void printDiskInfo()
{
   DWORD dwSectPerClust;
   DWORD dwBytesPerSect;
   DWORD dwFreeClusters;
   DWORD dwTotalClusters;
   CString csDirName = _DESTINATION_DIRECTORY;
   
   GetDiskFreeSpace (csDirName, 
      &dwSectPerClust,
      &dwBytesPerSect, 
      &dwFreeClusters,
      &dwTotalClusters);
   
   printf( "This disk drive has:\n" );
   printf( "%d Total clusters\n", dwTotalClusters );
   printf( "%d Free clusters\n", dwFreeClusters );
   printf( "%d bytes/sector\n", dwBytesPerSect);
   printf( "%d sectors/cluster\n", dwSectPerClust );
   
   double dTotalSizeBytes = (double)dwTotalClusters * dwSectPerClust * dwBytesPerSect;
   double dTotalSizeGB = dTotalSizeBytes / ( 1024 * 1024 * 1024 );
   printf( "%.0lf bytes (%.2lfGB) total size\n", dTotalSizeBytes, dTotalSizeGB);
   
   double dFreeBytes = (double)dwFreeClusters * dwSectPerClust * dwBytesPerSect;
   double dFreeGB = dFreeBytes / ( 1024 * 1024 * 1024 );
   printf( "%.0lf bytes (%.2lfGB) free \n", dFreeBytes, dFreeGB);
}


//
// Allocates the buffers for the images
//
void allocateBuffers()
{
   printf( "Allocating buffers\n" );

   for( unsigned iCamera = 0; iCamera < g_uiNumCameras; iCamera++ )
   {
      g_arpBuffers[iCamera] = new unsigned char*[ _BUFFERS ];
      
      for( unsigned i = 0; i < _BUFFERS; i++ )
      {
         g_arpBuffers[iCamera][i] = new unsigned char[ _MAX_IMAGE_SIZE ];
      }
   }

   g_bAllocated = true;
}


//
// Deallocates the buffers for the images
//
void deallocateBuffers()
{
   printf( "Deallocating buffers...\n" );

   for( unsigned iCamera = 0; iCamera < g_uiNumCameras; iCamera++ )
   {      
      if( g_arpBuffers[iCamera] != NULL )
      {
         for( unsigned i = 0; i < _BUFFERS; i++ )
         {
            if( g_arpBuffers[iCamera][i] != NULL )
            {
               delete [] g_arpBuffers[iCamera][i];
               g_arpBuffers[iCamera][i] = NULL;
            }
         }      
         
         delete [] g_arpBuffers[iCamera];
         g_arpBuffers[iCamera] = NULL;
      }
   }

   g_bAllocated = false;
}


//
// Create the files that will be used to store the images
//
int createFiles( HANDLE* arhFile )
{
   for ( unsigned int uiCamera = 0; uiCamera < g_uiNumCameras; uiCamera++ )
   {
      char tempFilename[256];
      sprintf( tempFilename, "%scamera%d.tmp", _DESTINATION_DIRECTORY, uiCamera );

      printf( "Creating %s...\n", tempFilename );
      
      // Create temporary files to do writing to
      arhFile[uiCamera] = CreateFile( 
	 tempFilename,          
	 GENERIC_WRITE,          
	 0,                      
	 NULL,                   
	 CREATE_ALWAYS,          
	 FILE_ATTRIBUTE_NORMAL | 
	 FILE_FLAG_WRITE_THROUGH,
	 NULL);    
      
      if ( arhFile[uiCamera] == INVALID_HANDLE_VALUE ) 
      {
	 assert( false );	 
	 return -1;
      }
   }

   return 0;
}


//
// Grab and test loop
//
int doGrabLoop()
{
   FlyCaptureError   error = FLYCAPTURE_FAILED;
   unsigned int	     aruiPrevSeqNum[ _MAX_CAMERAS ];
   unsigned int	     aruiDelta[ _MAX_CAMERAS ];
   unsigned int	     aruiCycles[ _MAX_CAMERAS ];
   HANDLE	     arhFile[ _MAX_CAMERAS ];
   DWORD	     ardwBytesWritten[ _MAX_CAMERAS ];
   DWORD	     dwTotalKiloBytesWritten = 0;
   bool		     bMissed = false;
   bool		     bOutOfSync = false;
   unsigned int	     uiMissedImages = 0;
   unsigned int	     uiOutOfSyncImages = 0;
   __int64	     nStartTime = 0;
   __int64	     nEndTime = 0;
   __int64	     nDifference = 0;
   __int64	     nTotalTime = 0;
   __int64	     nGlobalStartTime = 0;
   __int64	     nGlobalEndTime = 0;
   __int64	     nGlobalTotalTime = 0;
   __int64	     nFrequency = 0;

   QueryPerformanceFrequency( (LARGE_INTEGER*)&nFrequency );
   QueryPerformanceCounter( (LARGE_INTEGER*)&nGlobalStartTime );

   printf( "Starting grab...\n" );
   
   // Create files to write to
   if ( createFiles( arhFile ) != 0 )
   {
      printf( "There was error creating the files\n" );
      return -1;
   }  

   BOOL bSuccess;

   //
   // Start grabbing the images
   //

   for( int iImage = 0; iImage < g_iNumImagesToGrab; iImage++ )
   {
#ifdef _VERBOSE
      printf( "Grabbing image %u\n", iImage );
#else
      printf( "." );
#endif

      unsigned int uiCamera = 0;

      // Grab an image from each camera
      for( uiCamera = 0; uiCamera < g_uiNumCameras; uiCamera++ )
      {
	 error = flycaptureLockNext( g_arContext[uiCamera], &g_arImageplus[uiCamera] );
         _HANDLE_ERROR( error, "flycaptureLockNext()" );

	 // Save image dimensions & bayer info from first image for each camera
	 if(iImage == 0)
	 {
   	    g_arImageTemplate[uiCamera] = g_arImageplus[uiCamera].image;
	    
	    error = flycaptureGetColorTileFormat(g_arContext[uiCamera], &g_arBayerTile[uiCamera]);
	    _HANDLE_ERROR( error, "flycaptureGetColorTileFormat()" );
	 }
      }

      for( uiCamera = 0; uiCamera < g_uiNumCameras; uiCamera++ )
      { 	 
	 // Start timer
	 QueryPerformanceCounter( (LARGE_INTEGER*)&nStartTime );

	 // Calculate the size of the image to be written
	 int iImageSize = 0;	 
	 int iRowInc = g_arImageplus[uiCamera].image.iRowInc;
	 int iRows = g_arImageplus[uiCamera].image.iRows;
	 iImageSize = iRowInc * iRows;
 
	 // Write to the file
	 bSuccess = WriteFile(
	    arhFile[uiCamera], 
	    g_arImageplus[ uiCamera ].image.pData, 
	    iImageSize,
	    &ardwBytesWritten[uiCamera], 
	    NULL ); 
	 
	 // End timer
	 QueryPerformanceCounter( (LARGE_INTEGER*)&nEndTime );	

	 // Ensure that the write was successful
	 if ( !bSuccess || ( ardwBytesWritten[uiCamera] != (unsigned)iImageSize ) ) 
	 {
	    printf( "Error writing to file for camera %u!\n", uiCamera );
	    return -1;
	 }
	 
	 // Update various counters
	 dwTotalKiloBytesWritten += (ardwBytesWritten[uiCamera] / 1024);
	 nDifference = nEndTime - nStartTime;
	 nTotalTime += nDifference;

	 // Keep track of the difference in image sequence numbers (uiSeqNum)
	 // in order to determine if any images have been missed.  A difference
	 // greater than 1 indicates that an image has been missed.
         if( iImage == 0 )
         {
	    // This is the first image, set up the variables
            aruiPrevSeqNum[uiCamera] = g_arImageplus[uiCamera].uiSeqNum;
            aruiDelta[uiCamera] = 1;
         }
         else
         {
	    // Get the difference in sequence numbers between the current
	    // image and the last image we received
            aruiDelta[uiCamera] = 
               g_arImageplus[uiCamera].uiSeqNum - aruiPrevSeqNum[uiCamera];
         }         
	
         if( aruiDelta[uiCamera] != 1 )
         {
            // We have missed an image.
	    bMissed = true;
            uiMissedImages += aruiDelta[uiCamera] - 1;
         }
	 else
	 {
	    bMissed = false;
	 }
         
         aruiPrevSeqNum[uiCamera] = g_arImageplus[uiCamera].uiSeqNum;

	 // Calculate the cycle count for the camera
         aruiCycles[uiCamera] = 
            g_arImageplus[uiCamera].image.timeStamp.ulCycleSeconds * 8000 +
            g_arImageplus[uiCamera].image.timeStamp.ulCycleCount;

	 // Determine the difference of the timestamp for every image from the
	 // first camera.  If the difference is greater than 1 cycle count, 
	 // register the camera as being out of synchronization.
	 int iDeltaFrom0 = abs( (int)(aruiCycles[uiCamera] - aruiCycles[0]) );
	 
         if( ( iDeltaFrom0 % ( 128 * 8000 - 1 ) ) > 1 )
         {
	    bOutOfSync = true;
	    uiOutOfSyncImages++;
	 }
	 else
	 {
	    bOutOfSync = false;
	 }
	 
#ifdef _VERBOSE
	 // Output is in the following order:
	 // - The index of the image being captured
	 // - The index of the camera that is currently being captured
	 // - The time taken to write the image to disk
	 // - Number of kilobytes written
	 // - Write speed (in MB/s)
	 // - Sequence number
	 // - Cycle seconds in timestamp
	 // - Cycle count in timestamp
	 // - Delta from 0th value
	 // - Missed an image?
	 // - Out of sync?
	 printf( 
	    "%04d: \t%02u\t%0.5f\t%.0lf\t%.2lf\t%04u\t%03u.%04u\t%d\t%s %s\n",
	    iImage,
	    uiCamera,
	    nDifference,
	    (double)ardwBytesWritten[ uiCamera ] / 1024.0,
	    (double)ardwBytesWritten[ uiCamera ] / ( 1024 * 1024 * nDifference ),
	    g_arImageplus[ uiCamera ].uiSeqNum,
	    g_arImageplus[ uiCamera ].image.timeStamp.ulCycleSeconds,
            g_arImageplus[ uiCamera ].image.timeStamp.ulCycleCount,
	    iDeltaFrom0,
	    bMissed ? "Y" : "N",
	    bOutOfSync ? "Y" : "N");
#endif
      }

      // Unlock image, handing the buffer back to the buffer pool.
      for( uiCamera = 0; uiCamera < g_uiNumCameras; uiCamera++ )
      {
         error = flycaptureUnlock( 
            g_arContext[uiCamera], g_arImageplus[uiCamera].uiBufferIndex );
         _HANDLE_ERROR( error, "flycaptureUnlock()" );
      }
   }

   //
   // Done grabbing images
   //

   QueryPerformanceCounter( (LARGE_INTEGER*)&nGlobalEndTime );
   nGlobalTotalTime = nGlobalEndTime - nGlobalStartTime;

   double dGlobalTotalTime = (double)nGlobalTotalTime / (double)nFrequency;
   double dTotalTime = (double)nTotalTime / (double)nFrequency;

   // Report on the results
   // Burst time is the time that was spent writing to disk only
   // Overall time is total time taken, including image grabs, calculations etc
   printf( 
      "\nBurst: Wrote %.1lfMB in %0.2fs ( %.2lfMB/sec )\n", 
      (double)( dwTotalKiloBytesWritten / 1024 ), 
      dTotalTime,
      (double)( dwTotalKiloBytesWritten / ( 1024 * dTotalTime ) ) );

   printf(
      "Overall: Wrote %.1lfMB in %0.2fs ( %.2lfMB/sec )\n",
      (double)( dwTotalKiloBytesWritten / 1024 ), 
      dGlobalTotalTime,
      (double)( dwTotalKiloBytesWritten / ( 1024 * dGlobalTotalTime ) ) );  
   
   printf( g_bSyncSuccess ? "Sync success\n" : "Sync failed\n" );
   printf( "Missed images = %u.\n", uiMissedImages );
   printf( "Out of sync images = %u.\n", uiOutOfSyncImages );

   for ( unsigned int uiCamera = 0; uiCamera < g_uiNumCameras; uiCamera++ )
   {
      // Close file handles
      CloseHandle(arhFile[uiCamera]);
   }

   return 0;
}


//
// Create all contexts
//
int createContexts()
{
   // Create all contexts
   FlyCaptureError error;
   for( unsigned int uiCamera = 0; uiCamera < g_uiNumCameras; uiCamera++ )
   {      
      error = flycaptureCreateContext( &g_arContext[uiCamera] );
      _HANDLE_ERROR( error, "flycaptureCreateContext()" );
   }

   return 0;
}


//
// Initialize all cameras, reset to factory defaults and power up
//
int initCameras()
{
   FlyCaptureError error;
   for ( unsigned int uiCamera = 0; uiCamera < g_uiNumCameras; uiCamera++ )
   {      
      printf( "Initializing camera %u.\n", uiCamera );
      error = flycaptureInitializePlus( 
         g_arContext[uiCamera], 
         uiCamera,
         _BUFFERS,
         g_arpBuffers[uiCamera] );
      _HANDLE_ERROR( error, "flycaptureInitializePlus()" );
      
      //
      // Reset the camera to default factory settings by asserting bit 0
      //
      error = flycaptureSetCameraRegister( 
	 g_arContext[uiCamera], INITIALIZE, 0x80000000 );
      _HANDLE_ERROR( error, "flycaptureSetCameraRegister()" );
      
      //
      // Power-up the camera (for cameras that support this feature)
      //
      error = flycaptureSetCameraRegister( 
	 g_arContext[uiCamera], CAMERA_POWER, 0x80000000 );
      _HANDLE_ERROR( error, "flycaptureSetCameraRegister()" );

      //
      // Turn on Frame Counter (0x40) and Timestamp (0x01) bits in FRAME_INFO 
      // to attach the frame count and timestamp to the header.
      //
      error = flycaptureSetCameraRegister( 
	 g_arContext[ uiCamera ], FRAME_INFO, 0x80000041 );
      _HANDLE_ERROR( error, "flycaptureSetCameraRegister()" );

      // Get and set the framerate
      float fFrameRate = 0.0;
      error = flycaptureGetCameraAbsProperty(g_arContext[ uiCamera ], FLYCAPTURE_FRAME_RATE, &fFrameRate );
      
      if (error == FLYCAPTURE_OK)
      {
	 error = flycaptureSetCameraAbsPropertyEx(
	    g_arContext[ uiCamera ],
	    FLYCAPTURE_FRAME_RATE,
	    false,
	    true,
	    true,
	    fFrameRate );
      }
   }

   return 0;
}


//
// Start all cameras
//
int startCameras()
{
   // Start cameras   
   FlyCaptureError error;
   for ( unsigned int uiCamera = 0; uiCamera < g_uiNumCameras; uiCamera++ )
   {
      printf( "Starting camera %u.\n", uiCamera );
      error = flycaptureStartLockNext( 
	 g_arContext[uiCamera], _VIDEOMODE, _FRAMERATE );
      _HANDLE_ERROR( error, "flycaptureStartLockNext()" );      
   }

   return 0;
}


//
// Stop all cameras
//
int stopCameras()
{
   // Stop all cameras from grabbing
   FlyCaptureError error;
   for( unsigned int uiCamera = 0; uiCamera < g_uiNumCameras; uiCamera++ )
   {
      error = flycaptureStop( g_arContext[uiCamera] );
      _HANDLE_ERROR( error, "flycaptureStop()" );
   }

   return 0;
}


//
// Destroy all contexts
//
int destroyContexts()
{
   // Destroy all contexts
   FlyCaptureError error;
   for( unsigned int uiCamera = 0; uiCamera < g_uiNumCameras; uiCamera++ )
   {      
      error = flycaptureDestroyContext( g_arContext[uiCamera] );
      _HANDLE_ERROR( error, "flycaptureDestroyContext()" );
   }

   return 0;
}


//
// Begins the test
//
int startTest()
{
   FlyCaptureError error = FLYCAPTURE_FAILED;

   // Enumerate the bus
   error = flycaptureBusEnumerateCamerasEx( g_arInfo, &g_uiNumCameras );
   _HANDLE_ERROR( error, "flycaptureBusEnumerateCamerasEx()" );

   if ( g_uiNumCameras < 2 )
   {
      printf( "At least 2 cameras are required! Aborting.\n" );
      return -1;
   }

   printf( "%u cameras found...\n", g_uiNumCameras );

   // Allocate buffers for all the cameras
   allocateBuffers();   

   // Create contexts
   if ( createContexts() != 0 )
   {
      printf( "There was an error creating the contexts.\n" );
      return -1;
   }

   // Initialize the cameras
   if ( initCameras() != 0 )
   {
      printf( "There was an error initializing the cameras.\n" );
      return -1;
   }

   // Start the cameras
   if ( startCameras() != 0 )
   {
      printf( "There was an error starting the cameras.\n" );
      return -1;
   }

   // Synchronize buffers
   error = flycaptureSyncForLockNext( g_arContext, g_uiNumCameras );
   if( error == FLYCAPTURE_OK )
   {
      g_bSyncSuccess = true;
   }
   else
   {     
      // Could not sync buffers
      printf( 
         "flycaptureSyncForLockNext() failed with %s!\n", 
         flycaptureErrorToString( error ) );
      
      g_bSyncSuccess = false;
   }

   // Grab and test loop here
   if ( doGrabLoop() != 0 )
   {
      printf( "There was an error in the grab loop!\n" );
      return -1;
   }

   // We are done, stop the cameras
   if ( stopCameras() != 0 )
   {
      printf( "There was an error stopping the cameras.\n" );
      return -1;
   }

   // Destroy the contexts
   if ( destroyContexts() != 0 )
   {
      printf( "There was an error destroying the contexts.\n" );
      return -1;
   }

   // Deallocate buffers
   deallocateBuffers();

   return 0;
}

//
// Extracts images from raw file, color process them and saves them as separate images
//
int splitImages()
{
   FILE* rawFile;
   char tempFilename[256];
   FlyCaptureImage dstImage;
   FlyCaptureContext context;
   FlyCaptureError error;

   dstImage.pData = g_dstBuffer;
   dstImage.pixelFormat = FLYCAPTURE_BGR;

   printf("\nSplitting and color-processing images\n");

   // Context for performing color-processing, no camera association required
   error = flycaptureCreateContext(&context);
   _HANDLE_ERROR( error, "flycaptureCreateContext()" );

   // Color processing method used if stipple flag set
   error = flycaptureSetColorProcessingMethod(context, _COLOR_PROCESSING_METHOD);
   _HANDLE_ERROR( error, "flycaptureSetColorProcessingMethod()" );

   // Open file
   for (unsigned int uiCamera = 0; uiCamera < g_uiNumCameras; uiCamera++)
   {
      sprintf(tempFilename, "%scamera%d.tmp", _DESTINATION_DIRECTORY, uiCamera);

      printf("Opening %s...\n", tempFilename);

      if ((rawFile = fopen(tempFilename, "rb")) == NULL)
      {
	 printf("Error opening file: %s\n", tempFilename);
	 return -1;
      }

      // Use bayer tile format from camera when color processing
      error = flycaptureSetColorTileFormat(context, g_arBayerTile[uiCamera]);
      _HANDLE_ERROR( error, "flycaptureSetColorTileFormat()" );

      
      unsigned int imageSize = g_arImageTemplate[uiCamera].iRows * g_arImageTemplate[uiCamera].iRowInc;
      
      // Read image into buffer
      for(int i = 0; i < _IMAGES; i++)
      {
	 if(fread(g_srcBuffer, sizeof(char), imageSize, rawFile) != imageSize)
	 {
	    printf("Error reading image # %d from file: %s\n", i, tempFilename);
	    return -1;
	 }

	 // Import image into FlycapturImage structure
	 g_arImageTemplate[uiCamera].pData = g_srcBuffer;

	 // Color process image
	 error = flycaptureConvertImage(context, &g_arImageTemplate[uiCamera], &dstImage);
	 _HANDLE_ERROR( error, "flycaptureConvertImage()" );

	 sprintf(tempFilename, "%scamera%d_%d.%s", _DESTINATION_DIRECTORY, uiCamera, i, _SAVE_FILE_EXT);

	 //  Save image to disk
	 error = flycaptureSaveImage(context, &dstImage, tempFilename, _SAVE_FILE_FORMAT);
	 _HANDLE_ERROR( error, "flycaptureSaveImage()" );

	 printf(".");
      }
      printf ("\n");

      // Close file
      fclose(rawFile);
   }

   error = flycaptureDestroyContext(context);
   _HANDLE_ERROR( error, "flycaptureDestroyContext()" );

   return 0;
}


//
// Main function
//
int _tmain(int /*argc*/, TCHAR* /*argv[]*/, TCHAR* /*envp[]*/ )
{
   int nRetCode = 0;
   
   // initialize MFC and print and error on failure
   if ( !AfxWinInit( ::GetModuleHandle(NULL), NULL, ::GetCommandLine(), 0) )
   {
      printf( "Fatal Error: MFC initialization failed" );
      nRetCode = 1;
   }
   
   // Echo disk information
   printDiskInfo();
   
   nRetCode = startTest();

   // Split and color process images
   nRetCode = splitImages();

   printf( "Done! (Hit enter)" );
   getchar();

   return nRetCode;
}