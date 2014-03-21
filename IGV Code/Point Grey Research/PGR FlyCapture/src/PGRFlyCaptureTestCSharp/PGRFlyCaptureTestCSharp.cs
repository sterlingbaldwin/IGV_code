//=============================================================================
// Copyright © 2006 Point Grey Research, Inc. All Rights Reserved.
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
// $Id: PGRFlyCaptureTestCSharp.cs,v 1.7 2009/01/23 21:48:49 hirokim Exp $
//=============================================================================

//==========================================================================
// This code provides an example of how to access PGR's FlyCapture DLLs from
// a C# .NET environment. The examples include the simple data types such as 
// booleans, ints, longs, char arrays, etc, as well as structures, 
// enumerations, and most importantly, images.
//
// To be able to run this code, you will have to enable the unsafe option.
// To do so:
// Project->[Filename] Properties...->Configuration Properties->Build
// Change "Allow unsafe code blocks" to 'True'
//
// In order to run the executable after compiling, you must change the
// .NET Security settings as follows:
//   Start->Settings->Control Panel->Administrative Tools->
//   Microsoft .NET Framework 1.1 Wizards->Adjust .NET Security->
//   Make changes to this computer->My Computer->Full Trust
//==========================================================================


using System;
using System.Runtime.InteropServices;

namespace PGRFlyCaptureTestCSharp
{
   unsafe class PGRFlyCaptureTestCSharp
   {
      //
      // DLL Functions to import
      // 
      // Follow this format to import any DLL with a specific function.
      //

      [DllImport("pgrflycapture.dll")]
      public static extern int flycaptureCreateContext(int* flycapcontext);

      [DllImport("pgrflycapture.dll")]
      public static extern int flycaptureStart(int flycapcontext, 
						FlyCaptureVideoMode videoMode, 
						FlyCaptureFrameRate frameRate);

      [DllImport("pgrflycapture.dll")]
      public static extern string flycaptureErrorToString(int error);

      [DllImport("pgrflycapture.dll")]
      public static extern int flycaptureInitialize(int flycapContext, int cameraIndex);

      [DllImport("pgrflycapture.dll")]
      public static extern int flycaptureGetCameraInfo(int flycapContext, ref FlyCaptureInfoEx arInfo);

      [DllImport("pgrflycapture.dll")]
      unsafe public static extern int flycaptureGrabImage2(int flycapContext, ref FlyCaptureImage image);

      [DllImport("pgrflycapture.dll")]
      unsafe public static extern int flycaptureSaveImage(int flycapContext, 
							    ref FlyCaptureImage image, 
							    string filename, 
							    FlyCaptureImageFileFormat fileFormat);

      [DllImport("pgrflycapture.dll")]
      public static extern int flycaptureStop(int flycapContext);

      [DllImport("pgrflycapture.dll")]
      public static extern int flycaptureDestroyContext(int flycapContext);

      [DllImport("pgrflycapture.dll")]
      public static extern int flycaptureConvertImage(int flycapContext, 
						       ref FlyCaptureImage image, ref FlyCaptureImage imageConvert);


      // The index of the camera to grab from.
      public const int _CAMERA_INDEX = 0;

      // The maximum number of cameras on the bus.
      public const int _MAX_CAMS = 3;

      // The number of images to grab.
      public const int _IMAGES_TO_GRAB = 10;


      static void reportError( int ret, string fname )
      {
	 Console.Write(fname + " error: " + flycaptureErrorToString(ret) + "\n");
	 Console.Write("\nPress Enter");
	 Console.Read();
	 return;
      }

      [STAThread]
      static void Main(string[] args)
      {
	 int flycapContext;
	 int ret;
	 FlyCaptureInfoEx flycapInfo = new FlyCaptureInfoEx();
	 FlyCaptureImage image = new FlyCaptureImage();
	 FlyCaptureImage flycapRGBImage = new FlyCaptureImage();

	 // Create the context.
	 ret= flycaptureCreateContext(&flycapContext);
	 if ( ret!= 0 ) // test
	 {
	    reportError(ret,"flycaptureCreateContext");
	    return;
	 }

	 // Initialize the camera.
	 ret = flycaptureInitialize( flycapContext, _CAMERA_INDEX );
	 if (ret!= 0 )
	 {
	    reportError(ret,"flycaptureInitialize");
	    return;
	 }

	 // Get the info for this camera.
	 ret = flycaptureGetCameraInfo( flycapContext, ref flycapInfo );
	 if (ret!= 0 )
	 {
	    reportError(ret,"flycaptureGetCameraInformation");
	    return;
	 }
	 if (flycapInfo.CameraType == FlyCaptureCameraType.FLYCAPTURE_BLACK_AND_WHITE)
	 {
	    Console.Write( "Model: B&W " + flycapInfo.pszModelString + "\n"
			    + "Serial #: " + flycapInfo.SerialNumber + "\n");
	 }
	 else if (flycapInfo.CameraType == FlyCaptureCameraType.FLYCAPTURE_COLOR)
	 {
	    Console.Write( "Model: Colour " + flycapInfo.pszModelString + "\n"
			   + "Serial #: " + flycapInfo.SerialNumber + "\n");
	 }

	 // Start FlyCapture.
	 ret= flycaptureStart( flycapContext, 
			      FlyCaptureVideoMode.FLYCAPTURE_VIDEOMODE_ANY,
			      FlyCaptureFrameRate.FLYCAPTURE_FRAMERATE_ANY );
	 if (ret != 0)
	 {
	    reportError(ret,"flycaptureStart");
	    return;
	 }

	 // Grab one image to examine the image size
	 ret = flycaptureGrabImage2(flycapContext, ref image);
	 if (ret != 0)
	 {
	    reportError(ret, "flycaptureGrabImage2");
	    return;
	 }

	 // Allocate buffer for color processed image
	 byte[] BGR_buffer = new byte[image.iRows * image.iCols * 3];

	 // Start grabbing images.
	 Console.Write("\nGrabbing Images ");
	 for ( int iImage = 0; iImage < _IMAGES_TO_GRAB; iImage++ )
	 {
	    // grab an image
	    ret = flycaptureGrabImage2( flycapContext, ref image );
	    if (ret!= 0 )
	    {
	       reportError(ret,"flycaptureGrabImage2");
	       return;
	    }

	    // Convert the image.
	    fixed (byte* pBGR_buffer = BGR_buffer)
	    {
	       flycapRGBImage.pData = pBGR_buffer;
	       flycapRGBImage.pixelFormat = FlyCapturePixelFormat.FLYCAPTURE_BGR;
	       ret = flycaptureConvertImage(flycapContext, ref image, ref flycapRGBImage);
	       if (ret != 0)
	       {
		  reportError(ret, "flycaptureConvertImage");
		  return;
	       }
	    }

	    Console.Write(".");
	 }


	 // Save the image.
	 Console.Write("\nSaving Last Image ");
	 ret = flycaptureSaveImage( flycapContext, ref flycapRGBImage, "color.bmp", 
				    FlyCaptureImageFileFormat.FLYCAPTURE_FILEFORMAT_BMP );
	 if (ret != 0)
	 {
	    reportError(ret, "flycaptureSaveImage");
	    return;
	 }
	 else
	 {
	    System.Diagnostics.Process.Start("mspaint.exe", "color.bmp");
	 }

	 // Stop FlyCapture.
	 ret = flycaptureStop(flycapContext);
	 if (ret!= 0 )
	 {
	    reportError(ret,"flycaptureStop");
	    return;
	 }

	 // Destroy the context.
	 ret = flycaptureDestroyContext(flycapContext);
	 if (ret!= 0 )
	 {
	    reportError(ret,"flycaptureDestroyContext");
	    return;
	 }

	 Console.Write("\nPress Enter");
	 Console.Read();
      }
   }
}