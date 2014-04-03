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
// $Id: Form1.cs,v 1.15 2008/06/24 17:36:22 soowei Exp $
//=============================================================================

using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Text;
using System.Windows.Forms;
using System.Diagnostics;
using System.Threading;

namespace ActiveFlyCapCSharpEx
{
   /// <summary>
   /// ActiveFlyCapForm form declaration
   /// </summary>
   public partial class ActiveFlyCapForm : Form
   {
      ActiveFlyCapLib.CameraInfo m_cameraInfo;
      Thread m_workerThread;
      AutoResetEvent m_grabLoopExitEvent;
      bool m_bExitGrab = false;
      bool m_bPauseGrab = false;
      bool m_bDrawShapes = false;
      long m_lFramesCaptured = 0;

      /// <summary>
      /// Form constructor
      /// </summary>
      public ActiveFlyCapForm()
      {
	 InitializeComponent();

	 m_grabLoopExitEvent = new AutoResetEvent(false);

	 object camList = axActiveFlyCapControl.BusEnumerateCameras();
	 if (camList != null)
	 {
	    Array arCamList = (Array)camList;
	    Debug.WriteLine("Enumerating cameras...");
	    foreach (string s in arCamList)
	    {
	       Debug.WriteLine(s);
	    }
	    Debug.WriteLine("Done");

	    if (axActiveFlyCapControl.ShowCameraSelectionModal() == 1)
	    {
	       // Get camera info
	       m_cameraInfo = axActiveFlyCapControl.GetCameraInfo();

	       // Change the following line to determine which grab mode to use
	       axActiveFlyCapControl.SetGrabMode(ActiveFlyCapLib.GrabMode.LockNext);

	       // Start the selected camera
	       axActiveFlyCapControl.Start = 1;

	       if (axActiveFlyCapControl.GetGrabMode() != ActiveFlyCapLib.GrabMode.FreeRunning)
	       {
		  // Create a grab thread
		  m_workerThread = new Thread(new ThreadStart(this.GrabLoop));
		  m_workerThread.Start();
	       }	 

	       updateStartStopButton();

	       populateCameraInfo();
	    }
	    else
	    {
	       // TODO: Disable the GUI	       
	    }
	 }
	 else
	 {
	    // TODO: Disable the GUI
	 }
      }      

      /// <summary>
      /// Grab loop. All this does is keep trying to grab an image
      /// </summary>
      private void GrabLoop()
      {	 
	 while (m_bExitGrab != true)
	 {
	    // Keep trying to grab an image
	    if (m_bPauseGrab != true)
	    {
	       // This call is non blocking
	       ActiveFlyCapLib.GrabError grabError;
	       axActiveFlyCapControl.GrabImage();
	    }	   

	    // Give up the time slice
	    Thread.Sleep(1);
	 }

	 Debug.WriteLine("Quitting grab thread");

	 // Signal that the grab thread is exiting
	 m_grabLoopExitEvent.Set();
      }

      /// <summary>
      /// Form closing handler
      /// </summary>
      /// <param name="sender"></param>
      /// <param name="e"></param>
      private void Form1_FormClosing(object sender, FormClosingEventArgs e)
      {
	 if (axActiveFlyCapControl.GetGrabMode() != ActiveFlyCapLib.GrabMode.FreeRunning)
	 {
	    // Kill the grab thread
	    m_bExitGrab = true;

	    // Wait for the thread to signal that it is complete
	    m_grabLoopExitEvent.WaitOne();
	 }

	 // Stop camera if needed
	 if (axActiveFlyCapControl.Start == 1)
	 {
	    axActiveFlyCapControl.Start = 0;
	 }
      }

      /// <summary>
      /// Start/Stop button handler
      /// </summary>
      /// <param name="sender"></param>
      /// <param name="e"></param>
      private void buttonStartStop_Click(object sender, EventArgs e)
      {
	 // Start or stop the camera as needed
	 if (axActiveFlyCapControl.Start == 0)
	 {
	    axActiveFlyCapControl.Start = 1;
	    m_bPauseGrab = false;
	 }
	 else
	 {
	    m_bPauseGrab = true;
	    axActiveFlyCapControl.Start = 0;
	 }

	 updateStartStopButton();
      }

      /// <summary>
      /// Camera control dialog button handler
      /// </summary>
      /// <param name="sender"></param>
      /// <param name="e"></param>
      private void buttonCamControlDlg_Click(object sender, EventArgs e)
      {
	 axActiveFlyCapControl.ToggleSettingsWindowState();
      }      

      /// <summary>
      /// Quit button handler
      /// </summary>
      /// <param name="sender"></param>
      /// <param name="e"></param>
      private void buttonQuit_Click(object sender, EventArgs e)
      {
	 Application.Exit();
      }

      /// <summary>
      /// Get register button handler
      /// </summary>
      /// <param name="sender"></param>
      /// <param name="e"></param>
      private void buttonGetRegister_Click(object sender, EventArgs e)
      {	 
	 try
	 {
	    // Parse textbox as hex	 
	    int iRegister = Convert.ToInt32(textBox1.Text, 16);

	    int iValue = axActiveFlyCapControl.GetCameraRegister(iRegister);

	    Debug.WriteLine("Register: " + iRegister.ToString("X") + " Value: " + iValue.ToString("X"));

	    // Put the reg value back in the textbox
	    textBox2.Text = iValue.ToString("X");
	 }
	 catch (System.ArgumentOutOfRangeException e_arg)
	 {
	    // Invalid register
	    MessageBox.Show(
	       "Invalid argument. Error was " + e_arg.ToString(),
	       "Invalid argument",
	       MessageBoxButtons.OK,
	       MessageBoxIcon.Error);
	 }
	 catch (System.ArgumentException e_arg)
	 {
	    // Invalid register
	    MessageBox.Show(
	       "Error reading register. Error was " + e_arg.ToString(),
	       "Register error",
	       MessageBoxButtons.OK,
	       MessageBoxIcon.Error);
	 }	 	 	 
      }

      /// <summary>
      /// Set register button handler
      /// </summary>
      /// <param name="sender"></param>
      /// <param name="e"></param>
      private void buttonSetRegister_Click(object sender, EventArgs e)
      {		 
	 try
	 {
	    // Parse textboxes as hex
	    int iRegister = Convert.ToInt32(textBox1.Text, 16);
	    int iValue = Convert.ToInt32(textBox2.Text, 16);

	    // Set the register to the camera	 
	    axActiveFlyCapControl.SetCameraRegister(iRegister, iValue);

	    Debug.WriteLine("Register: " + iRegister.ToString("X") + " Value: " + iValue.ToString("X"));
	 }
	 catch (System.ArgumentOutOfRangeException e_arg)
	 {
	    // Invalid register
	    MessageBox.Show(
	       "Invalid argument. Error was " + e_arg.ToString(),
	       "Invalid argument",
	       MessageBoxButtons.OK,
	       MessageBoxIcon.Error);
	 }
	 catch (System.ArgumentException e_arg)
	 {
	    // Invalid register
	    MessageBox.Show(
	       "Error reading register. Error was " + e_arg.ToString(),
	       "Register error",
	       MessageBoxButtons.OK,
	       MessageBoxIcon.Error);
	 }	 
      }

      /// <summary>
      /// Auto resize checkbox handler
      /// </summary>
      /// <param name="sender"></param>
      /// <param name="e"></param>
      private void checkBoxAutoResize_CheckedChanged(object sender, EventArgs e)
      {
	 if (checkBoxAutoResize.Checked == true)
	 {
	    axActiveFlyCapControl.AutoResize = 1;
	 }
	 else if (checkBoxAutoResize.Checked == false)
	 {
	    axActiveFlyCapControl.AutoResize = 0;
	 }
      }

      /// <summary>
      /// Draw shapes checkbox handler
      /// </summary>
      /// <param name="sender"></param>
      /// <param name="e"></param>
      private void checkBoxDrawShapes_CheckedChanged(object sender, EventArgs e)
      {
	 m_bDrawShapes = checkBoxDrawShapes.Checked;
      }

      /// <summary>
      /// Update caption on Start/Stop button
      /// </summary>
      private void updateStartStopButton()
      {
	 if (axActiveFlyCapControl.Start == 1)
	 {
	    buttonStartStop.Text = "Stop";
	 }
	 else
	 {
	    buttonStartStop.Text = "Start";
	 }
      }

      /// <summary>
      /// New camera handler
      /// </summary>
      /// <param name="sender"></param>
      /// <param name="e"></param>
      private void newCameraToolStripMenuItem_Click(object sender, EventArgs e)
      {
	 axActiveFlyCapControl.Start = 0;

	 object camList = axActiveFlyCapControl.BusEnumerateCameras();
	 if (camList != null)
	 {
	    axActiveFlyCapControl.ShowCameraSelectionModal();

	    m_cameraInfo = axActiveFlyCapControl.GetCameraInfo();
	    axActiveFlyCapControl.Start = 1;

	    updateStartStopButton();
	    populateCameraInfo();
	 }
	 else
	 {
	    // Disable GUI
	 }
      }

      /// <summary>
      /// Exit handler
      /// </summary>
      /// <param name="sender"></param>
      /// <param name="e"></param>
      private void exitToolStripMenuItem_Click(object sender, EventArgs e)
      {
	 // Stop camera if needed
	 if (axActiveFlyCapControl.Start == 1)
	 {
	    axActiveFlyCapControl.Start = 0;
	 }

	 Application.Exit();
      }
      
      /// <summary>
      /// Save image handler
      /// </summary>
      /// <param name="sender"></param>
      /// <param name="e"></param>
 
      private void saveImageToolStripMenuItem_Click(object sender, EventArgs e)
      {
	 SaveFileDialog saveFileDialog = new SaveFileDialog();

	 saveFileDialog.Filter = 
	    "PGM files (*.pgm)|*.pgm|" +
	    "PPM files (*.ppm)|*.ppm|" + 
	    "BMP files (*.bmp)|*.bmp|" + 
	    "JPG files (*.jpg)|*.jpg|" + 
	    "PNG files (*.png)|*.png|" +
	    "RAW files (*.raw)|*.raw|" + 
	    "All files (*.*)|*.*";
	 saveFileDialog.FilterIndex = 1;
	 saveFileDialog.RestoreDirectory = true;

	 if (saveFileDialog.ShowDialog() == DialogResult.OK)
	 {
	    // Create a lower case string
	    string lowercaseFileName = saveFileDialog.FileName.ToLower();
	    ActiveFlyCapLib.ImageFileFormat format;

	    if (lowercaseFileName.Contains(".pgm"))
	    {
	       format = ActiveFlyCapLib.ImageFileFormat.PGM;
	    }
	    else if (lowercaseFileName.Contains(".ppm"))
	    {
	       format = ActiveFlyCapLib.ImageFileFormat.PPM;
	    }
	    else if (lowercaseFileName.Contains(".bmp"))
	    {
	       format = ActiveFlyCapLib.ImageFileFormat.BMP;
	    }
	    else if (lowercaseFileName.Contains(".jpg") || lowercaseFileName.Contains("jpeg"))
	    {
	       format = ActiveFlyCapLib.ImageFileFormat.JPG;
	    }
	    else
	    {
	       // Don't process png, raw or anything else
	       MessageBox.Show(
		  "File format not supported",
		  "File format error",
		  MessageBoxButtons.OK,
		  MessageBoxIcon.Error);

	       return;
	    }

	    axActiveFlyCapControl.SaveImage(saveFileDialog.FileName, format);	        
	 }
      }

      /// <summary>
      /// Show/hide camera control dialog
      /// </summary>
      /// <param name="sender"></param>
      /// <param name="e"></param>
      private void cameraControlDialogToolStripMenuItem_Click(object sender, EventArgs e)
      {
	 // Toggle settings
	 axActiveFlyCapControl.ToggleSettingsWindowState();
      }

      /// <summary>
      /// Handle bus reset
      /// </summary>
      /// <param name="sender"></param>
      /// <param name="e"></param>
      private void axFlyCapActiveXControl_BusReset(object sender, EventArgs e)
      {
	 Debug.WriteLine("Bus reset");
      }

      /// <summary>
      /// Handle camera arrival
      /// </summary>
      /// <param name="sender"></param>
      /// <param name="e"></param>
      private void axFlyCapActiveXControl_CameraArrival(object sender, AxActiveFlyCapLib._IActiveFlyCapControlEvents_CameraArrivalEvent e)
      {
	 Debug.WriteLine("Camera arrival #" + e.lSerialNum.ToString());

	 if (this.InvokeRequired)
	 {
	    this.BeginInvoke(new MethodInvoker(delegate()
	    {
	       try
	       {
		  axActiveFlyCapControl.Start = 0;
		  axActiveFlyCapControl.BusEnumerateCameras();

		  if (axActiveFlyCapControl.ShowCameraSelectionModal() == 1)
		  {
		     // Start the selected camera
		     m_cameraInfo = axActiveFlyCapControl.GetCameraInfo();
		     axActiveFlyCapControl.Start = 1;
		     updateStartStopButton();
		     populateCameraInfo();
		  }
	       }
	       catch (System.Exception eException)
	       {
		  Debug.WriteLine(eException.ToString());
	       }
	    }));
	 }	 	
      }

      /// <summary>
      /// Handle camera removal
      /// </summary>
      /// <param name="sender"></param>
      /// <param name="e"></param>
      private void axFlyCapActiveXControl_CameraRemoval(object sender, AxActiveFlyCapLib._IActiveFlyCapControlEvents_CameraRemovalEvent e)
      {
	 Debug.WriteLine("Camera removal #" + e.lSerialNum.ToString());

	 if (this.InvokeRequired)
	 {
	    this.BeginInvoke(new MethodInvoker(delegate()
	    {
	       try
	       {		  
		  axActiveFlyCapControl.Start = 0;

		  axActiveFlyCapControl.BusEnumerateCameras();

		  if (axActiveFlyCapControl.ShowCameraSelectionModal() == 1)
		  {
		     // Start the selected camera
		     m_cameraInfo = axActiveFlyCapControl.GetCameraInfo();
		     axActiveFlyCapControl.Start = 1;
		     updateStartStopButton();
		     populateCameraInfo();
		  }
	       }
	       catch (System.Exception eException)
	       {
		  Debug.WriteLine(eException.ToString());
	       }
	    }));
	 }	 	 
      }

      /// <summary>
      /// Image grabbed handler. The image data can be accessed within this function
      /// </summary>
      /// <param name="sender"></param>
      /// <param name="e"></param>
      private void axFlyCapActiveXControl_ImageGrabbed(object sender, EventArgs e)
      {
	 m_lFramesCaptured++;

	 // Get image info
	 ActiveFlyCapLib.ImageInfo imageInfo;
	 imageInfo = axActiveFlyCapControl.GetImageInformation();
	 
	 string strPixelFormat;
	 switch (imageInfo.pixelFmt)
	 {
	 case ActiveFlyCapLib.PixelFormat.Mono8:
	    strPixelFormat = "Mono8";
	    break;
	 case ActiveFlyCapLib.PixelFormat.Mono16:
	    strPixelFormat = "Mono16";
	    break;
	 case ActiveFlyCapLib.PixelFormat.Raw8:
	    strPixelFormat = "Raw8";
	    break;
	 case ActiveFlyCapLib.PixelFormat.Raw16:
	    strPixelFormat = "Raw16";
	    break;
	 case ActiveFlyCapLib.PixelFormat.RGB8:
	    strPixelFormat = "RGB8";
	    break;
	 case ActiveFlyCapLib.PixelFormat.RGB16:
	    strPixelFormat = "RGB16";
	    break;
	 case ActiveFlyCapLib.PixelFormat.YUV411:
	    strPixelFormat = "YUV411";
	    break;
	 case ActiveFlyCapLib.PixelFormat.YUV422:
	    strPixelFormat = "YUV422";
	    break;
	 case ActiveFlyCapLib.PixelFormat.YUV444:
	    strPixelFormat = "YUV444";
	    break;
	 default:
	    strPixelFormat = "N/A";
	    break;
	 }
	 
	 // Get the frame rate
	 ActiveFlyCapLib.AbsPropertyStruct framerateStruct;
	 framerateStruct = axActiveFlyCapControl.GetCameraAbsProperty(
	    ActiveFlyCapLib.CameraProperty.FrameRate);

	 toolStripStatusLabel1.Text =
	    "Image Info: " +
	    imageInfo.lCols.ToString() + "x" + imageInfo.lRows.ToString() +
	    " " + strPixelFormat +
	    " " + framerateStruct.fValue.ToString() + "Hz" +
	    " Image Avg: " + calcImageAvg(imageInfo).ToString();

	 // Draw shapes on screen
	 if ( m_bDrawShapes )
	 {
	    drawShapes(imageInfo);
	 }	 

	 // Draw image to screen
	 if (axActiveFlyCapControl.GetGrabMode() != ActiveFlyCapLib.GrabMode.FreeRunning)
	 {
	    // This should be set to 1 if the raw data has been modified
	    axActiveFlyCapControl.DrawSingleImage(0);
	 }
      }

      /// <summary>
      /// Fills in the camera info on the form
      /// </summary>
      private void populateCameraInfo()
      {
	 // Populate the camera info labels
	 labelSerialNum.Text = m_cameraInfo.lSerialNumber.ToString();

	 if (m_cameraInfo.camType == ActiveFlyCapLib.CameraType.BlackAndWhite)
	 {
	    labelCameraType.Text = "Mono";
	 }
	 else
	 {
	    labelCameraType.Text = "Color";
	 }

	 labelCameraModel.Text = m_cameraInfo.bstrModelName;
	 labelSensor.Text = m_cameraInfo.bstrSensorInfo;

	 float fDCAMVer = m_cameraInfo.lDCAMVer / 100.0f;
	 labelDCAMVer.Text = fDCAMVer.ToString();
	 labelBusNode.Text = m_cameraInfo.lBusNum.ToString() + " / " + m_cameraInfo.lNodeNum.ToString();

	 switch (m_cameraInfo.maxBusSpeed)
	 {
	 case ActiveFlyCapLib.BusSpeed.S100:
	    labelBusSpeed.Text = "S100";
	    break;
	 case ActiveFlyCapLib.BusSpeed.S200:
	    labelBusSpeed.Text = "S200";
	    break;
	 case ActiveFlyCapLib.BusSpeed.S400:
	    labelBusSpeed.Text = "S400";
	    break;
	 case ActiveFlyCapLib.BusSpeed.S800:
	    labelBusSpeed.Text = "S800";
	    break;
	 }
      }

      /// <summary>
      /// Calculate the image average
      /// </summary>
      /// <param name="imageInfo"></param>
      /// <returns></returns>
      private int calcImageAvg(ActiveFlyCapLib.ImageInfo imageInfo)
      {
	 // Comment this out if you want to calculate the average
	 return 0;
	 
	 // This should only be called if the control is not in free
	 // running mode
	 if (axActiveFlyCapControl.GetGrabMode() == ActiveFlyCapLib.GrabMode.FreeRunning)
	 {
	    return 0;
	 }

	 long lAvg = 0;
	 int iImageSize = imageInfo.lRows * imageInfo.lRowInc;

	 unsafe
	 {
	    // Get the pointer to the data
	    object obj = axActiveFlyCapControl.GetImagePtr( ActiveFlyCapLib.ImageType.RawImage );
	    int temp = (int)obj;
	    byte* pArray = (byte*)temp;

	    // Iterate over the array and calculate the average
	    for (int i = 0; i < iImageSize; i++)
	    {
	       try
	       {
		  lAvg += pArray[i];
	       }
	       catch (System.AccessViolationException e_accessViolation)
	       {
		  Debug.WriteLine("Exception: " + e_accessViolation.ToString());
	       }
	    }

	    lAvg /= iImageSize;
	 }

	 return (int)lAvg;
      }

      /// <summary>
      /// Draw shapes onto the control
      /// </summary>
      /// <param name="imageInfo"></param>
      private void drawShapes(ActiveFlyCapLib.ImageInfo imageInfo)
      {
	 int iThickness = (int)(0.005 * Math.Max(imageInfo.lRows, imageInfo.lCols));

	 // Rectangle
	 axActiveFlyCapControl.DrawRectangle(
	    (short)(imageInfo.lCols / 3),
	    (short)(imageInfo.lRows / 3),
	    (short)((imageInfo.lCols * 2) / 3),
	    (short)((imageInfo.lRows * 2) / 3),
	    (short)iThickness,
	    255,
	    0,
	    0);

	 // Vertical line
	 axActiveFlyCapControl.DrawLine(
	    (short)(imageInfo.lCols / 2),
	    0,
	    (short)(imageInfo.lCols / 2),
	    (short)imageInfo.lRows,
	    (short)iThickness,
	    0,
	    255,
	    0);

	 // Circle
	 int iRadius = (int)(0.03 * Math.Max(imageInfo.lRows, imageInfo.lCols));
	 axActiveFlyCapControl.DrawEllipse(
	    (short)((imageInfo.lCols / 2) - iRadius),
	    (short)((imageInfo.lRows / 2) - iRadius),
	    (short)((imageInfo.lCols / 2) + iRadius),
	    (short)((imageInfo.lRows / 2) + iRadius),
	    (short)iThickness,
	    128,
	    0,
	    128);

	 // Text
	 // Graphics unit needs to be point
	 Font newFont = new Font(
	    FontFamily.GenericSansSerif,
	    36,
	    FontStyle.Regular,
	    GraphicsUnit.Point);
	 axActiveFlyCapControl.Font = newFont;
	 axActiveFlyCapControl.DrawText(
	    10,
	    10,
	    "Frames Captured: " + m_lFramesCaptured.ToString(),
	    255,
	    0,
	    0);	 
      }
   }
}