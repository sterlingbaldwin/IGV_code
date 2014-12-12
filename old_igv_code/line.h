#ifndef LINE_H
#define LINE_H

#include "globals.h"

#include <cv.h>
#include <cxcore.h>
#include <highgui.h>

#define RAMPDECLINETHRESH 1000	//no. of pixels seen as a ramp (???)

#define WHTSS 20                //no. of pixels to be sampled for a line segment, to compute statistics and tolerances
                                //used in mouse callback functions

#define TESTSIZE 16				//max no. of elements expected to be passed to fixline()

#define PERCENTWHITE 10

#define MINWHITENESS 80
#define CELLWIDTH 5				//max number of white cells to be marked (??)

/* current.type values: */
#define UNKNOWN 0
#define CAUTION 1
#define OBSTACLE 2
#define LINE 2
#define CLEAR 0

#define IMAGE 0
#define VIDEO 1
#define STEREO 2

#define LEFT -1
#define RIGHT 1

#define NONE 0

#define OUTPUT NONE

const float linecellwidth = 0.15;	//width of cells to find average white pixels in; 19 cells
const float linedistance = 2.1;		//distance [m] at which 19 (0.15 meter wide) cells occur w/in FOV
const int numlinecells = 19;	//number of cells

float huetolfactor = .85;       //the factor to be applied to standard deviation of hue
float sattolfactor = .85;       //the factor to be applied to standard deviation of saturation
float valtolfactor = .85;       //the factor to be applied to standard deviation of value (brightness)

#define DISPLAY 1

#define TIMER 1		/* Time the Algorithm */

#define PRIMARYSAT 15

//Defined constants
#define WHITE 7
#define RED 1
#define GREEN 2
#define YELLOW 3
#define BLUE 4
#define MAGENTA 5
#define CYAN 6
#define BLACK 0
//a constant array used to undefined, based on previous constants
const char *undefine_rgb[8] = {"White", "Red", "Green", "Yellow", "Blue", "Magenta", "Cyan", "Black"};
//End defined constants

//Color channel to be applied to region types
const char whiteoutcolor = GREEN;       //when the region is clear of obstacles
const char lineoutcolor = BLUE;         //when the region contains a line
const char obstaclecolor = RED;         //when the region is an obstacle
//Note: there will often be combinations of region types, e.g. yellow might be obstacle, might be clear
//Note: lines typically show up as cyan (blue+green), not just blue

//A debug feature where color channels and color combinations can be turned off
//This may not be working properly --> should be further investigated
#define SHOWALL 1
#if SHOWALL
#define SHOWWHITE SHOWALL
#define SHOWRED SHOWALL
#define SHOWGREEN SHOWALL
#define SHOWYELLOW SHOWALL
#define SHOWBLUE SHOWALL
#define SHOWMAGENTA SHOWALL
#define SHOWCYAN SHOWALL
#else
#define SHOWWHITE 1
#define SHOWRED 0
#define SHOWGREEN 0
#define SHOWYELLOW 0
#define SHOWBLUE 0
#define SHOWMAGENTA 0
#define SHOWCYAN 0
#endif

//Defined macros
//Present configuration (160+side*i) scans left and right side of images
#define RCHAN(img,side) ((uchar*)(img->imageData + img->widthStep*j))[(160+side*i)*3+2]
#define GCHAN(img,side) ((uchar*)(img->imageData + img->widthStep*j))[(160+side*i)*3+1]
#define BCHAN(img,side) ((uchar*)(img->imageData + img->widthStep*j))[(160+side*i)*3]

//Scans whole image
#define RCHANC(img) ((uchar*)(img->imageData + img->widthStep*j))[i*3+2]
#define GCHANC(img) ((uchar*)(img->imageData + img->widthStep*j))[i*3+1]
#define BCHANC(img) ((uchar*)(img->imageData + img->widthStep*j))[i*3]

//Present configuration (160+side*i) scans left and right side of images
#define VCHAN(img,side) ((uchar*)(img->imageData + img->widthStep*j))[(160+side*i)*3+2]
#define SCHAN(img,side) ((uchar*)(img->imageData + img->widthStep*j))[(160+side*i)*3+1]
#define HCHAN(img,side) ((uchar*)(img->imageData + img->widthStep*j))[(160+side*i)*3]

//Scans whole image
#define VCHANC(img) ((uchar*)(img->imageData + img->widthStep*j))[i*3+2]
#define SCHANC(img) ((uchar*)(img->imageData + img->widthStep*j))[i*3+1]
#define HCHANC(img) ((uchar*)(img->imageData + img->widthStep*j))[i*3]

#define MAXV *((uchar*)&current+current.max)
#define MEDV *((uchar*)&current+current.med)
#define MINV *((uchar*)&current+current.min)

#define MAXI current.max
#define MEDI current.med
#define MINI current.min
//End defined macros

//Begin struct definitions
typedef struct
{
	uchar index;
	uchar red;
	uchar green;
	uchar min;
	uchar blue;
	uchar max;
	uchar med;
	char type;
} pixel;

typedef struct
{
	uchar minbright;
	uchar maxsat;
} threshold;
//End struct definitions

//Why global???
IplImage *hsv = cvCreateImage(cvSize(320, 240), IPL_DEPTH_8U, 3);

////////////////////////////////////////
// BEGIN MOUSE CALLBACK FUNCTIONS ETC //
////////////////////////////////////////
struct wht8
{
	unsigned char hue;
	unsigned char sat;
	unsigned char val;
};

struct whtf
{
	float huem;		//mean
	float huev;		//variance
	float satm;
	float satv;
	float valm;
	float valv;
};

whtf whiteaverage;
wht8 whitepoints[WHTSS];
int numwhitepoints = 0;
bool endcalibration = false;

// Picks a sample of WHTSS number points to compute average statistics, used to determine tolerances
void mousecallback(int event, int i, int j, int flags, void* param)
{
	wht8 temp;
	temp.hue = HCHANC(hsv);
	temp.sat = SCHANC(hsv);
	temp.val = VCHANC(hsv);
	int whiteness = VCHANC(hsv) - SCHANC(hsv);
	if (event == CV_EVENT_LBUTTONDOWN)
	{
		printf("hue: %d\tsat: %d\tval: %dwht: %d\n", HCHANC(hsv), SCHANC(hsv), VCHANC(hsv), whiteness);
		if (!endcalibration)
		{
			whitepoints[numwhitepoints] = temp;
			whiteaverage.huem += temp.hue;
			whiteaverage.satm += temp.sat;
			whiteaverage.valm += temp.val;
			numwhitepoints++;
			if (numwhitepoints >= 20)
				endcalibration = true;
			printf("numwhitepoints: %d\n", numwhitepoints);
		}
	}
}

//////////////////////////////////////
// END MOUSE CALLBACK FUNCTIONS ETC //
//////////////////////////////////////

inline int get_pixel_extrema (pixel *current);
inline float GetAvgSlope(int side, IplImage *img/*, pixel* current*/);
float getline(CvPoint* points, int length, float* results);
float checkgoodness(CvPoint* points, int length, float* line);

int FindLines()
{
	long framecount = 0;
	printf("hLines started.\n");
	pixel current;

	IplImage *img = cvCreateImage(cvSize(320, 240), IPL_DEPTH_8U, 3);       //320 x 240, 8 bit unsigned data, 3 channels
	IplImage *orig = cvCreateImage(cvSize(320, 240), IPL_DEPTH_8U, 3);      //320 x 240, 8 bit unsigned data, 3 channels
	//IplImage *hsv = cvCreateImage(cvSize(320, 240), IPL_DEPTH_8U, 3);     //320 x 240, 8 bit unsigned data, 3 channels
#if DISPLAY
	IplImage *blue = cvCreateImage(cvSize(320, 240), IPL_DEPTH_8U, 1);
	cvNamedWindow("Input", 1);
	cvSetMouseCallback("Input", mousecallback);
	cvNamedWindow("Blue", 1);
	cvNamedWindow("Output", 1);
	//cvNamedWindow("HSV", 1);
#endif
#if OUTPUT == VIDEO		//record to video file
	CvVideoWriter *writer = 0;
	int isColor = 1;
	int fps = 5;
	int frameW = 320;
	int frameH = 240;
	writer = cvCreateVideoWriter("out.avi",-1,fps,cvSize(frameW,frameH),isColor);
#endif
	//uchar primarysat = PRIMARYSAT;
	bool showcolors[7];
	showcolors[WHITE] = SHOWWHITE;
	showcolors[RED] = SHOWRED;
	showcolors[GREEN] = SHOWGREEN;
	showcolors[YELLOW] = SHOWYELLOW;
	showcolors[BLUE] = SHOWBLUE;
	showcolors[MAGENTA] = SHOWMAGENTA;
	showcolors[CYAN] = SHOWCYAN;

	float huen = 0.0;
	float huet = 0.0;
	float satn = 0.0;
	float satt = 0.0;
	float valn = 0.0;
	float valt = 0.0;
	float huemin = 0.0;
	float huemax = 0.0;
	float satmin = 0.0;
	float satmax = 0.0;
	float valmin = 0.0;
	float valmax = 0.0;
	bool hueinv = false;
	bool huepass;
	float avgLaneAngle = 0.0;
	float avgLaneAngleQuality = 0.0;
#ifdef CALFILE_H        //exclude calibration routine if file is missing
	puts("enter \'r\' to read calibration data from file, any other key to read in");
	char readIn = getchar();
	if(readIn == 'r' || readIn == 'R')
	{
		readCalFile(&huemin, &huemax, &satmin, &satmax, &valmin, &valmax);
	}
	else
	{
#endif
        //begin calibration routine
        printf(" white points\n");
		while (!GetAsyncKeyState('c') && (!endcalibration))
		{
			WaitForSingleObject(NewImage, INFINITE);    //Wait INFINITE time for a NewImage event to be triggered (must be set by stereo thread)
                                                        //Timeouts should be added for error checking
			//framecount++;
			EnterCriticalSection(&RawDisparityMap);
			memcpy(temp2RGBimage, RGBimage, sizeof(temp2RGBimage));
			LeaveCriticalSection(&RawDisparityMap);
			orig->imageData = ((char *)(&temp2RGBimage));
			/* Test CV Stuff */
			// the same OpenCV ops done before the image will be compared to the calibration settings found here
			cvErode(orig, orig);
			cvMorphologyEx(orig, orig, NULL, NULL, CV_MOP_OPEN);
			cvShowImage("Input", orig);         //no real need to show HSV on an RGB monitor; it just looks weird
			cvCvtColor(orig, hsv, CV_BGR2HSV);
			cvWaitKey(10);                      //necessary delay for video stream in or out
		}
		//compute calibration data
		whiteaverage.huem /= numwhitepoints;
		whiteaverage.satm /= numwhitepoints;
		whiteaverage.valm /= numwhitepoints;
		printf("counting white points\n");
		for (int i = 0; i < numwhitepoints; i++)
		{
			whiteaverage.huev += sqr(whitepoints[i].hue - whiteaverage.huem);
			whiteaverage.satv += sqr(whitepoints[i].sat - whiteaverage.satm);
			whiteaverage.valv += sqr(whitepoints[i].val - whiteaverage.valm);
		}
		whiteaverage.huev /= (numwhitepoints-1);
		whiteaverage.satv /= (numwhitepoints-1);
		whiteaverage.valv /= (numwhitepoints-1);
		huen = whiteaverage.huem;		//nominal
		huet = 1*sqrt(whiteaverage.huev);		//tolerance
		satn = whiteaverage.satm;
		satt = 1*sqrt(whiteaverage.satv);
		valn = whiteaverage.valm;
		valt = 1*sqrt(whiteaverage.valv);
		huemax = whiteaverage.huem + huetolfactor*sqrt(whiteaverage.huev);      //compute tolerance from factor of standard deviation
		if (huemax > 255)                                                       //adjust to within range (since 359 degs rolls over to 0)
			huemax -= 255;
		huemin = whiteaverage.huem - huetolfactor*sqrt(whiteaverage.huev);      //compute tolerance from factor of standard deviation
		if (huemin < 0)                                                         //adjust to within range (since 359 degs rolls over to 0)
			huemin += 255;
		hueinv = false;             //default regular; see below:
		if (huemin > huemax)        //since 359 degs and 0 degs are next to each other, huemin may be less than huemax
		{                           //if so, we must take note of this and use || instead of && in logical evaluations
			hueinv = true;          // Regular: 0|    o--------o    |359    //use &&
                                    // Inverse: 0|----o        o----|359    //use ||
		}
		satmax = whiteaverage.satm + sattolfactor*sqrt(whiteaverage.satv);      //compute tolerance from factor of standard deviation
		satmin = whiteaverage.satm - sattolfactor*sqrt(whiteaverage.satv);      //compute tolerance from factor of standard deviation
		valmax = whiteaverage.valm + valtolfactor*sqrt(whiteaverage.valv);      //compute tolerance from factor of standard deviation
		valmin = whiteaverage.valm - valtolfactor*sqrt(whiteaverage.valv);      //compute tolerance from factor of standard deviation
		//end calibration routine
#ifdef CALFILE_H        //exclude calibration routine if file is missing
		writeCalFile(huemin, huemax, satmin, satmax, valmin, valmax);
	}
#endif

    //print calibration data:
	printf("huemin: %f\nsatmax: %f\nvalmax: %f\n", huemin, satmax, valmax);
	printf("starting main program.\n");
	printf("Hue max: %f\tmin: %f\n", huemax, huemin);
	printf("Sat max: %f\tmin: %f\n", satmax, satmin);
	printf("Val max: %f\tmin: %f\n", valmax, valmin);
#if TIMER
    //TIMER is used to calculate average frame rate (times entire algorithm, divides by total number of frames)
    //Note: the first time through will take extra time to set up, so longer runs will provide more accurate frame rates
	DWORD start_time, current_time;
	start_time = GetTickCount();        //a windows-specific function to grab current time in milliseconds
#endif

	while(!escape)                  //a global variable to determine when to escape the algorithm
	{                                   //Note: without this, the thread will still stop, but it won't be as graceful
		//memset(temp2XYZimage, 0, sizeof(temp2XYZimage));
		WaitForSingleObject(NewImage, INFINITE);    //Wait INFINITE time for a NewImage event to be triggered (must be set by stereo thread)
                                                    //Timeouts should be added for error checking
		EnterCriticalSection(&RawDisparityMap);     //Critical sections are used instead of semaphores:
                                                    // 1) because they are faster (in Windows)
                                                    // 2) because data is copied from a global variable (big) to a local variable (also big)
                                                    //    and so big chunks of memory are not being dynamically allocated
                                                    //    (Multiprocessing instead of multithreading may allow for larger dynamic allocations,
                                                    //     or perhaps changing the default settings of the master process)
		//memset(temp2RGBimage, 0, sizeof(temp2RGBimage));		//pixels are sticking (???)
		memcpy(temp2RGBimage, RGBimage, sizeof(temp2RGBimage));     //copy from the global RGBimage to the local temp2RGBimage
		memcpy(temp2XYZimage, XYZimage, sizeof(temp2XYZimage));
		//memset(XYZimage, 0, sizeof(XYZimage));
		LeaveCriticalSection(&RawDisparityMap);     //be sure to leave teh critical section ASAP
		orig->imageData = ((char *)(&temp2RGBimage));       //set (OpenCV) IplImage orig to the raw RGB file from the Bumblebee
                                                            //data types and sizes are a match (see definition of orig)

		/* Test CV Stuff */
		// the same OpenCV ops as were used to determine calibration settings
		cvErode(orig, orig);
		cvMorphologyEx(orig, orig, NULL, NULL, CV_MOP_OPEN);
		cvCvtColor(orig, hsv, CV_BGR2HSV);
#if DISPLAY
		cvShowImage("Input", orig);
		//cvShowImage("HSV", hsv);
#endif

		double recentgrads[320] = {0};      //an array of recent gradient values used to compute inclines/declines
		float angletolerance;
		float ytolerancemin;                // if a valid region occurs below this height, it is an obstalce (hole)
		float ytolerancemax;                // if a valid region occurs above this height, it is an obstalce
		unsigned long rampdeclinecnt = 0;	// number of pixels thought to be driving down ramp
		for (int i = 0; i < 160; i++)
		{
			for (int j = 239; j >= 0; j--)
			{
				for (int side = -1; side <= 1; side+=2)		//reads like a book, from center to center, left first then right
				{
				    //now loops through the image systematically
				    //set temp structure to original pixel RGB values
					current.red = RCHAN(orig,side);
					current.green = GCHAN(orig,side);
					current.blue = BCHAN(orig,side);
					current.type = UNKNOWN;
					angletolerance = 0;
					BCHAN(img,side) = 0;

                    //////////////////////////////////////////////
                    //begin trying to determine if there is a ramp
					if (j < 239)			//will compare to previous value (+ 1 or more), therefore must be less than 239
					{
						if (temp2XYZimage[j][160+side*i].z >= 0)
						{

							int tempj;      //a temporary j value that points to values above the current j value
							for (tempj = j+1;       //tempj should start one above j, and increment accordingly
							 ((tempj <= 239) &&         //tempj must be less than or equal to 239
                              (temp2XYZimage[tempj][160+side*i].z < 0) &&       //z must be greater than 0 (otherwise, error)
							  (temp2XYZimage[tempj][160+side*i].xz - temp2XYZimage[j][160+side*i].xz < .1));        //do not compare two points which
							  tempj++);                                                                             //are more than 0.1 meters apart

							if (tempj <= 239)
							{
							    //grab these values to compare slope:
								float tempgrady2 = temp2XYZimage[j][160+side*i].y;
								float tempgrady1 = temp2XYZimage[tempj][160+side*i].y;
								float tempgradxz2 = temp2XYZimage[j][160+side*i].xz;
								float tempgradxz1 = temp2XYZimage[tempj][160+side*i].xz;
								//m = (y2 - y1) / (x2 - x1)
								float tempgrad = (tempgrady2-tempgrady1)/(tempgradxz2-tempgradxz1);
								//average the current tempgrad with the average of all (320) the previous tempgrads (recentgrads) FOR THE CURRENT PIXEL
								recentgrads[160+side*i] = (recentgrads[160+side*i]+tempgrad)/2;
								//make it positive
								if (recentgrads[160+side*i] < 0)
                                    recentgrads[160+side*i] *= -1.0;
                                //if the current value for recentgrads does not exceed the allowable MAXGRAD
								if (recentgrads[160+side*i] <= MAXGRAD)
								{
								    //set the angle tolerance to the current recentgrads value
								    //Note: angle tolerance is a reference parameter which accounts for the current gradient for the region
									angletolerance = recentgrads[160+side*i];
								}
							}
						}
					}
					angletolerance *= temp2XYZimage[j][160+side*i].xz;



					if (angletolerance < 0)	/* Slope down */
					{
						ytolerancemin = CAMH-tan(CAMANGTOL*TORAD)+angletolerance-CAMHTOL;
						ytolerancemax = CAMH+tan(CAMANGTOL*TORAD)/*-angletolerance*/+CAMHTOL+CAMHTOLDOWN;
					}
					else					/* Slope up or no slope*/
					{
						ytolerancemin = CAMH-tan(CAMANGTOL*TORAD)-angletolerance-CAMHTOL;
						ytolerancemax = CAMH+tan(CAMANGTOL*TORAD)/*+angletolerance*/+CAMHTOL+CAMHTOLDOWN;
					}
/*
						ytolerancemin = CAMH-tan(CAMANGTOL*TORAD)-angletolerance-CAMHTOL;
						ytolerancemax = CAMH+tan(CAMANGTOL*TORAD)+angletolerance+CAMHTOL+CAMHTOLDOWN;
*/

                    ////////////////////////////////////////////
                    //end trying to determine if there is a ramp ???????????????

					if ((temp2XYZimage[j][160+side*i].z < 0) || (temp2XYZimage[j][160+side*i].z > 35) || (temp2XYZimage[j][160+side*i].x > 1.5) || (temp2XYZimage[j][160+side*i].x < -1.5))
					{
						/* Invalid */
						current.type = UNKNOWN;
						RCHAN(img,side) = 0;
						GCHAN(img,side) = 0;
						BCHAN(img,side) = 0;
					}
					else if ((temp2XYZimage[j][160+side*i].y < ytolerancemax) && (temp2XYZimage[j][160+side*i].y > ytolerancemin)) //?add in max z distance??
					{
						if (temp2XYZimage[j][160+side*i].y > ytolerancemin) // this pixel is part of a declining ramp
							rampdeclinecnt++;

						current.type = CLEAR;	//will be modified later if a line is found
						/* Drivable */
						//get_pixel_extrema(&current);
						/* Color Testing */
						//current.index = (MAXV-MEDV < primarysat) ? MAXI+MEDI : MAXI;

						/*
						int whiteness = 255;
						if (VCHAN(hsv, side))
							whiteness = (int)(SCHAN(hsv, side) - VCHAN(hsv, side));
						whiteness = (whiteness > 255) ? 255 : whiteness;
						*/


						// Look for hue values in regions that may wrap around the spectrum; huepass indicates it meets tolerance criteria
						if ((hueinv) && ((HCHAN(hsv, side) > huemin) || (HCHAN(hsv, side) < huemax)))
							huepass = true;
						else if ((!hueinv) && ((HCHAN(hsv, side) > huemin) && (HCHAN(hsv, side) < huemax)))
							huepass = true;
						else
							huepass = false;

						if ((huepass) && (SCHAN(hsv, side) > satmin) && (SCHAN(hsv, side) <= satmax) && (VCHAN(hsv, side) > valmin))
						{
							/* White */
							current.index = WHITE;
							RCHAN(img,side) = (lineoutcolor & RED) ? 255 : 0;
							GCHAN(img,side) = (lineoutcolor & GREEN) ? 255 : 0;
							BCHAN(img,side) = (lineoutcolor & BLUE) ? 255 : 0;
						}
						else if (showcolors[current.index])
						{
							/* CLEAR */
							current.type = CLEAR;
							RCHAN(img,side) = (whiteoutcolor & RED) ? 255 : 0;
							GCHAN(img,side) = (whiteoutcolor & GREEN) ? 255 : 0;
							BCHAN(img,side) = (whiteoutcolor & BLUE) ? 255 : 0;
						}
						else
						{
							/* UNKNOWN */
							current.type = UNKNOWN;
							RCHAN(img,side) = 0;
							GCHAN(img,side) = 0;
							BCHAN(img,side) = 0;
						}
					}
					else
					{
						/* Obstacle */
						current.type = OBSTACLE;
						RCHAN(img,side) = (obstaclecolor & RED) ? 255 : 0;
						GCHAN(img,side) = (obstaclecolor & GREEN) ? 255 : 0;
						BCHAN(img,side) = (obstaclecolor & BLUE) ? 255 : 0;
					}
				}
			}
		}
		////printf("Average Slope: %f\n", (GetAvgSlope(LEFT, img)+GetAvgSlope(RIGHT, img))/2);
		cvDilate(img, img);
		//cvMorphologyEx(img, img, NULL, NULL, CV_MOP_CLOSE);
		cvMorphologyEx(img, img, NULL, NULL, CV_MOP_OPEN);
		////cvErode(img, img);

#if DISPLAY
		cvSplit(img, blue, NULL, NULL, NULL);
#endif
		//col LineAvg[6] = {0};

		/* send to rich */
		CvPoint ltemppts[38400], rtemppts[38400];  //arrays to be used in OpenCV line detection
		int ltempcnt = 0, rtempcnt = 0;				//number of pixels to be considered in the above arrays
		for (int i = 0; i < 160; i++)
		{
			for (int j = 239; j >= 0; j--)
			{
				for (int side = -1; side <= 1; side+=2)		//from the bottom up, inside to outside
				{
				    //printf("i: %d,\tj: %d\t, side: %d\n", i, j, side);
				    //printf("7.1\n");
                    float rangle = atan2(temp2XYZimage[j][160+side*i].x, temp2XYZimage[j][160+side*i].z)*TODEG;
                    //printf("7.2\n");
                    //printf("rangle: %f\n", rangle);
                    int rindex = (int)(rangle/5.0);
                    //printf("rindex: %d\n", rindex);
                    //printf("7.3\n");
                    //printf("7.4\n");
					if ((temp2XYZimage[j][160+side*i].z >= 0.7) && (temp2XYZimage[j][160+side*i].z <= 3.1) && rindex >= -7 && rindex <= 6)	//within valid range
					{
					    //dindex stores the array value for the depth of the current pixel
					    float rdepth = temp2XYZimage[j][160+side*i].z*MTOIN;
					    int dindex = 0;
                        for (float depth = 0; rdepth > depth; dindex++)
                        {
                            depth += Circledindex[dindex];
                        }
                        //printf("dindex: %d\tlindex: %d\tx: %f\n", dindex, lindex, temp2XYZimage[j][160+side*i].x);
                        if (GCHAN(img,side) && RCHAN(img,side) && !BCHAN(img,side))
                        {	/* YELLOW */
                            FromVisionCircle[dindex][rindex].YelCount++;
                            FromVisionCircle[dindex][rindex].TotCount++;
                        }
                        else if (BCHAN(img,side) && !RCHAN(img,side))
                        {	/* CYAN or BLUE (WHITE) */
                            //if (((temp2XYZimage[j][160+side*i].x > -1.5) && (temp2XYZimage[j][160+side*i].x < -0.5)) || ((temp2XYZimage[j][160+side*i].x > 0.5) && (temp2XYZimage[j][160+side*i].x < 1.5)) && (temp2XYZimage[j][160+side*i].z < 2))
                            {
                                if ((side == LEFT) && (ltempcnt < 38400))
                                {
                                    ltemppts[ltempcnt].x = cvRound(temp2XYZimage[j][160+side*i].z*1000);
                                    ltemppts[ltempcnt].y = cvRound(temp2XYZimage[j][160+side*i].x*1000);
                                    ltempcnt++;
                                    //printf("ltempcnt: %d\n", ltempcnt);
                                }
                                else if ((side == RIGHT) && (rtempcnt < 38400))
                                {
                                    rtemppts[rtempcnt].x = cvRound(temp2XYZimage[j][160+side*i].z*1000);
                                    rtemppts[rtempcnt].y = cvRound(temp2XYZimage[j][160+side*i].x*1000);
                                    rtempcnt++;
                                    //printf("rtempcnt: %d\n", rtempcnt);
                                }
                            }
                            FromVisionCircle[dindex][rindex].WhtCount++;
                            FromVisionCircle[dindex][rindex].TotCount++;
                        }
                        else if (GCHAN(img,side) && !BCHAN(img,side) && !RCHAN(img,side))
                        {	/* GREEN */
                            FromVisionCircle[dindex][rindex].GrnCount++;
                            FromVisionCircle[dindex][rindex].TotCount++;
                        }
                        else if (RCHAN(img,side) && !BCHAN(img,side) && !GCHAN(img,side))
                        {	/* RED */
                            FromVisionCircle[dindex][rindex].RedCount++;
                            //printf("rindex: %d\tRedCount: %d\n", rindex, FromVisionCircle[dindex][rindex].RedCount);
                            FromVisionCircle[dindex][rindex].TotCount++;
                        }
					}
				}
			}
		}

        ///////////////////////////////////
        //Line Fitting Stuff Begins Here...
        ///////////////////////////////////

		//OpenCV Line Fit
		float lline[4], rline[4];
		//getline(ltemppts, ltempcnt, lline);
		//getline(rtemppts, rtempcnt, rline);

		//CvMat pointMat = cvMat(1, tempcnt, CV_32SC2, ltemppts);
		CvMat lpointMat = cvMat(1, ltempcnt, CV_32SC2, ltemppts);
		CvMat rpointMat = cvMat(1, rtempcnt, CV_32SC2, rtemppts);

		float langle = 0, lgoodness = 0, rangle = 0, rgoodness = 0;


		//printf("ltempcnt: %d\n", ltempcnt);
		if (ltempcnt >= 1)
		{
			//printf("L goodness: %f\n", getline(ltemppts, ltempcnt, lline));

			cvFitLine(&lpointMat, CV_DIST_L2, 1, 0.001, 0.001, lline);
			langle = atan(lline[1]/lline[0])*TODEG;
			//printf("L angle: %f\n", langle);
			//printf("L x-int: %f\n", (lline[2]-lline[3]/(lline[1]/lline[0]))/1000);
			lgoodness = checkgoodness(ltemppts, ltempcnt, lline);
			//printf("L goodness: %f\n", lgoodness);

		}
		else
			printf("L angle: invalid\n");

		//printf("rtempcnt: %d\n", rtempcnt);
		if (rtempcnt >= 1)
		{
			//printf("R goodness: %f\n", getline(rtemppts, rtempcnt, rline));

			cvFitLine(&rpointMat, CV_DIST_L2, 1, 0.001, 0.001, rline);
			rangle = atan(rline[1]/rline[0])*TODEG;
			//printf("R angle: %f\n", rangle);
			//printf("R x-int: %f\n", (rline[2]-rline[3]/(rline[1]/rline[0]))/1000);
			rgoodness = checkgoodness(rtemppts, rtempcnt, rline);
			//printf("R goodness: %f\n", rgoodness);

		}
		else
			printf("R angle: invalid\n");

        float laneAngle = 0;
        if (lgoodness+rgoodness)
        {
            laneAngle = (langle*lgoodness+rangle*rgoodness)/(lgoodness+rgoodness);
        }
        else
        {
            printf("no goodness vals\n");
        }
		//float laneAngle = (lgoodness+rgoodness) ? (langle*lgoodness+rangle*rgoodness)/(lgoodness+rgoodness) : 0;
		printf("laneAngle: %f\n", laneAngle);
		float laneAngleQuality = atan((lgoodness+rgoodness)/2.0)*TODEG*10.0/9.0;
		printf("laneAngleQuality: %f\n", laneAngleQuality);

		float sumAngleQuality = laneAngleQuality + avgLaneAngleQuality;
		if (abs(avgLaneAngleQuality) < 0.05)
		{
		    avgLaneAngle = laneAngle;
		    avgLaneAngleQuality = laneAngleQuality;
		}
		else if (abs(sumAngleQuality) > 0.05)
        {
            avgLaneAngle = (laneAngle*laneAngleQuality + avgLaneAngle*avgLaneAngleQuality)/sumAngleQuality;
            avgLaneAngleQuality = (sqr(laneAngleQuality)+sqr(avgLaneAngleQuality))/sumAngleQuality;
        }

        //Post FromVisionCircle globally (to GlobalCircle) here (through Mutex or C.S.)
        EnterCriticalSection(&PostDecisionMap);
        globalLaneAngle = avgLaneAngle;
        globalLaneAngleQuality = avgLaneAngleQuality;
        for (int j = 0; j < 10; j++)
        {
            for (int i = -7; i < 7; i++)
            {

                GlobalCircle[j][i].GrnCount = FromVisionCircle[j][i].GrnCount;
                FromVisionCircle[j][i].GrnCount = 0;
                //printf("lineGrnCnt: %d\n", GlobalCircle[j][i].GrnCount);
                GlobalCircle[j][i].YelCount = FromVisionCircle[j][i].YelCount;
                FromVisionCircle[j][i].YelCount = 0;
                //printf("lineYelCnt: %d\n", GlobalCircle[j][i].YelCount);
                GlobalCircle[j][i].WhtCount = FromVisionCircle[j][i].WhtCount;
                FromVisionCircle[j][i].WhtCount = 0;
                //printf("lineWhtCnt: %d\n", GlobalCircle[j][i].WhtCount);
                GlobalCircle[j][i].RedCount = FromVisionCircle[j][i].RedCount;
                FromVisionCircle[j][i].RedCount = 0;
                //printf("lineRedCnt: %d\n", FromVisionCircle[j][i].RedCount);
                GlobalCircle[j][i].TotCount = FromVisionCircle[j][i].TotCount;
                FromVisionCircle[j][i].TotCount = 0;
                //printf("lineTotCnt: %d\n", GlobalCircle[j][i].TotCount);
            }
        }
        LeaveCriticalSection(&PostDecisionMap);


        //Set Event (??)
        SetEvent(NewMap);
        //printf("Setting NewMap\n");

		//laneAngleQuality ranges
		//printf("%f\n", 1.34567890);
		//printf("%f,%f\n", laneAngle, laneAngleQuality);
#if DISPLAY
		printf("Lane Angle: %f\tGoodness: %f\n", (langle*lgoodness+rangle*rgoodness)/(lgoodness+rgoodness), (lgoodness+rgoodness)/2);
#endif

		if (rampdeclinecnt > RAMPDECLINETHRESH)
		{
#if DISPLAY
			printf("RAMP!!!\n");	/* Send Bit to say we're on a ramp, YAY! */
#endif
		}

		//printf("Updating Output, Blue windows\n");

#if DISPLAY
		//cvShowImage("Input", orig);
		cvShowImage("Output", img);
		cvShowImage("Blue", blue);
#endif
#if OUTPUT == VIDEO
		cvWriteFrame(writer,img);
#endif
		framecount++;
		cvWaitKey(30);
		//printf("Looping line.h\n");
	}
		printf("Total Frames: %ld\n", framecount);
#if TIMER
		current_time = GetTickCount();
		printf("Total Time (ms): %ld\n", current_time-start_time);
		printf("Frames per second: %f\n", framecount*1.0/(current_time-start_time)*1000);
		//mdelay(1000);
#endif
	//cvWaitKey();
#if DISPLAY
	cvDestroyWindow("Input");
	cvDestroyWindow("Output");
	cvDestroyWindow("Blue");
	cvReleaseImage(&orig);
	cvReleaseImage(&img);
	cvReleaseImage(&blue);
#endif
#if OUTPUT == VIDEO
	cvReleaseVideoWriter(&writer);
#endif
	return 0;
}

inline int get_pixel_extrema(pixel *current)		//determines max, min and median channels for each cell
{
	if (current->red < current->green)				//check an initial condition (arbitrary)
	{
		current->min = RED;
		current->med = BLUE;
		current->max = GREEN;
		if (current->blue < *((uchar*)current+current->min))		//adjust if necessary by swaping
		{
			current->min = BLUE;
			current->med = RED;
		}
		else if (current->blue > *((uchar*)current+current->max))	//adjust if necessary by swaping
		{
			current->med = GREEN;
			current->max = BLUE;
		}
	}
	else
	{
		current->min = GREEN;
		current->med = BLUE;
		current->max = RED;
		if (current->blue < *((uchar*)current+current->min))		//adjust if necessary by swaping
		{
			current->min = BLUE;
			current->med = GREEN;
		}
		else if (current->blue > *((uchar*)current+current->max))	//adjust if necessary by swaping
		{
			current->med = RED;
			current->max = BLUE;
		}
	}
	return 0;
}

float checkgoodness(CvPoint* points, int length, float* line)
{
	/* all the existing stuff here */
	float factor = 1;
	float m = line[1]/line[0];
	float x0 = line[2];
	float y0 = line[3];
	float ymath;
	float variance = 0, stdev, goodness;
	for (int i = 0; i < length; i++)
	{
		ymath = m*(points[i].x - x0) + y0;
		variance += sqr(ymath - points[i].y);
	}
	variance /= (length-1);
	stdev = sqrt(variance);
	if (stdev)
	{
		goodness = length/stdev*factor;	//divide by variance or stdev?
		printf("Goodness: %f\tLength: %d\nStdev: %f\n", goodness, length, stdev);
		//goodness /= /*max_goodness;*/
		//printf("Goodness: %f\n", goodness);
	}
	else
	{
		goodness = 0.0;
#if DISPLAY
		printf("Error; variance 0\n");
#endif
	}
	return goodness;
}

#endif
