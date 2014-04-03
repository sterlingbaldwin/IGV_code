#ifndef CVTEST_CPP_INCLUDED
#define CVTEST_CPP_INCLUDED

#warning "including CVTest.cpp"

#include "globals.h"

#include <iostream>
#include <stdio.h>
#include <stdlib.h>

//#include "circle.h"
#include "afsocket.h"
//#include "globals.h"
#include "decide.h"

#include "gps.h"
#include "compass.h"
#include "stereo.h"
#include "calfile.h"
#include "line.h"

/* ********************************** */
/* Begin Global Variable Declarations */
/* ********************************** */
char mode = MANUALMODE;

float MaxDens = .5;
float MaxWhiteDensity = 15;//15;

/* ******************************************************** */
/* Critical Section, Semaphore and Generic IPC Declarations */
/* ******************************************************** */

CRITICAL_SECTION RawDisparityMap;       //used between stereo --> line processes, temp1(RGB/XYZ)image -> (RGB/XYZ)image -> temp2(RGB/XYZ)image
//CRITICAL_SECTION GrabSemaMap;
CRITICAL_SECTION SetCommData;
CRITICAL_SECTION PostDecisionMap;       //used between line --> decision processes, FromVisionCircle -> GlobalCircle -> ToDecisionCircle
CRITICAL_SECTION GetDecisionMap;         //used between line --> decision processes, second half

/* ******************************************* */
/* Necessary Windows Multithreading Structures */
/* ******************************************* */
HANDLE NewImage, NewMap, NewCommData, EthernetUp;

DWORD tidVision, tidStereo, tidDecide;
HANDLE hVision = 0, hStereo = 0, hDecide = 0;

/* Logic Control Variables */

bool LineEscape;// = 0;
bool stopmain;// = false;
bool escape;// = false;

//const int mapwidth[16] = {7, 9, 9, 11, 13, 13, 15, 15, 17, 19, 19, 21, 23, 23, 25, 27};

// Array to hold map data
//VisHeader BrianMap[266] = {0};
//VisHeader BigSemaMap[266] = {0};
Circle<VisHeader> FromVisionCircle[11];
Circle<VisHeader> GlobalCircle[10];
Circle<VisHeader> ToDecisionCircle[10];
const int Circledindex[10] = {6, 6, 6, 6, 6, 12, 12, 24, 48, 96};

// Array to hold ______
//char temp1BitMap[283] = {0};
//char SemaMap[283] = {0};
//char temp2BitMap[283] = {0};

// Array to hold raw RGB and XYZ image
// --> From stereo.h to line.h
// 320 x 320 array was necessary b/c PTGrey inverted x and y dims somewhere in their code
// This has been addressed in the code which is used to access it
rgb RGBimage[320][320];	/*right was 240*/               //global copy
xyz XYZimage[320][320];	/*right was 240*/               //global copy

rgb temp1RGBimage[240][320];	/*right was 240*/       //used by stereo process
xyz temp1XYZimage[240][320];	/*right was 240*/       //used by stereo process
rgb temp2RGBimage[240][320];	/*right was 240*/       //used by line process
xyz temp2XYZimage[240][320];	/*right was 240*/       //used by line process

char CommData[1024];
char temp2CommData[1024];

float globalLaneAngle;
float globalLaneAngleQuality;

/* ******************************************** */
/* Begin GPS Global Definitions                 */
/* ******************************************** */

CommDevice *GPS = 0;

//Event to signal that new GPS data has been received
HANDLE hNewGPSData;

//Event to signal that the current GPS point should be stored
HANDLE hStoreGPS;

//Mutex to transfer curGPSvec
HANDLE mGPSData;
//GPS Data to be transfered
GPSvector curGPSvec;

//Mutex to change StoreGPS
HANDLE mStoreGPS;
bool StoreGPS;

//Handle to the GPS thread
HANDLE hGPS;

//GPS Thread ID
DWORD tidGPS;

//Bool if current GPS is hit
char curGPSHit;

//Event to signal when all GPS points have been loaded
HANDLE hTargetsLoaded;

/* ******************************************** */
/* Begin Compass Global Definitions             */
/* ******************************************** */

//a pointer to Compass class (instantiated in Thread 0)
CommDevice *Compass = 0;

//A mutex to protect float CompassData
HANDLE mCompassData;
float CompassData;

//A handle to the compass thread
HANDLE hCompass;

//A thread ID for the compass thread
DWORD tidCompass;

HANDLE hMotion;

DWORD tidMotion;

//////////////////
// Events
//////////////////

//An event to flag when new commands should be sent via Ethernet
//if hNewMotion is set and TermString is not empty, then the contents of TermString are sent
//else, the vector (long int speed, long int dir) is sent
HANDLE hNewMotion;

//An event to flag when the current command has been sent via Ethernet
HANDLE hMotionDone;

//An event to flag when the vehicle has been commanded to stop
//This is primarily used to ensure the vehicle is stopping before exiting the program
HANDLE hStopped;

//An event to flag when data has been received via Ethernet
HANDLE hNewRecv;

//////////////////
// Semaphores
//////////////////

//A mutex to protect char TermString[64];
//if hNewMotion is set and TermString is not empty, then the contents of TermString are sent
//else, the vector (long int speed, long int dir) is sent
HANDLE mTermString;
char TermString[64];

//A mutex to protect long int speed and long int dir
//if hNewMotion is set and TermString is not empty, then the contents of TermString are sent
//else, the vector (long int speed, long int dir) are sent
HANDLE mVector;
long int speed;
long int dir;

//A mutex to protect char* RecvPtr;
HANDLE mRecvPtr;
char* RecvPtr;

/* ******************** */
/* End Global Variables */
/* ******************** */

DWORD WINAPI ProcessVision(LPVOID param)
{
     //call function or include here
    FindLines();
    return 0;
}

DWORD WINAPI MakeDecision(LPVOID param)
{
     //call function or include here
    while (!escape)
    {
        doGo();
        printf("starting doGo again.\n");
    }
    return 0;
}

DWORD WINAPI StereoProcessing(LPVOID param)
{
     //call function or include here
    RunStereo();
    return 0;
}

void StartThreads(int RunThreads)
{
	//printf("%d\n", RunThreads);
	InitializeCriticalSection(&RawDisparityMap);
	//InitializeCriticalSection(&GrabSemaMap);
	InitializeCriticalSection(&SetCommData);
	InitializeCriticalSection(&PostDecisionMap);
	InitializeCriticalSection(&GetDecisionMap);
	NewImage = CreateEvent(NULL, FALSE, FALSE, NULL);
	NewMap = CreateEvent(NULL, FALSE, FALSE, NULL);
	NewCommData = CreateEvent(NULL, FALSE, FALSE, NULL);
	hNewMotion = CreateEvent(NULL, FALSE, FALSE, NULL);
	hMotionDone = CreateEvent(NULL, FALSE, FALSE, NULL);
	hStopped = CreateEvent(NULL, FALSE, FALSE, NULL);
	EthernetUp = CreateEvent(NULL, TRUE, FALSE, NULL);      //must be manually reset; currently not used
	hNewGPSData = CreateEvent(NULL, FALSE, FALSE, NULL);        //an event to let the decide code know that new GPS data has been
                                                                //processed and turned into a vector
	hStoreGPS = CreateEvent(NULL, FALSE, FALSE, NULL);          //an event to trigger the GPS thread to store the current position
	hTargetsLoaded = CreateEvent(NULL, FALSE, FALSE, NULL);     //to let everything know the GPS targets have been loaded
	mTermString = CreateMutex(NULL, FALSE, NULL);               //TermString is used to send terminal strings to motion (afsocket) *
	mRecvPtr = CreateMutex(NULL, FALSE, NULL);                  //RecvPtr is used to send terminal strings to motion (afsocket) *
	mVector = CreateMutex(NULL, FALSE, NULL);                   //vector (and mVector) is used to communicate btwn. decide and motion (afsocket)
	mGPSData = CreateMutex(NULL, FALSE, NULL);                  //a mutex to protect the new GPS data/(vector); associated w/ hNewGPSData
	mStoreGPS = CreateMutex(NULL, FALSE, NULL);                 //a mutex to protect the StoreGPS boolean; associated w/ hStoreGPS

    //Event to signal that the current GPS point should be stored
    hStoreGPS = CreateEvent(NULL, FALSE, FALSE, NULL);

	//Event to signal when all GPS points have been loaded
    hTargetsLoaded = CreateEvent(NULL, FALSE, FALSE, NULL);
    if ((RunThreads & STEREOID) && (hStereo == 0))
    {
		//printf("%d\n", RunThreads);
		printf("Starting hStereo.\n");
        hStereo = (HANDLE) CreateThread(NULL, 0, StereoProcessing, 0, CREATE_SUSPENDED, &tidStereo);
		SetThreadPriority(hStereo, THREAD_PRIORITY_NORMAL);
		/* Add Processor Affinities Here? */
		ResumeThread(hStereo);
	}
	if ((RunThreads & VISIONID) && (hVision == 0))
	{
		//printf("%d\n", RunThreads);
		printf("Starting hLines.\n");
		hVision = (HANDLE) CreateThread(NULL, 0, ProcessVision, 0, CREATE_SUSPENDED, &tidVision);
		SetThreadPriority(hVision, THREAD_PRIORITY_NORMAL);
		/* Add Processor Affinities Here? */
		ResumeThread(hVision);
	}

	if ((RunThreads & DECIDEID) && (hDecide == 0))
	{
		//printf("%d\n", RunThreads);
		printf("Starting hDecide.\n");
		hDecide = (HANDLE) CreateThread(NULL, 0, MakeDecision, 0, CREATE_SUSPENDED, &tidDecide);
		SetThreadPriority(hDecide, THREAD_PRIORITY_NORMAL);
		/* Add Processor Affinities Here? */
		ResumeThread(hDecide);
	}
	if ((RunThreads & GPSID) && (hGPS == 0))
	{
	    printf("Starting hGPS.\n");
	    hGPS = (HANDLE) CreateThread(NULL, 0, GPSLoop, 0, CREATE_SUSPENDED, &tidGPS);
	    SetThreadPriority(hGPS, THREAD_PRIORITY_NORMAL);
	    /* Add Processor Affinities Here? */
	    ResumeThread(hGPS);
	}
	#warning Scott is gay.
    if ((RunThreads & COMPASSID) && (hCompass == 0))
	{
	    printf("Starting hCompass.\n");
	    hCompass = (HANDLE) CreateThread(NULL, 0, CompassLoop, 0, CREATE_SUSPENDED, &tidCompass);
	    SetThreadPriority(hCompass, THREAD_PRIORITY_NORMAL);
	    /* Add Processor Affinities Here? */
	    ResumeThread(hCompass);
	}
	if ((RunThreads & MOTIONID) && (hMotion == 0))
	{
	    printf("Starting hMotion.\n");
	    hMotion = (HANDLE) CreateThread(NULL, 0, MotionLoop, 0, CREATE_SUSPENDED, &tidMotion);
	    SetThreadPriority(hMotion, THREAD_PRIORITY_NORMAL);
	    /* Add Processor Affinities Here? */
	    ResumeThread(hMotion);
	}
}

void StopThreads()
{
	if (hStereo)
		CloseHandle(hStereo);
	if (hVision)
		CloseHandle(hVision);
	if (hDecide)
		CloseHandle(hDecide);
    if (hGPS)
        CloseHandle(hGPS);
    if (hCompass)
        CloseHandle(hCompass);
    if (hMotion)
        CloseHandle(hMotion);
	DeleteCriticalSection(&RawDisparityMap);
	DeleteCriticalSection(&SetCommData);
	DeleteCriticalSection(&PostDecisionMap);
	DeleteCriticalSection(&GetDecisionMap);
}

int main()
{
	//4/27/13
	StartThreads(STEREOID | VISIONID | DECIDEID | MOTIONID);
	//StartThreads(VISIONID | STEREOID );
	char defmode = mode;
	long int prevspeed = 0, prevdir = 0, locspeed = 0, locdir = 0;
	while(!GetAsyncKeyState(VK_ESCAPE) && !stopmain)
	{
	    if (GetKeyState(VK_CAPITAL) & 0x01)
	    {
	        //CAPS LOCK is on
	        //run manual
	        printf("Manual Mode\n");
	        mode = MANUALMODE;
            do
            {
                if (GetAsyncKeyState('W') && !GetAsyncKeyState('S'))
                {
                    locspeed = 6;
                }
                else if (GetAsyncKeyState('S') && !GetAsyncKeyState('W'))
                {
                    locspeed = -4;
                }
                else
                {
                    locspeed = 0;
                }
                if (GetAsyncKeyState('A') && !GetAsyncKeyState('D'))
                {
                    locdir = 15;
                }
                else if (GetAsyncKeyState('D') && !GetAsyncKeyState('A'))
                {
                    locdir = -15;
                }
                else
                {
                    locdir = 0;
                }
                if (GetAsyncKeyState(VK_CONTROL))
                {
                    printf("Telling GPS thread to store cur GPS...\n");
                    WaitForSingleObject(mStoreGPS, INFINITE);
                    StoreGPS = true;
                    ReleaseMutex(mStoreGPS);
                    while(GetAsyncKeyState(VK_CONTROL));
                }
                //printf("speed: %f\ndir: %ld\n", speed, dir);
            } while (locspeed == prevspeed && locdir == prevdir);
            prevspeed = locspeed;
            prevdir = locdir;
            WaitForSingleObject(mVector, INFINITE);
            speed = locspeed;
            dir = locdir;
            //printf("dir: %f\n", dir);
            ReleaseMutex(mVector);
            SetEvent(hNewMotion);
	    }
	    else
	    {
	        //printf("Auto Mode\n");
            //CAPS LOCK is off
	        //reset to whatever the original mode was
	        if (mode != defmode)
	        {
	            mode = defmode;
	        }
	    }
    }
	printf("Stopping...\n");
    WaitForSingleObject(mVector, INFINITE);
    speed = 0;
    dir = 0;
    //printf("dir: %f\n", dir);
    ReleaseMutex(mVector);
    SetEvent(hNewMotion);
	escape = true;
	if (!stopmain)
	{
        while(GetAsyncKeyState(VK_ESCAPE));
	}
	StopThreads();
	return 0;
}

#endif
