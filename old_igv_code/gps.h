#pragma once
#ifndef GPS_H_INCLUDED
#define GPS_H_INCLUDED

///     Example of receiving curGPSvec data:
///         WaitForSingleObject(mGPSData, INFINITE);
///         ThetaTarget = curGPSvec.direction;
///         ThetaCurrent = curGPSvec.curdir;
///         DistanceToTar = curGPSvec.distance;
///         ReleaseMutex(mGPSData);

#include <windows.h>
#include "serial.h"
#include <cmath>
#include <vector>
#include <fstream>

#define PI 3.14159265359
#define TODEG 180/PI
#define TORAD PI/180

extern char mode;
extern bool escape;

#define MANUALMODE 00000000
#define AUTOMODE 00000001
#define GPSMODE 00000010
extern bool stopmain;

//#include "globals.h"

using namespace std;

#define GPSDEBUG 0
#define GPSINPUT 0      //currently not used
#define FORCEGPSINPUT 0

struct smGPSDatum
{
    double latitude;
    double longitude;
};

struct GPSDatum
{
    double latitude;
    double longitude;
    float velocity;
    float heading;
    float magvariation;
    char time[7];
    char date[7];
    char checksum[5];
    char status;
    char latitudeNS;
    char longitudeEW;
    char magvariationEW;
};

struct GPSvector
{
    float distance;
    float direction;
    float curdir;
    //magvariation (to adjust compass in the receiving thread)
    float magvariation;
    //GPSDatum* curGPS;
};

void GPSstuff(char *filename = "test.csv");

GPSvector getvector(GPSDatum* curGPS, smGPSDatum* otherGPS);

//A Windows function used for multitasking
DWORD WINAPI GPSLoop(LPVOID param);

bool saveGPStar(smGPSDatum* sdatum, fstream *fio);

bool loadGPStar(smGPSDatum* ldatum, fstream *fio);

#endif // GPS_H_INCLUDED
