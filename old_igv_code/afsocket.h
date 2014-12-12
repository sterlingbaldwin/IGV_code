#pragma once
#ifndef AFSOCKET_H_INCLUDED
#define AFSOCKET_H_INCLUDED

#define SPDSF 1000       //speed factor; max speed (in encoder counts per second) is 96*SPDSF
#define TURNF 10000

#include "globals.h"


///     To send a motion command via Ethernet
///         WaitForSingleObject(mVector, INFINITE);
///         //set TermString or speed and dir
///         //if TermString is not empty, then the contents of TermString are sent
///         //else, the vector (long int speed, long int dir) is sent
///         ReleaseMutex(mVector);
///         SetEvent(hNewMotion);
///         WaitForSingleObject(hMotionDone, INFINITE);
///
///     Example of how to grab Encoder Position for Y Axis
///         //Allocate a local char array
///         char locRecv[64] = {0};
///         WaitForSingleObject(mRecvPtr, INFINITE);
///         RecvPtr = locRecv;
///         ReleaseMutex(mRecvPtr);
///         WaitForSingleObject(mTermString, INFINITE);
///         sprintf(TermString, "TPY;");
///         ReleaseMutex(mTermString);
///         SetEvent(hNewMotion);
///         //Wait until the TPY command has been sent
///         WaitForSingleObject(hMotionDone, INFINITE);
///         //And then until a response has been received
///         WaitForSingleObject(mRecvPtr, INFINITE);
///         XCurrent = atoi(locRecv);
///         ReleaseMutex(mRecvPtr);

//Necessary (with Windows Vista, at least); unsure why
#define _WIN32_WINNT 0x501

//#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <stdlib.h>
#include <stdio.h>
//#include "globals.h"

#define MIN(X, Y)  ((X) < (Y) ? (X) : (Y))

// Need to link with Ws2_32.lib, Mswsock.lib, and Advapi32.lib
// In Code::Blocks: Project->Build options->[project name]->Linker settings->Link libraries
// Add libraries from Microsoft SDKs directory (i.e. C:\Program Files\Microsoft SDKs\v6.0A\WS-32.Lib)

//Debug and test features
#define ETHERNET 1          //1 turns on Ethernet output, 0 redirects Ethernet output only to console
#define ETHERNETDEBUG 0     //Turns on extra printf() statements used to debug Ethernet code
#define MOTIONDEBUG 0       //Turns on extra printf() statements related specifically to motion
#define RECVDEBUG 0         //Turns on extra printf() statements related specifically to receiving data
//#define MANUALMODETEST 0

#define DEFAULT_BUFLEN 512
#define DEFAULT_PORT "27015"

int socketstuff();

//A Windows function used for multitasking
DWORD WINAPI MotionLoop(LPVOID param);

#endif // AFSOCKET_H_INCLUDED
