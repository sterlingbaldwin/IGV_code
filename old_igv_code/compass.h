#pragma once
#ifndef COMPASS_H_INCLUDED
#define COMPASS_H_INCLUDED

///     Example of receiving current compass data:
///         WaitForSingleObject(mCompassData, INFINITE);
///         float curHeading = CompassData;
///         ReleaseMutex(mCompassData);

//#include "globals.h"
#include "serial.h"
#include <cstring>

#define COMPASSDEBUG 0

void compassstuff();

extern bool escape;

//A Windows function used for multitasking
DWORD WINAPI CompassLoop(LPVOID param);



#endif // COMPASS_H_INCLUDED
