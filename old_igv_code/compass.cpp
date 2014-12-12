#include "compass.h"

/* ******************************************** */
/* Begin Compass Global Definitions             */
/* ******************************************** */

//a pointer to Compass class (instantiated in Thread 0)
extern CommDevice *Compass;

//A mutex to protect float CompassData
extern HANDLE mCompassData;
extern float CompassData;

//A handle to the compass thread
extern HANDLE hCompass;

//A thread ID for the compass thread
extern DWORD tidCompass;

void compassstuff()
{
    float avgCompass = 0;
    bool firstpass = true;
    CommDevice Compass("COM8", 0, 9600, 16);
    #if COMPASSDEBUG
        printf("Compass successfully loaded...\n");
    #endif

    char templocComp[32] = {0};

    while (!escape)
    {
        char *tc = strchr(templocComp, 0);
        char *CompassToProcess = 0;

        /*DWORD evstatus;
        do
        {
            WaitCommEvent(Compass.hComm, &evstatus, NULL);
        } while (evstatus != EV_RXFLAG);
        */

        //Compass.Read(rawcompassdata/*, sizeof(rawcompassdata)*/);
        Compass.Read(tc);

        //char *CompassBegin = 0;
        int i;
        for (i = 32; i > 0 && tc[i] != '\n'; --i);
        if (tc[i] == '\n')
        {
            tc[i] = 0;
            for ( ; i > 0 && tc[i] != '\n'; --i);
            if (tc[i] == '\n')
            {
                CompassToProcess = &tc[i];
            }
        }

        if (CompassToProcess)
        {

            //Compass.Read(tc);
            if (firstpass)
            {
                avgCompass = atof(CompassToProcess);
                firstpass=false;
            }
            else
            {
                avgCompass = atof(CompassToProcess) + 0*avgCompass;
                avgCompass /= 1;
            }

            //Parse the compass data from rawcompassdata
            WaitForSingleObject(mCompassData, INFINITE);
            //CompassData = atof(rawcompassdata);
            CompassData = avgCompass;
            #if COMPASSDEBUG
                printf("CompassData: %f\n", CompassData);
            #endif
            //copy to global compass value here
            ReleaseMutex(mCompassData);
        }
        char tempstr[32];
        strcpy(tempstr, tc);
        memset(templocComp, 0, sizeof(templocComp));
        strcpy(templocComp, tempstr);
    }
}

DWORD WINAPI CompassLoop(LPVOID param)
{
    compassstuff();
    return 0;
}
