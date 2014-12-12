#include "gps.h"

/* ******************************************** */
/* Begin GPS Global Definitions                 */
/* ******************************************** */

extern CommDevice *GPS;

//Event to signal that new GPS data has been received
extern HANDLE hNewGPSData;

//Event to signal that the current GPS point should be stored
extern HANDLE hStoreGPS;

//Mutex to transfer curGPSvec
extern HANDLE mGPSData;
//GPS Data to be transfered
extern GPSvector curGPSvec;

//Mutex to change StoreGPS
extern HANDLE mStoreGPS;
extern bool StoreGPS;

//Handle to the GPS thread
extern HANDLE hGPS;

//GPS Thread ID
extern DWORD tidGPS;

//Bool if current GPS is hit
extern char curGPSHit;

//Event to signal when all GPS points have been loaded
extern HANDLE hTargetsLoaded;

///     Due to the nature of the program (being multithreaded), each serial buffer (FIFO) has an opportunity to fill up
///     faster than it can be read.  As a result, if the buffer were read and a simple atoi or parsing algorithm applied,
///     then it would read old data.  To fix this, the local copy of the receive buffer is scanned backwards.  This can
///     result in some values not being read (i.e. GPS update rate of 3 Hz instead of 5 Hz), but means that the GPS value
///     parsed is always as recent as possible.

void GPSstuff(char *filename)
{
    #if GPSDEBUG
        printf("GPS starting...\n");
    #endif
    //CommDevice GPS("COM1", 0, 115200, 128);
    GPS = new CommDevice("COM1", 0, 115200, 180);
    #if GPSDEBUG
        printf("GPS successfully started...\n");
    #endif
    GPS->Write("em,,/msg/nmea/RMC:.2\n");
    //fio << "Hello world" << endl;
    vector<smGPSDatum> GPStars;

    char templocGPSbuf[360];

    smGPSDatum tempsmGPS;

    char prevmode = mode;

    while (!escape)
    {
        //printf("Switching modes...\n");
        fstream fio(filename);
        if (!fio.is_open())
        {
            printf("Error: unable to open %s\n", filename);
        }

        //load the target GPSs
        if (mode == AUTOMODE)
        {
            //printf("auto mode\n (GPS)\n");
            while(!fio.eof())
            {
                //printf("loading target...\n");
                smGPSDatum smtemp;
                if (loadGPStar(&smtemp, &fio))
                {
                    printf("Loaded Target: Lat: %2.12f, Lon: %3.12f\n", smtemp.latitude, smtemp.longitude);
                    GPStars.insert(GPStars.begin(), smtemp);
                }
            }
            printf("%d GPS targets loaded.\n", GPStars.size());
        }
        SetEvent(hTargetsLoaded);

        while (!escape)
        {
            if (mode != prevmode)
            {
                prevmode = mode;
                break;
            }
            //find terminating null-character
            char *tc = strchr(templocGPSbuf, 0);
            char *GPSToParse = 0;
            #if FORCEGPSINPUT
                GPSToParse = new char[256];
                sprintf(GPSToParse, "$GNRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W,N*6A");
            #else
                //DWORD evstatus;
                //do
                //{
                //    WaitCommEvent(GPS.hComm, &evstatus, NULL);
                //} while (evstatus != EV_RXFLAG);
                //printf("starting GPS read\n");
                GPS->Read(tc);
                //GPS.Read(rawGPSdata, sizeof(rawGPSdata));
                int i;
                //char *GPSEnd = 0;
                for (i = strlen(tc)/*256*/; i > 0 && tc[i] != '\n'; --i);
                if (tc[i] == '\n')
                {
                    //printf("\\n at i=%d\n", i);
                    //GPSEnd = &tc[i];
                    //tc[i] = 0;
                    for ( ; i > 0 && tc[i] != '$'; --i);
                    if (tc[i] == '$')
                    {
                        GPSToParse = &tc[i];
                        //printf("GPSToParse: %s\n", GPSToParse);
                    }
                }
            #endif

            if (GPSToParse)
            {
                //strcpy(strrchr(locGPSbuf, 0), rawGPSdata);
                #if GPSDEBUG
                    if (rawGPSdata)
                        printf("GPS: %s\n", rawGPSdata);
                    else
                        printf("Error: rawGPSdata == 0\n");
                #endif
                //printf("rawGPSdata: %s\n", rawGPSdata);
                //printf("Going to parse GPS...\n");
                //printf("GPSToParse: %s\n", GPSToParse);
                char *cur_pos = strstr(GPSToParse, "$GNRMC");
                //printf("cur_pos: %s\n", cur_pos);
                if (cur_pos)//strstr(rawGPSdata, "$GNRMC"))
                {
                    //Parse the GPS data from rawGPSdata
                    //Look for GPS coordinates, etc:
                    //char *cur_pos;
                    unsigned char i = 0;            //shoud always be less than 255 commas
                    //printf("looking for a value for cur_pos\n");
                    cur_pos = strchr(cur_pos, ',');//strchr(rawGPSdata, ',');
                    GPSDatum curGPS = {0};
                    //printf("cur_pos: %d\n", cur_pos);
                    while (cur_pos)
                    {
                        double tempd;
                        cur_pos = strchr(cur_pos, ',');
                        if (cur_pos)
                        {
                            cur_pos++;

                            switch(++i)
                            {
                            case 1:     //UTC Time [char*]
                                memcpy(curGPS.time, cur_pos, 6);
                                #if GPSDEBUG
                                    printf("time: %s\n", curGPS.time);
                                #endif
                                break;
                            case 2:     //Status (A = Active, V = Void) [char]
                                curGPS.status = (cur_pos && *cur_pos != ',') ? *cur_pos : 0;
                                #if GPSDEBUG
                                    printf("status: %c\n", curGPS.status);
                                #endif
                                if (tolower(curGPS.status) != 'a')
                                {
                                    //force a break from the loop
                                    cur_pos = 0;
                                }
                                break;
                            case 3:     //Latitude [double]
                                //must convert from DDMM.MMM to DD.DDDD
                                tempd = atof(cur_pos+2);
                                curGPS.latitude = ((atof(cur_pos) - tempd)/100.0 + tempd/60.0);
                                //curGPS.latitude = atof(cur_pos);
                                break;
                            case 4:     //Latitude N/S [char]
                                curGPS.latitudeNS = (cur_pos && *cur_pos != ',') ? *cur_pos : 0;
                                if (tolower(curGPS.latitudeNS) == 's')
                                {
                                    curGPS.latitude *= -1.0;
                                }
                                //#if GPSDEBUG
                                    printf("latitude: %f\n", curGPS.latitude);
                                //#endif
                                break;
                            case 5:     //Longitude [double]
                                //must convert from DDMM.MMM to DD.DDDD
                                tempd = atof(cur_pos+3);
                                curGPS.longitude = ((atof(cur_pos) - tempd)/100.0 + tempd/60.0);
                                //curGPS.longitude = atof(cur_pos);
                                break;
                            case 6:     //Longitude E/W [char]
                                curGPS.longitudeEW = (cur_pos && *cur_pos != ',') ? *cur_pos : 0;
                                if (tolower(curGPS.longitudeEW) == 'w')
                                {
                                    curGPS.longitude *= -1.0;
                                }
                                //#if GPSDEBUG
                                    printf("longitude: %f\n", curGPS.longitude);
                                //#endif
                                break;
                            case 7:     //Velocity (knots) [float]
                                curGPS.velocity = (float)atof(cur_pos);
                                #if GPSDEBUG
                                    printf("velocity: %f\n", curGPS.velocity);
                                #endif
                                break;
                            case 8:     //Heading True North [float]
                                curGPS.heading = (float)atof(cur_pos);
                                #if GPSDEBUG
                                    printf("heading: %f\n", curGPS.heading);
                                #endif
                                break;
                            case 9:     //UTC Date [char*]
                                memcpy(curGPS.date, cur_pos, 6);
                                #if GPSDEBUG
                                    printf("date: %s\n", curGPS.date);
                                #endif
                                break;
                            case 10:    //Magnetic Variation [float]
                                curGPS.magvariation = (float)atof(cur_pos);
                                #if GPSDEBUG
                                    printf("magvariation: %f\n", curGPS.magvariation);
                                #endif
                                break;
                            case 11:    //Magnetic Variation E/W [char]
                                curGPS.magvariationEW = (cur_pos && *cur_pos != ',') ? *cur_pos : 0;
                                if (tolower(curGPS.magvariationEW) == 'e')
                                {
                                    curGPS.magvariation *= -1.0;
                                }
                                #if GPSDEBUG
                                    printf("magvariationEW: %c\n", curGPS.magvariationEW);
                                #endif
                                break;
                            case 12:    //Checksum [char*]
                                memcpy(curGPS.checksum, cur_pos, 4);
                                #if GPSDEBUG
                                    printf("checksum: %s\n", curGPS.checksum);
                                #endif
                                break;
                            }
                        }
                    }
                    if (curGPS.checksum[1] != '*')
                    {
                        printf("continuing...\n\a");
                        goto mylab;     //perhaps there is a more elegant way to fix this (?)
                        //continue;     //continue; did not seem to work
                    }
                    if (mode != GPSMODE)
                    {
                        WaitForSingleObject(mStoreGPS, INFINITE);
                        if (StoreGPS)
                        {
                            smGPSDatum savetempGPSDatum;
                            savetempGPSDatum.latitude = curGPS.latitude;
                            savetempGPSDatum.longitude = curGPS.longitude;
                            saveGPStar(&savetempGPSDatum, &fio);
                            //store the current GPS location at this point in the code
                            StoreGPS = false;
                            printf("GPS Stored!\n");
                            SetEvent(hStoreGPS);        //currently unusued; perhaps later to stop robot until GPS point recorded
                        }
                        ReleaseMutex(mStoreGPS);
                    }
                    else
                    {
                        //it is auto mode
                        printf("GPStars.size(): %d\n", GPStars.size());
                        if (curGPSHit)
                        {
                            if (GPStars.size())
                            {
                                tempsmGPS = GPStars.back();
                                printf("New GPS Target: Lat: %f, Long: %f\n", tempsmGPS.latitude, tempsmGPS.longitude);
                                GPStars.pop_back();
                                curGPSHit = 0;
                            }
                            else
                            {
                                //all GPS targets have been reached, or no GPS targets remain
                                printf("Done.\n");
                                stopmain = true;
                            }
                        }
                        GPSvector tempGPSvec = getvector(&curGPS, &tempsmGPS);
                        tempGPSvec.curdir = curGPS.heading;
                        tempGPSvec.magvariation = curGPS.magvariation;
                        WaitForSingleObject(mGPSData, INFINITE);
                        //curGPSHit = false;
                        curGPSvec = tempGPSvec;
                        //CompassData = atof(rawGPSdata)/10;
                        //copy to global compass value here
                        ReleaseMutex(mGPSData);
                        SetEvent(hNewGPSData);
                    }
                }
            }
mylab:
            char tempstr[256];
            strcpy(tempstr, tc);
            memset(templocGPSbuf, 0, sizeof(templocGPSbuf));
            strcpy(templocGPSbuf, tempstr);

        }
        fio.close();
    }
}

GPSvector getvector(GPSDatum* curGPS, smGPSDatum* otherGPS)
{
    //lat2 ==> otherGPS->latitude;
    //lon2 ==> otherGPS->longitude;
    //lat1 ==> curGPS->latitude;
    //lon1 ==> curGPS->longitude;
    GPSvector tempv;
    //Get Distance
    float R = 6371000; // m
    double dLat = (otherGPS->latitude-curGPS->latitude)*TORAD;
    double dLon = (otherGPS->longitude-curGPS->longitude)*TORAD;
    double a = sin(dLat/2) * sin(dLat/2) + cos(curGPS->latitude*TORAD) * cos(otherGPS->latitude*TORAD) * sin(dLon/2) * sin(dLon/2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    double d = R * c;

    tempv.distance = d;
    #if GPSDEBUG
        printf("d: %f\n", d);
    #endif

    //Get Heading
    double y = sin(dLon) * cos(otherGPS->latitude*TORAD);
    double x = cos(curGPS->latitude*TORAD)*sin(otherGPS->latitude*TORAD) - sin(curGPS->latitude*TORAD)*cos(otherGPS->latitude*TORAD)*cos(dLon);
    float brng = (float)atan2(y, x)*TODEG;
    ///Note: Magnetic variation (available through GPS data) must be accounted for when computing angles
//#if COMPASSON
//    brng += curGPS->magvariation;
//#endif
    //Adjust brng to within range [0, 360)
    if (brng < 0)
        brng += 360.0;

    tempv.direction = brng;
    #if GPSDEBUG
        printf("b: %f\n", brng);
    #endif
    //Calculate distance and direction from GPS latitudes and longitudes
    return tempv;
}

bool saveGPStar(smGPSDatum* sdatum, fstream *fio)
{
    printf("Saving GPS target...\n");
    char stemp[128] = {0};
    sprintf(stemp, "%2.12f, %3.12f\n", sdatum->latitude, sdatum->longitude);
    printf("stemp: %s\n", stemp);
    *fio << stemp << endl;
    //fio->write(stemp, strlen(stemp));
    return true;
}

bool loadGPStar(smGPSDatum* ldatum, fstream *fio)
{
    //printf("starting loadGPStar\n");
    char stemp[128] = {0};
    if (fio->eof())
        return false;
    //printf("stemp: %s, sizeof(stemp): %d\n", stemp, sizeof(stemp));
    fio->getline(stemp, sizeof(stemp));
    char* ctemp = strchr(stemp, ',');
    if (ctemp)
        ctemp++;
    else
        return false;
    ldatum->latitude = atof(stemp);
    //printf("ctemp: %d\n", ctemp);
    ldatum->longitude = atof(ctemp);
    return true;
}

DWORD WINAPI GPSLoop(LPVOID param)
{
    //param to GPS stuff is filename to store to and/or read from GPS points
    GPSstuff("test.csv");
    return 0;
}
