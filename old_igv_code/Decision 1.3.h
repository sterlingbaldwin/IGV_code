#if 0
//Decision coce 1.3
//not using Shadow array
//params passed--
//       linked list to cell densities
//              - hit, wht, clr numbers passed, calc density in this code
//              - pointer to linked list
//
//

//////////////////////// functions ////////////////////////////

// Beginnings of Decision code

//#include <stdio.h>
//#include <math.h>

//%%%%%%%%%%%%%%%%%  Structures Definition %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

#if 0
typedef struct VisPoint{
        int type;             //0 - clear; 1 - sonar; 2 - obstacle ; 4 - white
        float radius;         //sqrt(x^2+y^2)
        float theta;          //arctan(x/y)
        VisPoint *next;
};

typedef struct VisHeader{
        unsigned int RedCount;             //target color
        unsigned int WhtCount;             //line color
        unsigned int GrnCount;             //clear color
        unsigned int YelCount;             //if adjacent to red is target, if only yel should avoid
        unsigned int TotCount;             //total hits
        float CenterX;
        /*VisHeader *next;*/
};
#endif

struct SonHit{
        float distance;
};

struct SonScan{
        SonHit FrontR;
        SonHit FrontL;
        SonHit BackR;
        SonHit BackL;
};

struct Direction{
        float XError[27];                      // average clear distance if > 6 clear cells
		int XErrorCount;
		char Clear;							//true if and only if completely clear
        float YDist;                         // equal to radius
        float ErrorAngle;
        float ErrorDist;
/*        float FirstHit;
        float LastHit;*/
};

#define DEBUG 0

//%%%%%%%%%%%%%%%%%%%%%%%% End of Structure Definitons %%%%%%%%%%%%%%%%%%%%%%%%%%%

//%%%%%%%%%%%%%%%%%%%%%%%%%    VARIABLES     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
int speed;
int ErrorAngle;
                               //need a flag to motion control

//%%%%%%%%%%%%%%%%%%%%%%%%% End of Variables %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//%%%%%%%%%%%%%%%%%%%%%%%% Function Declarations %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

/*int Length(struct VisPoint* head);*/
/*void SpeedToRes(double speed, double theta, int incrementer);*/

//%%%%%%%%%%%%%%%%%%%%%%% End of Function Declarations %%%%%%%%%%%%%%%%%%%%%%%%%

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Begin Cyclic %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
int RefreshMap(void){
	printf("Refresh Map started successfully.\n");
    while(1)
    {
     //MapCell* RefreshedMap;
     //int ClearOnLeft=0;
     //int ClearOnRight=0;

     float MaxDens = 5;                           //Max density allowed to be called clear
     int ConsecClrRow = 0;
     //float ClrDist = 0;
     //int HoldingFlag = 0;                        //flag to be set after placing y-axis onto target
                                                 //**set in motion control

     Direction ClearArray[16]={0};                //1D array holding direction to individual row target

     VisHeader DecideMap[266];                 // array to be memcopy'd into
     //int indexer = 0;
     //float CellWidth = 0.15;                  //
	 float XInc = 0.15;
     float ErrorX = 0;                        //  x value of target
                                              //  (outer - inner)/2 -- inner 1st hit, outer last hit
     int First = 0;                        //  Holder of first and last hit -- used for calc'n ErrorX
     int Last = 0;
	 int FirstSet = 0;
	 float FirstX = 0;
	 float LastX = 0;
	 //float BotWidth = 0.9;				//0.887 m is measured bot width
	 //float percentage = 0;
	 //int ClearRowFlag = 0;


  /*      receive array from BumbleBee Code when allowed	*/
		memset(DecideMap,0,sizeof(DecideMap));
		memset(ClearArray, 0, sizeof(ClearArray));
		WaitForSingleObject(NewMap, INFINITE);
		EnterCriticalSection(&GrabSemaMap);
		memcpy(DecideMap,BigSemaMap,sizeof(DecideMap));
		memset(BigSemaMap,0,sizeof(BigSemaMap));
		LeaveCriticalSection(&GrabSemaMap);

        /* fill up Clear Array */
		int MapOffset;
		//int CellCount = 0;
		ConsecClrRow=0;
		bool speedset = false;
		float ConsecClrCells[6];
		int ConsecClrCount;
		float x;

		for(int i=0; i < 16; i++)
		{
				memset(ConsecClrCells, 0, sizeof(ConsecClrCells));
				ConsecClrCount = 0;
				ClearArray[i].XErrorCount = 0;
				MapOffset = 0;
				for(int ITemp = 0; ITemp < i; ITemp++)
					MapOffset+=mapwidth[ITemp];
				//printf("i: %d\tMapOffset: %d\n", i, MapOffset);
				ClearArray[i].YDist=(0.15*(i/*+1*/))+.7;
				//ClearArray[i].XError = 100;
				FirstSet = 0;
				LastX=0;
				FirstX=0;
				First=0;
				Last=0;
	#if DEBUG
				for (int spacer = (mapwidth[15]-mapwidth[i])/2; spacer >= 0; spacer--)
					printf(" ");
	#endif
				for(int n = 0/*, clrcnt = 0*/; n < mapwidth[i]; n++)               /* index 0->29, then 29->29+33, etc */
				{
					x = n*XInc-mapwidth[i]*XInc/2+.15;		//dunno why the .15 had to be added, but it is important
					int CellCount = MapOffset+n;
					int Clear;							// 1 for clear, 0 for set
					//printf("%d: %d\n", CellCount, DecideMap[CellCount].TotCount);
					if (ConsecClrCount >= 6)
					{
						ClearArray[i].XError[ClearArray[i].XErrorCount] = 0;
						for (int c = 0; c < 6; c++)
							ClearArray[i].XError[ClearArray[i].XErrorCount]+=ConsecClrCells[c];
						ClearArray[i].XError[ClearArray[i].XErrorCount] /= 6;
						ClearArray[i].XErrorCount++;
					}
					if (DecideMap[CellCount].TotCount)
					{
						 //percentage = ((float)DecideMap[CellCount].RedCount/DecideMap[CellCount].TotCount)*100;
						 //printf("Percent: \t%f\n", percentage);
						 if((((float)DecideMap[CellCount].RedCount/DecideMap[CellCount].TotCount)*100) < MaxDens)
							 Clear = 1;
						 else
							 Clear = 0;
					}
					else
						Clear = 1;
					switch(Clear)
					{
					case 1:	/* Clear Case */
	#if DEBUG
						printf("0");
	#endif
						ConsecClrCells[ConsecClrCount%6] = x;
						ConsecClrCount++;
						break;
					case 0:	/* Not Clear */
	#if DEBUG
						printf("1");
	#endif
						ConsecClrCount = 0;
						break;
					}
				}
				if (ConsecClrCount == mapwidth[i])
				{
					ClearArray[i].Clear = 1;
					//printf("X0: %f\tXl: %f\n", ClearArray[i].XError[0], ClearArray[i].XError[ClearArray[i].XErrorCount-1]);
				}
				else
					ClearArray[i].Clear = 0;

	#if DEBUG
				printf("\n");
	#endif
				//printf("XErrorCount: %d\n", ClearArray[i].XErrorCount);
				//printf("FirstX: %f\tLastX: %f\n", FirstX, LastX);
				//printf("ConsecClrRow: %d\n", ConsecClrRow);
				if (ClearArray[i].XErrorCount && i < 15)
				{
					 ConsecClrRow++;						// 3 clear rows = .45 meters ~1.5 ft; base speed upon the number
				}											// of clear rows in x refreshed frames
				else
				{
					if (!speedset)
					{
						speed=ConsecClrRow;
						ConsecClrRow = 0;
						speedset=true;
					}
				}
		}
		// calculate path
		int i;
		for (i = 0; ClearArray[i].Clear; i++);
		ErrorX = ClearArray[i].XError[0];

		for (int n = 1; n < ClearArray[i].XErrorCount; n++)
			if (ClearArray[i].XError[n] < ErrorX)
				ErrorX = ClearArray[i].XError[n];
#if DEBUG
		printf("Angle: %f\n", atan(ErrorX/ClearArray[i].YDist)*TODEG);
		printf("\t\tSpeed: \t%d\n",speed);
#endif
	/*
		char temp1CommData[256] = {0};
		sprintf(temp1CommData, "%d, %f\n", speed, atan(ErrorX/ClearArray[i].YDist)*TODEG);
		//printf("%s", temp1CommData);
		EnterCriticalSection(&SetCommData);
		memcpy(CommData, temp1CommData, sizeof(CommData));
		LeaveCriticalSection(&SetCommData);
		SetEvent(NewCommData);
		*/
	}
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% End of Cyclic %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Function Definitions %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

/*
int Length(VisPoint* head){

    VisPoint* current = head;
    int count = 0;

    while (current != NULL) {
          count++;
          current = current->next;
    }
    return count;
}*/

//build and return a pointer to a 2 cell array holding pointers to Ground and Shadow arrays
//Ground and Shadow are both of type int and holding hit densities
//not passing the right things nor returning the right thing

//vispoint *visindex

/*void SpeedToRes(double speed, double theta, int incrementer){

//   take speed and base theta and radius resolution on it
//   switch-case statement broken into:
//               Fastest - theta / 3   .3782 rads
//                         inc = 2 / 5  .4m 15.75in
//                         3X5 Array
//               Fast - theta / 4      .2836 rads
//                      inc = 2 / 10     .2m 7.87in
//                      4X10 Array
//               Medium - theta / 5     .2269 rads
//                        inc = 2 / 20   .1m 3.94in
//                        5X20 Array
//               Slow - theta / 6       .1891 rads
//                      inc = 2 / 40     .05m 1.97in
//                      6X40 Array
}*/

//%%%%%%%%%%%%%%%%%%%%%% End of Function Definitions %%%%%%%%%%%%%%%%%%%%%%%%%%

#endif
