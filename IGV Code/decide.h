#ifndef DECIDE_H_INCLUDED
#define DECIDE_H_INCLUDED

#warning "including decide.cpp"

#include <iostream>
//#include "circle.h"
#include <cmath>
#include <map>
#include <utility>
#include <algorithm>

#include "globals.h"

using std::cout;
using std::endl;
using std::cerr;
using std::multimap;
using std::pair;

typedef unsigned short ushort;

#define PI 3.1415926535897932384626433832795028841971
#define cDepth 10
#define lateralLimit 7 /// +/- 35 degrees on either side
//using namespace std;

enum pixelTypes{PCLEAR=0, PUNKNOWN, PAVOID, POBSTACLE, PLINE};


class Section
{
    public:
    Section(ushort newCert=0, ushort newObst=0)
    :certainty(newCert),obstacle(newObst)
    {
        findType();
    }

    ushort certainty; ///certainty of what is there
    ushort obstacle; ///confidence it is an obstacle, clear = 0

    ushort type;

    void doType()
    {
        switch(type)
        {
            case POBSTACLE:
                obstacle = 100;
                certainty = 100;
                break;
            case PAVOID:
                obstacle = 50;
                certainty = 100;
                break;
            case PUNKNOWN:
                obstacle = 50;
                certainty = 0;
                break;
            case PLINE:
                obstacle = 100;
                certainty = 100;
                break;
            default: ///PCLEAR
                obstacle = 0;
                certainty = 100;
                break;
        }
    }

    void setType(ushort newType)
    {
        if(newType <= PLINE)
            type = newType;

        doType();
    }

    void findType()
    {
        if(certainty <= 50)
        {
            type = PUNKNOWN;
            return;
        }

        if(obstacle >= 67)
        {
            type = POBSTACLE;
            return;
        }

        if(obstacle >= 33)
        {
            type = PAVOID;
            return;
        }

        type = PCLEAR;
        return;
    }
};

class dPixel
{ ///These are just for the 2D map to simulate camera input roughly.
    public:
    dPixel(ushort newType=PCLEAR, ushort newDepth=0):type(newType),depth(newDepth){}
    //~dPixel()
    //{
        //cout << type << "," << depth << " ";
    //}
    ushort type; //enum pixeltypes
    ushort depth;
};

struct Gap
{
    short begin;
    short end;
//    ushort depth;
};

///10 circles, each circle a circular array, I think I intend each Circle[0] to be straight ahead.
///I also think I might have intended for each angle to represent the middle of the section
Circle<Section> theGrid[cDepth];
short center=0; /// center of the grid, needs to be on a border so must be %5

const ushort circDepth[] = {6, 6, 6, 6, 6, 12, 12, 24, 48, 96};
//                          30, 60, 90, 120, 150, 210, 270, 390, 630, 1110
//                          1.1, 1.2, 1.3, 1.4, 1.5, 1.7, 1.9, 2.3, 3.1, 4.7
///Certainty levels at a variety of depths.
const ushort certDepth[] = {100, 100, 100, 13/14, 13/15, 13/17, 13/19, 13/23, 13/31, 13/47};
const ushort circWidth = 360/5; ///degrees, 0 is straight ahead

//int fill;
//unsigned short i;

dPixel testMap[240][320]; ///This is just for the rect->polar stuff which is probably just demo.
//map<ushort, map<ushort, Gap> > theGaps;
multimap<ushort, Gap> theGaps;
double gapMap[10][14]; ///Probably better to do 140 equations right off the bat than n->inf over time.



/**
*   Looks for gaps in the polar map to help determine where to go.
*   @param dLimit: The MAXimum depth to search;
*   @param cTolerance: The MINimum certainty level to consider.
*   @param depLimit: The MAXimum obstacle level to consider.
*/
void gapFinder(ushort dLimit, ushort cTolerance = 50, ushort oTolerance = 33)
{

    Section current;
    Gap* gapMaker = 0;
    //list<Gap>* listMaker;

    bool inGap, lastGap;

    ushort i;
    short j;


    theGaps.clear();

    for(i=0;i<dLimit;i++)
    {
        inGap = lastGap = false;
        //listMaker = new list<Gap>;
        ///There is an issue here: Do we do a lateralLimit of +/- 7 so we get a span of 70?
        ///Or do we just go the full circle and have it skip the obvious unknowns?
        ///For now: laterLimit is more efficient.
        ///If we performed this on a merged map: We would want to be able to detect gaps
        ///     as wide as 180 degrees. (The math works out like that at point blank)
        ///Actually, we can't see the first three or so circles anyway. But the max would
        ///     increase anyway.
cerr << endl;
        for(j=lateralLimit*-1;j<lateralLimit;j++)
        {
            //cerr << j << " ";
            ///We are starting not in a gap. A gap is defined by the tolerance parameters.
//cerr << (theGrid[i][j]).certainty << "," << (theGrid[i][j]).obstacle << " ";

  if( (theGrid[i][j]).obstacle < oTolerance ) cerr << "0 ";
  else cerr << "X ";

            if( (theGrid[i][j]).certainty > cTolerance
            &&  (theGrid[i][j]).obstacle < oTolerance)
            {
                inGap = true;
            }
            else
            {
                inGap = false;
                //cerr << i << "," << j << "! (";
                //cerr << ((*theGrid[i])[j]).certainty << " < " << cTolerance << " ";
                //cerr << ((*theGrid[i])[j]).obstacle << " > " << oTolerance << ") ";
            }

            if(inGap && !lastGap)
            {
                gapMaker = new Gap;
                //gapMaker->depth = i;
                gapMaker->begin = j;
                //cerr << i << ": " << j << ">|| ";
            }

            ///if lastGap and if gapMaker should be equivalent, but the latter is "safer"
            if( (!inGap || (inGap && j+1 == lateralLimit) ) && gapMaker)
            {
                gapMaker->end = j;
                //cerr << "||" << j << endl;
                pair<ushort, Gap> helper;
                helper.first = i;
                helper.second = *gapMaker;
                //theGaps.insert(pair<ushort,Gap>(i, *gapMaker);
                theGaps.insert(helper);
                //listMaker->push_back(*gapMaker);
                delete gapMaker;
                gapMaker = 0;
            }

            lastGap = inGap;
        }
        //theGaps.push_back(*listMaker);
        //cerr << endl;
    }

    //cerr << theGaps.size() << endl;
}

ushort minGap[10]; ///Minimum gap needed to clear.

void fillGapMap()
{
    int i,j;
    double dist=0, angle=0, gap=0;

    for(i=0;i<10;i++)
    {
        dist += circDepth[i];
        minGap[i] = 0;

        for(j=0;j<14;j++)
        {
            angle = 5.0*j;
            /// This is actually dealing with two right triangles mashed together
            /// and the angle is the total angle at the bottom.
            /// So .5 the angle, then 2 the gap.
            gap = 2.0 * dist * sin(angle*.5*PI/180);

            gapMap[i][j] = gap;

            if( !(minGap[i]) && gap > 36)
                minGap[i] = j;

            //cerr << i << "," << j << ": " << gap << endl;
            //if(gap >= 36)
            //    cout << "*";
        }

        if(!(minGap[i]))
            minGap[i] = 15; ///error, need more space, never gonna have 14 + 1
        //cout << endl;
    }
}

/**
*   Checks each depth on record for adequately large gaps.
*   For simplicity's sake, it does not check each gap to be contiguous.
*   In the real world, you can't see behind walls anyway.
*       Although we might with a merged map, so this logic would have to be improved.
*   @return ushort: The furthest depth that does not contain an open gap.
*/
ushort gapChecker()
{
    ushort i;
    bool open;
    multimap<ushort,Gap>::iterator it;
    pair<multimap<ushort, Gap>::iterator,multimap<ushort, Gap>::iterator> ret;

    //for(it=theGaps.begin();it!=theGaps.end();it++)
    for(i=0;i<cDepth;i++)
    {
        open=false;

        ///Is there more than one gap at this depth?
        if(theGaps.count(i) > 1)
        {

            ///Give me a list of all the depths.
            ret = theGaps.equal_range(i);

            ///Are any of them big enough?
            for(it=ret.first;it!=ret.second;it++)
            {

            cerr << i << ": " << (it->second).begin << " to " << (it->second).end << ": " << gapMap[i][ (it->second).end - (it->second).begin] << endl;
                if( i<5 || (it->second).end - (it->second).begin >= minGap[i])
                    open=true;
            }
        }
        else
        {
            ///Give me the only gap.
            it = theGaps.find(i);

            ///Is it big enough?
                ///We can't actually see 36 worth of space until depth 5
                ///I guess I'll just hard core it, for now, to ignore single gaps in that range.
            if( i<5 || (it->second).end - (it->second).begin >= minGap[i])
                open=true;

            cerr << i << ": " << (it->second).begin << " to " << (it->second).end << ": " << gapMap[i][ (it->second).end - (it->second).begin] << endl;
        }

        if(!open)
            return i;
    }

    return i-1;
}

struct theGapComp
{
    bool operator()(pair <ushort,Gap> A, pair <ushort,Gap> B)
    {
        return ( ( (A.second).end - (A.second).begin )/2
                   <
                 ( (B.second).end - (B.second).begin )/2 );
    }
} gapComp;

///Requires theGaps to be filled.
///Returns the largest gap at the first need for a choice.
///Depth loops from dMin to dMax.
///If both are the same number, it will point to somewhere in that specific line regardless
pair<short,short> gapPicker(ushort dMin=0, ushort dMax=10)
{
    pair<short, short> toReturn; ///The i and j of the section in the middle of the target gap.
    ushort i;
    //list<Gap>::iterator it;
    multimap<ushort,Gap>::iterator it;
    pair<multimap<ushort, Gap>::iterator, multimap<ushort, Gap>::iterator> ret;

    if(dMin==dMax)
    {
        ret = theGaps.equal_range(dMin);

        it = max_element(ret.first, ret.second, gapComp);

        //cerr << "Max Gap: " << (it->second).begin << " - " << (it->second).end << endl;

        toReturn.first = dMin;
        toReturn.second = ((it->second).end + (it->second).begin)/2;

        return toReturn;
    }

    //for(it=theGaps.begin();it!=theGaps.end();it++)
    for(i=dMin;i<dMax;i++)
    {
        ///Is there more than one gap at this depth?
        if(theGaps.count(i) > 1)
        {
            ///Give me a list of all the depths.
            ret = theGaps.equal_range(i);

            ///What is the largest Gap in this range of gaps?
            it = max_element(ret.first, ret.second, gapComp);

            /*//Okay, which gap was that?
            for(it=ret.first;it!=ret.second;it++)
            {
                if( ((it->second).end - (it->second).begin)/2 == toPick)
                {
                    toReturn.first = i;
                    toReturn.second = ((it->second).end + (it->second).begin)/2;
                }
            }*/

            toReturn.first = i;
            toReturn.second = ((it->second).end + (it->second).begin)/2;

            break;
        }
    }

    return toReturn;
}


/**
*   ///Check to see how far you can just plow forward.
*    ///At such a point we should probably check any gaps at that depth.
*   @param dLimit: The MAXimum depth to search;
*   @param cTolerance: The MINimum certainty level to consider.
*   @param depLimit: The MAXimum obstacle level to consider.
*/
ushort checkPlow(ushort dLimit, ushort cTolerance = 50, ushort oTolerance = 33)
{
    ushort i;
    short j;

    for(i=0;i<dLimit;i++)
    {
        //cerr << endl;
        //cerr << center << " +/- " << minGap[i]/2 << endl;
        for(j=center-minGap[i]/2;j<=center+minGap[i]/2;j++)
        {
            //cerr << j << " from " << center-minGap[i]/2 << " to " << center+minGap[i]/2
            //     << " unless " << lateralLimit-1 << " or " << lateralLimit*-1 << endl;
            if(j>lateralLimit-1) ///edge of map
                break;

            if(j<lateralLimit*-1)
                j=lateralLimit*-1;

            /*if(i==4)
            {
             cerr << ((*theGrid[i])[j]).certainty << " "
                  << cTolerance << " "
                  << ((*theGrid[i])[j]).obstacle << " "
                  << oTolerance << endl;
            }*/

            //cerr << (theGrid[i][j]).certainty << " > " << cTolerance << "?" << endl;
            //cerr << (theGrid[i][j]).obstacle << " < " << oTolerance << "?" << endl;

            if( (theGrid[i][j]).certainty > cTolerance
            &&  (theGrid[i][j]).obstacle < oTolerance)
            {
                //cerr << "0";
                continue;
            }

            //cerr << "x";
            ///else all hope is lost

            return i;
        }
    }

    return i;
}

Section* getPixel(ushort y, short x) ///RECTANGULAR TO POLAR
{
    //cerr << "obstacle: " << y << "," << x << endl;

    //cerr << "obstacle bottom: " << x-160 << "," << y << " (" << testMap[y][x].depth << ") " << pixelCert;
    ushort i=0;
    float t = atan2(x-160, y); ///0 is in the middle of 320, Flipped on purpose, draw out the triangle and you'll see.
    ushort r = testMap[y][x].depth; ///Really we get this from the camera.
    //cerr << "t: " << t << " rads; r: " << r << "in" << endl;

    t = 180*t/PI; ///LOL RADIANS
    //t = 10;
    //cerr << "r: " << r << " in" << endl;
    //cerr << "t: " << t << " deg" << endl;

    if(t>0)
    {
        while(1)
        {
            t -= 5; ///5 degrees each section

            if(t<=0)
                break;

            i++;
        }
        t = i; ///number of sections

        //t %= 5; no modulo on floats
    }
    else if(t<0)
    {
        while(1)
        {
            t += 5;

            if(t>=0)
                break;

            i++;
        }
        t = i*-1;

        //t = -1*((-1*t)%5); no modulo on floats
    }
    //t = i;
    //Now it is the grid index.
//    cerr << "[" << t << "," << endl;

    i=0;
    while( /*r>circDepth[i]*/ 1)
    {
        r -= circDepth[i]; ///depth of each section

        if(i<9 && r<circDepth[i+1] || i>=9)
            break;

        i++;
        //j++;
    }

    r = i; ///grid index
    //cerr << r << "]" << endl;

    return &theGrid[(int)r][(int)t];

}

Section* getFloor(ushort y, short x)
{ ///CURRENTLY NOT IN USE I DON'T THINK BUT MAYBE IT IS OKAY JUST CHECK WHATEVER

    ///vertical object, just get down to where it starts
    //but what if the bottom bits aren't marked as an obstacle for speed bumps?
        //whatever, the logic exists

    if((testMap[y-1][x].type == POBSTACLE) && (testMap[y][x].type == POBSTACLE))
    {
        if(testMap[y-1][x].depth == testMap[y][x].depth
        && testMap[y][x].depth != 0)
            return NULL;
    }

    return getPixel(y,x);
}

/*Section visToSect(/*VisHeader input, bool test=false/)
{
    Section theSect;

theSect.obstacle = 0;
theSect.certainty = 100;
//printf("4decideYelCnt: %d\n", ToDecisionCircle[4][2].YelCount);
//printf("decideRedCnt: %d\n", ToDecisionCircle[4][2].RedCount);
    //printf("visToSect 1\n");
    if (input.TotCount)
    {
        /*theSect.obstacle =
        100*(input.TotCount/2 + (input.WhtCount + input.RedCount + input.YelCount/2)/2 - input.GrnCount/2)/input.TotCount;

        //printf("visToSect 2\n");

        //(100*(input.WhtCount + input.RedCount + input.YelCount/2 - input.GrnCount))/input.TotCount;

        if(input.WhtCount + input.RedCount > input.GrnCount)
            theSect.certainty = (100*(input.WhtCount + input.RedCount))/input.TotCount;
        else
            theSect.certainty = 100*input.GrnCount/input.TotCount;/

        if(ToDecisionCircle.RedCount/ToDecisionCircle.TotCount > .15
        || ToDecisionCircle.WhtCount/ToDecisionCircle.TotCount > .05)
            theSect.obstacle = 100;
            //theSect.certainty = 100;


    }

    cerr << "{" << input.RedCount << " " << input.YelCount << " " << input.GrnCount
        << " " << input.WhtCount << " " << input.TotCount << "} ";
    //printf("visToSect 3\n");

    /*cerr << input.WhtCount << " " << input.RedCount << " " << input.YelCount << " "
         << input.GrnCount << " " << input.TotCount << " " << theSect.obstacle << " "
         << theSect.certainty << endl;/

    theSect.findType();

    //printf("visToSect 4\n");

    return theSect;
}*/

void processArray(/*Circle<VisHeader> input[]*/)
{

    ///For right now pretend we already know it's 10 deep and 14 wide
    ///Also remember that sizeof() method of determining array size, hells yes.
    ushort i=0;
    short j=-7;
    for(i=0;i<10;i++)
    {
        for(j=-7;j<7;j++)
        {

            //printf("i: %d,\tj: %d\n", i, j);
            //theGrid[i][j] = visToSect(ToDecisionCircle[i][j]);
            theGrid[i][j].obstacle = 0;
            theGrid[i][j].certainty = 100;

            if (ToDecisionCircle[i][j].TotCount)
            {
                if((float)ToDecisionCircle[i][j].RedCount/(float)ToDecisionCircle[i][j].TotCount > .25
                || (float)ToDecisionCircle[i][j].WhtCount/(float)ToDecisionCircle[i][j].TotCount > .05)
                    theGrid[i][j].obstacle = 100;

            //cerr << (float)ToDecisionCircle[i][j].RedCount << "|" << (float)ToDecisionCircle[i][j].WhtCount
            //     << "/" << (float)ToDecisionCircle[i][j].TotCount << " ";
            //cerr << (float)ToDecisionCircle[i][j].RedCount/(float)ToDecisionCircle[i][j].TotCount << ","
            //     << (float)ToDecisionCircle[i][j].WhtCount/(float)ToDecisionCircle[i][j].TotCount << " ";
            }
        }
        cerr << endl;
    }
}

void setCenter(float newCenter)
{
    //cerr << "math" << (newCenter - ((short)(newCenter/5)*5)) << endl;
//cerr << "newCenter: " << newCenter << endl;
//cerr << newCenter - ((short)(newCenter/5)*5) << endl;

    if( (newCenter - ((short)(newCenter/5)*5)) >= 2.5) ///Hopefully this truncation trick works.
        center = ((short)(newCenter/5)+1);
    else
        center = (short)(newCenter/5);


}

void doGo()
{
    float curLaneAngle;
    float curHeading;

    WaitForSingleObject(NewMap, INFINITE);
    EnterCriticalSection(&GetDecisionMap);
    for (int j = 0; j < 10; j++)
    {
        for (int i = -7; i < 7; i++)
        {
            ToDecisionCircle[j][i].GrnCount = 0;
            ToDecisionCircle[j][i].GrnCount = GlobalCircle[j][i].GrnCount;
            //printf("decideGrnCnt: %d\n", ToDecisionCircle[j][i].GrnCount);
            GlobalCircle[j][i].GrnCount = 0;
            ToDecisionCircle[j][i].YelCount = 0;
            ToDecisionCircle[j][i].YelCount = GlobalCircle[j][i].YelCount;
            //printf("decideYelCnt: %d\n", ToDecisionCircle[j][i].YelCount);
            GlobalCircle[j][i].YelCount = 0;
            ToDecisionCircle[j][i].WhtCount = 0;
            ToDecisionCircle[j][i].WhtCount = GlobalCircle[j][i].WhtCount;
            //printf("decideWhtCnt: %d\n", ToDecisionCircle[j][i].WhtCount);
            GlobalCircle[j][i].WhtCount = 0;
            ToDecisionCircle[j][i].RedCount = 0;
            ToDecisionCircle[j][i].RedCount = GlobalCircle[j][i].RedCount;
            //printf("decideRedCnt: %d\n", ToDecisionCircle[j][i].RedCount);
            GlobalCircle[j][i].RedCount = 0;
            ToDecisionCircle[j][i].TotCount = 0;
            ToDecisionCircle[j][i].TotCount = GlobalCircle[j][i].TotCount;
            //printf("decideTotCnt: %d\n", ToDecisionCircle[j][i].TotCount);
            GlobalCircle[j][i].TotCount = 0;
        }
    }
    curLaneAngle = globalLaneAngle;
    cerr << "laneAngle" << curLaneAngle << endl;

    LeaveCriticalSection(&GetDecisionMap);
//printf("1decideYelCnt: %d\n", ToDecisionCircle[4][2].YelCount);
//printf("decideRedCnt: %d\n", ToDecisionCircle[4][2].RedCount);
#if 0
    WaitForSingleObject(mCompassData, INFINITE);
    curHeading = CompassData;
    ReleaseMutex(mCompassData);
#endif
//printf("2decideYelCnt: %d\n", ToDecisionCircle[4][2].YelCount);
//printf("decideRedCnt: %d\n", ToDecisionCircle[4][2].RedCount);
    setCenter(curLaneAngle);

//printf("3decideYelCnt: %d\n", ToDecisionCircle[4][2].YelCount);
//printf("decideRedCnt: %d\n", ToDecisionCircle[4][2].RedCount);
    if(center!=0)
        cerr << "CENTER::: " << center << endl;
    processArray(); ///Process the array into our Sections.
//printf("4decideYelCnt: %d\n", ToDecisionCircle[4][2].YelCount);
//printf("decideRedCnt: %d\n", ToDecisionCircle[4][2].RedCount);
    gapFinder(10); /// get all the gaps
    fillGapMap(); /// set up some constants
    //cerr << gapChecker() << endl;; /// checks how far until there is an unpassable gap

    ///tell me when plowing forward hits an obstacle,
    ///and check from that point on for a gap to aim at
        ///By giving it checkPlow twice I am forcing it to pick a section on that line.
        ///If it is a full wall it will probably just point at the left edge.
            ///We can give it more attention later.
    pair<short, short> helper = gapPicker(checkPlow(10), checkPlow(10));
    cerr << endl << helper.first << "(" << checkPlow(10) << ")," << helper.second << endl;
    //cout << helper.second *5 << " degrees" << endl;

    WaitForSingleObject(mVector, INFINITE);
    //set TermString or speed and dir
    //if TermString is not empty, then the contents of TermString are sent
    //else, the vector (long int speed, long int dir) is sent
    printf("speed: %ld\n", helper.first);
    speed = (long)helper.first; /// distance in inches
    printf("dir: %ld\n", long(helper.second*5.0));
    dir = (long)(helper.second*5.0); /// degrees
    ReleaseMutex(mVector);
    SetEvent(hNewMotion);
    //WaitForSingleObject(hMotionDone, INFINITE);


    //return helper.second*5;
}

//Circle<VisHeader> visMap[10];
int main2()
{
    int i, j;
    ushort holdObst = 0, holdCert = 0;
    Section* theSect;

    //for(i=0;i<10;i++)
    //    theGrid[i] = new Circle<Section>(circDepth[i],circWidth);

/*    for(i=239;i>=0;i--)
    {
        for(j=0;j<320;j++)
        {
            if(i >= 90 && i <= 150 && j >= 120 && j <= 200)
            {
                testMap[i][j].type = POBSTACLE;
                testMap[i][j].depth = 30;
            }
            else
                testMap[i][j].depth = i/3; ///Simulating depth of ground over distance
        }
        //if(i > 150 && testMap[i][j].type == POBSTACLE)
        //    cerr << i << "," << j << " ";
    }

    for(i=0;i<240;i++)
    {
        for(j=0;j<320;j++)
        {

            //if(testMap[y-1][x].depth == testMap[y][x].depth
            //&& testMap[y][x].depth != 0)

            //if((testMap[y-1][x].type == POBSTACLE) && (testMap[y][x].type == POBSTACLE))


            if(theSect = getPixel(i, j))
            {
                ///default for clear
                holdObst = 0;
                holdCert = 100;

                switch(testMap[i][j].type)
                {
                    case POBSTACLE:
                        //if( testMap[i][j].depth < 10)
                          //  cerr << "obstacle: " << i << "," << j << endl;
                        holdObst = 100;
                        break;
                    case PAVOID:
                        holdObst = 50;
                        break;
                    case PUNKNOWN:
                        holdObst = 50;
                        holdCert = 0;
                        break;
                    case PLINE:
                        holdObst = 100;
                        break;
                    default: //PCLEAR
                        //already set defaults before switch
                        break;
                }
                //if(helper && i>150)
                //    cerr << "obstacle: " << i << "," << j << endl;
                //if(testMap[i][j].depth > 0)
                    //cerr << i << "," << j << ": " << testMap[i][j].depth << "  ";
                //if(helper > 0 /*&& testMap[i][j].depth != 28*)
                //    cerr << "i:" << i << endl;

                theSect->certainty = holdCert;
                theSect->obstacle = holdObst;
            }
            else
            {

                theSect = getPixel(i,j);
                theSect->certainty = 100;
                theSect->obstacle = 0;
            }
        }
    }*/

    /*for(i=0;i<10;i++)
    {
        for(j=-7;j<7;j++)
        {
            visMap[i][j].RedCount = 10;
            visMap[i][j].YelCount = 10;
            visMap[i][j].GrnCount = 80;
            visMap[i][j].WhtCount = 0;
            visMap[i][j].TotCount = 100;

            if(i ==5 && j >= -3 && j <= 3)
            {
                visMap[i][j].RedCount = 80;
                visMap[i][j].YelCount = 10;
                visMap[i][j].GrnCount = 10;
            }
        }
        //if(i > 150 && testMap[i][j].type == POBSTACLE)
        //    cerr << i << "," << j << " ";
    }*/


//
//    cout << getAngle(0, visMap) << " degrees" << endl;


    for(i=9;i>=0;i--) // 0 to 6+6+6+6+6+12+12 etc
    {
        for(j=-7;j<7;j++) //-35 to 35 degrees
        //for(j=-13;j<13;j++)
        {
                //cout << i << " " << j << " ";
                //helper = (*theGrid[i])[j].obstacle;

                if( theGrid[i][j].certainty < 50 )
                {
                    //cerr << i << "," << j << ": " << (*theGrid[i])[j].certainty << endl;
                    cout << "? ";
                }
                else
                {
                    if(theGrid[i][j].obstacle < 50)
                        cout << "0 ";
                    else
                    {
                        cout << "X ";
                        //cout << (*theGrid[i]).depth << " ";
                    }
                }

                //cout << (*theGrid[i])[j].certainty << " ";
        }
        cout << endl;
    }

    /*gapFinder(10);
    fillGapMap();
    cout << gapChecker() << "!";

    pair<short, short> helper = gapPicker();
    cout << endl << helper.first << "," << helper.second << endl;
    cout << helper.second *5 << " degrees" << endl;

    //cout << checkPlow(10) << endl;
    helper = gapPicker(checkPlow(10), 10);
    cout << endl << helper.first << "," << helper.second << endl;
    cout << helper.second *5 << " degrees" << endl;*/

    return 0;
}



void gapDemo()
{
    int i,j;
    double dist=0, angle=0, gap=0;

    cout.precision(4);
    cout << "\t05\t10\t15\t20\t25\t30" << endl;

    for(i=0;i<20;i++)
    {
        if(i==10)
        {
            dist=0;
                cout << endl << "\t35\t40\t45\t50\t55\t60" << endl;
        }

        dist += circDepth[i%10];
        cout << dist << "\": ";

        if(i<10)
            angle = 0;
        else
            angle = 30;
        for(j=0;j<6;j++)
        {
            angle += 5;
            cout << "\t";
            gap = dist * 2.0*sin(angle*.5*PI/180);
            cout << gap;

            if(gap >= 36)
                cout << "*";
        }

        cout << endl;
    }
}
#else
#warning "decide.cpp didn't get through"
#endif
