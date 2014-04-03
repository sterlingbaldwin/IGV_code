#pragma once
#ifndef CIRCLE_H_INCLUDED
#define CIRCLE_H_INCLUDED

typedef unsigned short ushort;

template <class DATA> class Circle
{
    public:
        Circle(ushort defDepth=6, ushort defSize=72)
        :depth(defDepth), size(defSize)
        {
            data = new DATA*[size];
            for(int i=0;i<size;i++)
                data[i] = new DATA;
        }

        ~Circle()
        {
            for(int i=0;i<size;i++)
                delete data[i];

            delete [] data;
        }


        DATA& operator[] (short pos)
        {
            if(pos>=0)
            {
                if(pos < size) // 0 to n-1
                    return *(data[pos]);
            }
            else
            {
                if(pos*-1 <= size) // -1 to -n
                    return *(data[pos + size]);
            }

        }

        ushort depth; // inches, just set it once so I don't have to be a douche about it
    private:
        DATA** data;
        ushort size;
};


struct VisHeader
{
    unsigned int RedCount;             //target color
    unsigned int YelCount;             //if adjacent to red is target, if only yel should avoid
    unsigned int GrnCount;             //clear color
    unsigned int WhtCount;             //line color
    unsigned int TotCount;             //total hits
};

#endif
