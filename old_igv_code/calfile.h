#pragma once
#ifndef CALFILE_H
#define CALFILE_H

//#include <stdio.h>

typedef struct
{
	float huemin;
	float huemax;
	float satmin;
	float satmax;
	float valmin;
	float valmax;
} calStructure;

void writeCalFile(float huemin, float huemax, float satmin, float satmax, float valmin, float valmax);

void readCalFile(float* huemin, float* huemax, float* satmin, float* satmax, float* valmin, float* valmax);

#endif
