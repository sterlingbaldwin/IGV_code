#include "globals.h"

/* ********************************** */
/* Begin Global Function Declarations */
/* ********************************** */

void delay(int dseconds)
{
	time_t dstart_time, dcur_time;

	time(&dstart_time);
	do
	{
		time(&dcur_time);
	}
	while ((dcur_time - dstart_time) < dseconds);
}

void mdelay(unsigned int mseconds)
{
	DWORD start, current;

	start = GetTickCount();
	do
	{
		current = GetTickCount();
	} while ((current - start) < mseconds);
}

float sqr(float x)
{
	return x*x;
}

/* ******************************** */
/* End Global Function Declarations */
/* ******************************** */
