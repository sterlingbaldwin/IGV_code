#if 1
#include "calfile.h"

//#include "globals.h"
#include <fstream>

void writeCalFile(float huemin, float huemax, float satmin, float satmax, float valmin, float valmax)
{
	calStructure calInfo = {huemin, huemax, satmin, satmax, valmin, valmax};
	FILE *calFile;
	calFile = fopen("f:\\calfile.dat", "wb");

	fwrite(&calInfo, sizeof(calInfo), 1, calFile);

	fclose(calFile);
	return;
}

void readCalFile(float* huemin, float* huemax, float* satmin, float* satmax, float* valmin, float* valmax)
{
	calStructure calInfo;
	FILE *calFile;
	calFile = fopen("f:\\calfile.dat", "rb");

	fread(&calInfo, sizeof(calInfo), 1, calFile);
	fclose(calFile);

	*huemin = calInfo.huemin;
	*huemax = calInfo.huemax;
	*satmin = calInfo.satmin;
	*satmax = calInfo.satmax;
	*valmin = calInfo.valmin;
	*valmax = calInfo.valmax;

	return;
}
#endif
