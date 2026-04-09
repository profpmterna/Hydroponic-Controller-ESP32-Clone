#ifndef LASER_TOF_MANAGER_H
#define LASER_TOF_MANAGER_H

#include "define.h"

void laserInit();
void laserUpdate();
void laserReset();


extern float g_laserStdDev;
extern float g_laserHealthPct;
extern bool laserEnabled;
extern bool g_laserReflectionFlag;
extern float g_laserCorrectedDistCm;

// FreeRTOS task function
void laserTask(void *parameter);

#endif