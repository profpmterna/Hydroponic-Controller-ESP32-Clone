#ifndef BACKEND_MANAGER_H
#define BACKEND_MANAGER_H

#include <Arduino.h>

void backendInit();
void backendTask(void *parameter);
void backendSendStatus();
bool isBackendConnected();


#endif