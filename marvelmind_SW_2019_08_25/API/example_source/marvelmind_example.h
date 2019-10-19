#ifndef __MARVELMIND_EXAMPLE_H_
#define __MARVELMIND_EXAMPLE_H_

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include "marvelmind_api.h"

bool marvelmindCheckVersionCommand(char *token);
bool marvelmindCheckWakeCommand(char *token1, char *token2);
bool marvelmindCheckSleepCommand(char *token1, char *token2);
bool marvelmindCheckDefaultCommand(char *token1, char *token2);
bool marvelmindCheckTelemetryCommand(char *token1, char *token2);
bool marvelmindCheckSubmapCommand(char *token1, char *token2, char *token3);
bool marvelmindCheckMapCommand(char *token1, char *token2);
bool marvelmindCheckRateCommand(char *token1, char *token2, char *token3);
bool marvelmindCheckUltrasoundCommand(char *token1, char *token2, char *token3);
bool marvelmindCheckAxesCommand(char *token1, char *token2, char *token3, char *token4);

void marvelmindCycle();
void marvelmindStart();
void marvelmindFinish();

#endif // __MARVELMIND_EXAMPLE_H_
