#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "marvelmind_example.h"
#include "marvelmind_utils.h"
#ifdef WIN32
#include <conio.h>
#else
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#endif

#ifndef WIN32
int kbhit(void)
{
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if(ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
}
#endif

// Check input command
static void checkCommand() {
    #ifdef WIN32
    if (!kbhit()) return;
    if (getch() != ' ') return;
    #else
    if (!kbhit()) return;
    if (getchar() != ' ') return;
    #endif

    char str [80];

    printf("Enter command: ");
    fgets(str, 79, stdin);

    char *token1 = strtok(str, " ");
    trim(token1);
    if (strcmp(token1,"quit") == 0) {
        exit(0);
        return;
    }

    char *token2 = strtok(NULL, " ");
    char *token3 = NULL;
    if (token2 != NULL) {
        trim(token2);

        token3 = strtok(NULL, " ");
        if (token3 != NULL) {
            trim(token3);
        }
    }

    char *token4 = NULL;
    if (token3 != NULL) {
        trim(token3);

        token4 = strtok(NULL, " ");
        if (token4 != NULL) {
            trim(token4);
        }
    }

    if (marvelmindCheckVersionCommand(token1)) {
        return;
    }

    if (marvelmindCheckWakeCommand(token1, token2)) {
        return;
    }

    if (marvelmindCheckSleepCommand(token1, token2)) {
        return;
    }

    if (marvelmindCheckDefaultCommand(token1, token2)) {
        return;
    }

    if (marvelmindCheckTelemetryCommand(token1, token2)) {
        return;
    }

    if (marvelmindCheckSubmapCommand(token1, token2, token3)) {
        return;
    }

    if (marvelmindCheckMapCommand(token1, token2)) {
        return;
    }

    if (marvelmindCheckRateCommand(token1, token2, token3)) {
        return;
    }

    if (marvelmindCheckUltrasoundCommand(token1, token2, token3)) {
        return;
    }

    if (marvelmindCheckAxesCommand(token1, token2, token3, token4)) {
        return;
    }
}

int main()
{
    marvelmindStart();

    while(1) {
        marvelmindCycle();

        sleep_ms(1);

        checkCommand();
    }

    marvelmindFinish();

    return 0;
}
