# **Warning! This is probably out of date and might be inaccurate. Use at your own risk.**

## Libraries needed for installation on Raspberry Pi
Required header libraries at src/robot/lib
- ArduinoJson
- Catch (for tests)
- Eigen
- plog
- marvelmind library, compiled as .a - https://marvelmind.com/download/ I used 2020_04_10_C_example. This required the addition of `#include <pthread.h>` in `marvelmind.h` to compile correctly

Required system libraries at /usr/local/include
- libSerial: https://github.com/crayzeewulf/libserial (have to build from source due to outdated apt package)

Required libraries at /usr/local/lib
- libserial.a
- opencv: (following instructions here: https://docs.opencv.org/master/d7/d9f/tutorial_linux_install.html and fix from here: https://github.com/cggos/DIPDemoQt/issues/1)

Required libraries installed with apt-get
 - libconfig++-dev

 ## Build and run
 Go to `DominoRobot/src/robot` folder and run `make` to build. You can build tests with `make test` and remove all outputs with `make clean`.

 Run main or test using `build/robot-main` or `build/robot-test` respectively. The Raspberry Pi on the robot has these aliased to `run_robot` and `run_test` for ease of use.