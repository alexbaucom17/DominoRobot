Required header libraries at src/robot/lib
- ArduinoJson
- Catch (for tests)
- Eigen
- plog
- marvelmind library, compiled as .a - https://marvelmind.com/download/ I used 2020_04_10_C_example. This required the addition of `#include <pthread.h>` in `marvelmind.h` to compile correctly
- tysik/kalman_filters from https://github.com/tysik/kalman_filters

Required system libraries at /usr/local/include
- libSerial: https://github.com/crayzeewulf/libserial (have to build from source due to outdated apt package)

Required libraries at /usr/local/lib
- libserial.a

Required libraries installed with apt-get
 - libconfig++-dev
 - libarmadillo-dev