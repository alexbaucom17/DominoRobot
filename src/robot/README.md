Required header libraries at src/robot/lib
- ArduinoJson
- Catch (for tests)
- Eigen
- plog
- marvelmind_api, compiled as .a - https://marvelmind.com/download/ 

Required system libraries at /usr/local/include
- libSerial: https://github.com/crayzeewulf/libserial (have to build from source due to outdated apt package)

Requred libraries at /usr/local/lib
- libdashapi.so - https://marvelmind.com/download/ 
- libserial.a

Required libraries installed with apt-get
 - libconfig++-dev