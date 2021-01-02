# Domino Robot

**WORK IN PROGRESS**

This is a project that uses a robot to setup a field of dominos. 

## Folder structure

- `doc`: Some high level documentation stuff, some of it is probably out of date
- `experimental_testing`: Various experimental files
- `src`: The real stuff
    - `base_station`: Code for the domino distribution station
    - `esp_wifi_server`: Code for an ESP8266 chip to make a passthrough wifi server to communicate with Arduinos over wifi
    - `master`: Code for running the master controller and GUI
    - `robot`: Code that runs on the Raspberry Pi of the robot
    - `robot_motor_driver`: Code that runs on the ClearCore of the robot
    - `tools`: Some various utility scripts

## Usage

Check out the [README for the master code](src/master/README.md) for installaiton and usage instructions
