# Domino Robot

This is the code for Mark Rober's Domino Robot project. You can find a bunch more info about the project, including details about the software architecture [on my website](https://www.baucomrobotics.com/domino-robot).

Here is the video that Mark made about the project:
[![Marks video](https://img.youtube.com/vi/8HEfIJlcFbs/0.jpg)](https://www.youtube.com/watch?v=8HEfIJlcFbs)

## Folder structure
If you are interested in browsing the files in this repo, here is the general structure to get you oriented.
- `doc`: Some high level documentation stuff, much of it is probably out of date
- `experimental_testing`: Various experimental files
- `src`: The real stuff
    - `master`: Code for running the master controller and GUI
    - `robot`: Code that runs on the Raspberry Pi of the robot
    - `robot_motor_driver`: Code that runs on the ClearCore of the robot
    - `tools`: Some various utility scripts

## Usage (Not Recommended)
This repository exists mostly for those who are interested in browsing the code to see how it works. It is almost certainly a bad idea to try and download this code and use it unless you have identical hardware to the robot in the video and/or really know what you are doing. You should consider this code completely unsupported as I do not plan to make future updates or fix any bugs you may find.

If for some reason you really want to try and run this code, there are some (out of date) instructions on how to run the [the master code](src/master/README.md) and also some (possibly also out of date) info on which libraries are used for compiling [the robot code](src/robot/README.md). Good luck.
