# Domino Robot Software Architecture

The high level software architecure can be seen in the following diagram (click to enlarge). More detailed explanaion of each module can be found below the diagram.

<img src="DominoRobotSoftwareArchitecture.png" width="600">

## Master
The master software is responsible for all of the high level planning of the domino field sends step-by-step instructions to the robot in order to place the dominos correctly in the field. 

The FieldPlanner takes in an input image that we want the final field to look like and generates the domino field by scaling the image to the appropriate size, matching colors in the image with the available domino colors we have, and splitting the image up into 15x20 'tiles' that represent the group of dominos the robot can put down at once. These tiles are then sequenced such that the robot never has to drive near the tiles for very long and the next tile to put down is always accessible by the robot. Finaly, for each tile a sequence of 'actions' is generated which are used as the primary means of communicating with and controlling the robot. These actions are things like Load Dominos, Move to X Position, Move Using Vision, Place Dominos, Request Status, etc. Theses action sequences form cycles where each cycle places one tile into the field. All the cycles are essentially identical other than the positions the robot moves to and this group of cycles can be saved and loaded as a 'plan'.

Once there is a plan, the main master program runs and is responsible for handling a few things: Monitoring and communicating with the robot, monitoring and executing the plan, and providing a GUI for the user to interface with the robot and the plan. The GUI is a relatively simple UI that has buttons for sending commands and modifying the plan, status outputs for the robot and plan, and a visualization of the robot position in the world. The RuntimeManager handles the plan execution by iterating through cycles and actions, sending each action to the robot, and then waiting for confirmation that the action is completed. The RuntimeManager can hold a collection of interfaces to various robots (which are mostly just a few layers of wrappers around a TCP socket) as the original plan was to have the master software interface with multiple robots and possibly multiple base stations, but in the end it was scaled back to just a single robot. 

The master-robot communcation protocol works sort of like a client-server protocol, but slightly worse. The robot acts a server where it waits for actions to be sent to it. It will acknowledge the action request and then process to do it. The master then sends regular status requests to the robot, which will respond with the current status, including the state of the action. This allows for the communication handling to be a bit simpler as the master always sends one message and recieves exactly one message back immediately after. There is no need to handle asynchronous communication which greatly simplifies the code.

## Robot
Fill in

## Motor Driver
Fill in