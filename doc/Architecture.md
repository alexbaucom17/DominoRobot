# Domino Robot Software Architecture

The high level software architecure can be seen in the following diagram (click to enlarge). More detailed explanaion of each module can be found below the diagram.

<img src="DominoRobotSoftwareArchitecture.png" width="600">

## Master
The master program is responsible for all of the high level planning of the domino field sends step-by-step instructions to the robot in order to place the dominos correctly in the field. It runs on a remote laptop that connects to the robot over wifi.

### Field Planner
The `FieldPlanner` takes in an input image that we want the final field to look like. It then generates the domino field by scaling the image to the appropriate size, matching colors in the image with the available domino colors we have, and splitting the image up into 15x20 'tiles' that represent the group of dominos the robot can put down at once. These tiles are then sequenced such that the robot never has to drive near the tiles for very long and the next tile location the robot has to get to is always accessible. Finally, for each tile a sequence of 'actions' is generated which are used as the primary means of communicating with and controlling the robot. These actions are things like Load Dominos, Move to X Position, Move Using Vision, Place Dominos, Request Status, etc. Theses action sequences form cycles where each cycle places one tile into the field. All the cycles are essentially identical other than the positions the robot moves to and this group of cycles can be saved and loaded as a 'plan'.

### GUI and Runtime Manager
Once there is a plan, the main master program runs and is responsible for handling a few things: Monitoring and communicating with the robot, monitoring and executing the plan, and providing a GUI for the user to interface with the robot and the plan. The GUI is a relatively simple UI that has buttons for sending commands and modifying the plan, status outputs for the robot and plan, and a visualization of the robot position in the world. The `RuntimeManager` handles the plan execution by iterating through cycles and actions, sending each action to the robot, and then waiting for confirmation that the action is completed. The `RuntimeManager` can hold a collection of interfaces to various robots (which are mostly just a few layers of wrappers around a TCP socket) as the original plan was to have the master software interface with multiple robots and possibly multiple base stations, but in the end it was scaled back to just a single robot. 

### Robot Client
The master-robot communication protocol works sort of like a client-server protocol, but slightly worse. The robot acts as a server where it waits for actions to be sent to it. It will acknowledge the action request and then process to do it. The master then sends regular status requests to the robot, which will respond with the current status, including the state of the action. This allows for the communication handling to be a bit simpler as the master always sends one message and receives exactly one message back immediately after. There is no need to handle asynchronous communication which greatly simplifies the code.

## Robot
The robot software is the most complex part of the system and it runs on the Raspberry Pi onboard the robot. Most objects on the robot operate in a cooperative multitasking mode where non of them block for any significant amount of time. Methods that do block or take a long time are run in separate threads. The top level robot object implements a simple loop that checks for a new command, starts a new command if needed, updates the currently running command, and checks if the current command is completed.

### Robot Server
The `RobotServer` is responsible for all the communication with the Master software. Internally, it uses a multi-threaded wrapper around a TCP socket to avoid blocking for server activity. The server parses incoming packets into JSON and then checks for a `type` field which must be present in all commands to indicate the type of the command. Certain commands also contain additional data from Master such as a position or status. The Robot Server then returns the command type and any additional data so that the correction actions can be taken for the given command.

### Status Updater
The `StatusUpdater` maintains the current status of the robot and many other classes take this object by reference so that they can all update the global status. When the Master software requests the robot status, this module serializes the status to JSON and returns it back to the Master.

### Tray Controller
The `TrayController` provides an interface for controlling the tray to load and place dominos. It maintains small state machines for each of the tray actions (PLACE, LOAD, and TRAY_INIT) and sequences individual subcommands (such as moving the lifter to a specific spot, waiting for some amount of time, or opening or closing the latch that holds the dominos) to send to the Motor Driver over a USB Serial connection.

### Marvelmind Wrapper
The `MarvelmindWrapper` handles interfacing with the Marvelmind Library and gets the current position from the marvelmind sensors. It then provides this information to the top level robot object to route to the correct place.

### Camera Tracker
The `CameraTracker` is responsible for tracking the position of the robot computed from the markers on the ground. Within the `CameraTracker`, two `CameraPipeline` objects run camera processing for each of the cameras. Each `CameraPipeline` spawns a thread for the camera processing because processing each image takes 100 ms. The image processing does some initial filtering and thresholding and then uses blob detection to find the marker in the image. The marker is then converted from image coordinates to physical coordinates by using the intrinsic and extrinsic properties of the camera. When the marker pose is returned from each camera for the current image, the `CameraTracker` computes an estimate of the robot pose relative to the marker positions.

### Robot Controller
The `RobotController` handles all of the motion control and driving of the robot. It maintains global localization using odometry and position information from the Marvelmind sensors. It also can instantiate different control modes for different types of movement. Each control mode generates a command velocity at each time step and implements its own check for trajectory completion. The command velocity at each time step is sent to the motor controller over a USB Serial connection.

#### Global Localization
Global localization is handled by a Kalman Filter that fuses local odometry from the motors and global position estimates from the Marvelmind sensors. The Kalman Filter itself is very simple as it only maintains the 3x3 position estimate but there is a little bit of extra work that goes into processing the position update step from the Marvelmind. Since the Marvelmind readings can lag when the robot is moving, the estimated variance of the measurement is scaled with the velocity of the robot. This is not an accurate model as the variance is not truly Gaussian, but it works well enough.

#### Position Control Mode
The Position Control Mode handles most of the driving modes for the robot. It generates a point to point trajectory and then uses simple PID controllers along with feedback from the localization system to generate velocity commands for each axis that are then sent to the motor controller.

#### Vision Control Mode
The Vision Control Mode is used for all motions where the markers on the floor need to be tracked. It does not use the global localization system and instead only considers the relative displacement of the markers from their target. In practice, this is still handled by computing an 'estimated pose' of the robot so that the trajectory generation system can work in the same fashion, but in essence it is just driving the markers in the image to a target location. This control mode also handles its own 'localization' using a second Kalman Filter that combines the wheel odometry with the 'position estimates' provided by the `CameraTracker`. Similarly to the Position Control Mode, it uses simple PID controllers to generate velocity commands for each axis to send to the motor controller.

## Motor Driver
The Motor Driver runs on a Teknic ClearCore which was generously provided by Teknic for the project (along with the drive motors for the base). This board functions very much like an 'Industrial Arduino' so much so that Teknic actually provides an Arduino-Compatible library so it can be programmed directly from the Arduino IDE. This board is responsible for all of the real time driving of the motors as well as the servo that drops the dominos and the optical endstop for the lifter motor.

### Lifter Action Handler
Communication with the Raspberry Pi that is specific to the lifter gets routed to the lifter action handler which has simple state machines for operating the servo, moving the lifter to a specific height, and homing the lifter. 

### Robot Velocity Handler
Velocity commands for the base are routed to the robot velocity handler which takes in a cartesian velocity in the local robot frame, converts it to wheel speeds, and then updates the commanded velocity for each motor. It also returns an odometry estimate from the motor, but due to how the ClearPath motors work, this isn't actually a true encoder reading from the motors. However, the motors themselves are extremely accurate and follow the velocity commands extremely well so not reading directly from an encoder is not really a huge problem.