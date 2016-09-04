# p3n15

The project makes use of the PS-Drone library (http://www.playsheep.de/drone/), pygame (http://www.pygame.org/hifi.html) and OpenCV (http://opencv.org/) to control the AR.Drone 2.0.
If a 'male' symbol is detected in the image of the front camera of the drone, the drone tries to follow it. If the symbol is near the edges of the image the drone turns in order to center it. If the image box is to small / big in comparison to the predefined size the drone moves forward / backward.

## Requirements

To run this project the following libraries are required:

* Python 2.7
* OpenCV with Python bindings
* Pygame
* PS-Drone

*Note: We were not able to run all libraries on either MacOS Sierra or Windows 10. Installing Ubuntu on a Virtual Machine provided a swift solution for setting up a suitable development environment.*

## Usage
With p3n15 implementation prioritises the safety of the user and the environment. Hence, manual control is always active and will override any autonomous flight commands. The drone automatically starts in the manual mode.

To start the drone, connect to it's WiFi and start the p3n15.py script.

### Manual Controls
To steer the drone, make sure the pygame window with the video feed is active.
### Autonomous Flight

## Issues
During development we faced several time and/or library related issues which are described in the following.

### pygame vs PS-Drone
In order to steer the drone manually we used pygame to detect keydown and keyup events. We faced the issue that initalising pygame before ps-drone rendered certain functions of the ps-drone library useless. Especially switching to the bottom facing camera was not possible then.

### Drone Flight Precision
Our original approach was to use the bottom facing camera of the drone to detect markers on the floor. These markers should encode the direction of the next marker. When a marker is detected the drone should hover and turn above the marker and continue the flight in the encoded direction.
However, it was not possible to rotate the drone while keeping the marker centered. Turning always created a slight drift, moving the marker out of the picture. Paired with the small resolution of the bottom facing camera it was not possible to develop a working solution in the given time.

### PID Controller
A PID Controller (Proportional–Integral–Derivative Controller) is a control loop feedback mechanism taking error terms into account to steer the drone in autonomous flight. The controller for our final track-and-follow solution was developed in a rapid manner as we concluded our original approach was feasible the night before the 'final race'. As such, it is rather a 'P Controller' supporting only turning and forward / backward movement.
*Note: Starting the autonomous flight in the required height is critical, if one wants to fly through objects such a soccer goals. ;-)*

### Symbol Detection Reliability
Contrary to most other teams, we decided to implement detection algorithm for a custom symbol. As we wanted to encode a direction we decided to use the 'male' symbol printed on a white DIN-A4 sheet as our marker. Factors influencing the detection reliability are the contrast between the background (dark) and DIN-A4 sheet (white) with the symbol printed on it (dark). Only white areas close to the DIN Format are processed.