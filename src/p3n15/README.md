# === P3N15 ===

P3N15 makes use of the PS-Drone library (http://www.playsheep.de/drone/), pygame (http://www.pygame.org/hifi.html) and OpenCV (http://opencv.org/) to control the AR.Drone 2.0.
When a marker is detected in the image of the front camera, the drone tries to follow it. If the symbol is near the edges of the image the drone turns in order to center it. If the image box is to small / big in comparison to the predefined size the drone moves forward / backward.

**Note:** To download the files we recommend **NOT** to checkout the entire repository as it is about 400 megabytes in size (we added a lot of training images). Rather try one of the following:

Either clone the latest commit (still 140 megabytes)
```
git clone --depth=1 https://github.com/FroeMic/CDTM-Deep-Learning-Drones.git
```
or **just download the src directory using svn**
```
svn checkout https://github.com/FroeMic/CDTM-Deep-Learning-Drones/trunk/src
```

## 1.0 Requirements

To run this project the following libraries are required:

* Python 2.7
* OpenCV with Python bindings
* pygame
* PS-Drone

**Note:** *We were not able to run all libraries on either MacOS Sierra or Windows 10. Installing Ubuntu on a Virtual Machine provided a swift solution for setting up a suitable development environment.*

## 2.0 Usage
The P3N15 implementation prioritises the safety of the user and the environment. Therefore, manual control is ALWAYS active and will override any autonomous flight commands. The drone automatically starts in the manual mode.

To start the drone, connect to it's WiFi and start the p3n15.py script. Once you see the battery status of the drone it is ready to operate.

### 2.1 Manual Controls
To steer the drone, make sure the pygame window with the video feed is active.

| Key          | Action          |
| ------------ |---------------- |
| SPACE        | start / land    |
| ESC          | shutdown        |
| C            | toggle camera   |
| M            | make screenshot |
| P            | autonomous mode |
| W            | forward         |
| S            | backward        |
| A            | left            |
| D            | right           |
| Q            | rotate left     |
| E            | rotate right    |
| UP           | move up         |
| DOWN         | move down       |

### 2.1 Autonomous Flight
Press **P** to switch into autonomous mode. The drone will analyse the image for a marker and follow it using a simple track and follow approach.

**Note:** *Pressing any key during autonomous flight will switch to manual mode.*

#### 2.1.1 Using a different Controller
P3N15 is designed to easily switch the controller handling the autonomous flight. Controllers are implemented in  flightController.py and should be subclasses of the FlightController class defined there.

To change the controller used for autonomous flight simply import the desired controller at the beginning of the p3n15.py file.
```Python
from flightController import P3N15Controller as controller
```

## 3.0 Issues
During development we faced several time and/or library related issues which are described in the following. Please be aware that the entire project was done in a hackathon style manner with no extensive testing. The following list of issues might not be complete.

### 3.1 pygame vs PS-Drone
In order to steer the drone manually we used pygame to detect keydown and keyup events. We faced the issue that initalising pygame before ps-drone rendered certain functions of the ps-drone library useless. Especially switching to the bottom facing camera was not possible then. Additionally, we could not access the raw nav-data of the drone (probably) due to this issue.

### 3.2 Drone Flight Precision
Our original approach was to use the bottom facing camera of the drone to detect markers on the floor. These markers should encode the direction of the next marker. When a marker is detected the drone should hover and turn above the marker and continue the flight in the encoded direction.
However, it was not possible to rotate the drone while keeping the marker at the center of the picture. Turning always created a slight drift, moving the marker out of the picture. Paired with the small resolution of the bottom facing camera it was not possible to develop a working solution in the given time.

### 3.3 PID Controller
A PID Controller (Proportional–Integral–Derivative Controller) is a control loop feedback mechanism taking error terms into account to steer the drone in autonomous flight. The controller for our final track-and-follow solution was developed in a rapid manner as we concluded our original approach was not feasible the night before the 'final race'. As such, it is rather a 'P Controller' and only supports turning and forward / backward movement.

**Note:** *Starting the autonomous flight in the required height is critical, if one wants to fly through objects such a soccer goals. ;-)*

### 3.4 Symbol Detection Reliability
Contrary to most other teams, we decided to implement detection algorithm for a custom symbol. As we wanted to encode a direction we decided to use the 'male' symbol printed on a white DIN-A4 sheet as our marker. Factors influencing the detection reliability are the contrast between the background (dark) and DIN-A4 sheet (white) with the symbol printed on it (dark). Only white areas close to the DIN Format are processed.
