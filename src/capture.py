##! /usr/bin/env python
# -*- coding: utf-8 -*-

import time, sys
import cv2
import ps_drone                                              # Import PS-Drone

drone = ps_drone.Drone()                                     # Start using drone
drone.startup()                                              # Connects to drone and starts subprocesses

drone.reset()                                                # Sets drone's status to good (LEDs turn green when red)
while (drone.getBattery()[0] == -1):      time.sleep(0.1)    # Waits until drone has done its reset
print "Battery: "+str(drone.getBattery()[0])+"%  "+str(drone.getBattery()[1])	# Gives a battery-status
drone.useDemoMode(True)                                      # Just give me 15 basic dataset per second (is default anyway)

##### Mainprogram begin #####
drone.setConfigAllID()                                       # Go to multiconfiguration-mode
drone.sdVideo()                                              # Choose lower resolution (hdVideo() for...well, guess it)
drone.saveVideo(False)
drone.groundCam()                                            # Choose front view
CDC = drone.ConfigDataCount
while CDC == drone.ConfigDataCount:       time.sleep(0.0001) # Wait until it is done (after resync is done)
drone.startVideo()                                           # Start video-function

##### And action !
print "Use <space> to capture groundcamera frame, any other key to stop"
IMC =    drone.VideoImageCount                               # Number of encoded videoframes
stop =   False

cv2.namedWindow("frame", cv2.WINDOW_NORMAL)

while not stop:
    while drone.VideoImageCount == IMC: time.sleep(0.01)     # Wait until the next video-frame
    IMC = drone.VideoImageCount
    cv2.imshow("frame", drone.VideoImage)
    key = cv2.waitKey(1) & 0xFF                                   # Gets a pressed key
    if key==ord(" "):
        filename = "img/" + str(drone.VideoDecodeTimeStamp) + '.png'
        cv2.imwrite(filename, drone.VideoImage)
        print "saved {}".format(filename)
    elif key != 0xFF:
        stop = True
cv2.destroyAllWindows()
