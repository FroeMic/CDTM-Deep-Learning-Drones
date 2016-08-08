#! /usr/bin/env python
# -*- coding: utf-8 -*-

import time
import ps_drone

drone = ps_drone.Drone()
drone.startup()
drone.reset()

while (drone.getBattery()[0] == -1):
    time.sleep(0.1)
print "Battery: "+str(drone.getBattery()[0])+"%  "+str(drone.getBattery()[1])

drone.useDemoMode(True)
drone.setConfigAllID()
drone.sdVideo()
drone.frontCam()
CDC = drone.ConfigDataCount
while CDC == drone.ConfigDataCount:
    time.sleep(0.0001)
#drone.slowVideo()
#drone.mp4Video()
drone.videoFPS(60)
drone.videoBitrate(20000)
drone.startVideo()
drone.showVideo()

print 'ready!'

imcount = drone.VideoImageCount
while True:
    try:
        # wait for next frame
        while imcount == drone.VideoImageCount:
            time.sleep(0.01)
        imcount = drone.VideoImageCount

        print 'frame {}'.format(imcount)
    except KeyboardInterrupt:
        exit(0)
