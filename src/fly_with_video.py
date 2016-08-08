#! /usr/bin/env python
# -*- coding: utf-8 -*-

import time
import ps_drone
import cv2

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
drone.videoFPS(10)
#drone.videoBitrate(20000)
drone.startVideo()
#drone.showVideo()

cv2.namedWindow( "Display");

print 'ready!'

imcount = drone.VideoImageCount
time_temp = time.time()
while True:
    try:
        # wait for next frame
        while imcount == drone.VideoImageCount:
            time.sleep(0.01)
        imcount = drone.VideoImageCount

        t_now = time.time()
        t_diff = t_now - time_temp
        time_temp = t_now
        fps = (1 / t_diff) if t_diff > 0 else 0
        print 'frame {}. {: <2} FPS'.format(imcount, fps)

        mat = drone.VideoImage;
        cv2.imshow("Display", mat)
        cv2.waitKey(1)

    except KeyboardInterrupt:
        drone.shutdown()
        cv2.destroyAllWindows()
        exit(0)
