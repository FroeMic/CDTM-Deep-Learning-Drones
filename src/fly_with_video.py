#! /usr/bin/env python
# -*- coding: utf-8 -*-

import time
import sys
import ps_drone
import cv2

# ------------------------
# -- MARK: Constants
# ------------------------
EXIT_SUCCESS = 0
EXIT_FAILURE = 1

# ------------------------
# -- MARK: Helper Functions
# ------------------------
def printRed(msg):
    '''Prints a message in red font'''
    print "\x1b[0;31m" + msg + "\x1b[0m"

def printBlue(msg):
    '''Prints a message in blue font'''
    print "\x1b[0;34m" + msg + "\x1b[0m"

def raiseError(msg, fatal = False):
    '''Prints an error and exits the program if it was fatal'''
    printRed("[Error] " + msg)
    if fatal:
        printRed("[FATAL] Terminating now!")
        sys.exit(EXIT_FAILURE)


# ------------------------
# -- MARK: Main Program
# ------------------------
def controlLoop(drone):
    if drone == None:
        printRed("[Error] Drone not initialized")


def main():
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

if __name__ == '__main__':
    raiseError("No data specified", fatal=True)
    printBlue("Test2")
    #main()
