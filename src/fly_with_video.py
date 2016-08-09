#! /usr/bin/env python
# -*- coding: utf-8 -*-

import time
import sys
import thread
import ps_drone
import cv2

# ------------------------
# -- MARK: Constants
# ------------------------
EXIT_SUCCESS = 0
EXIT_FAILURE = 1

# ------------------------
# -- MARK: global config variables
# ------------------------
verbose = True

# ------------------------
# -- MARK: Helper Functions
# ------------------------
def log(msg):
    '''Prints a message if in verbose mode'''
    if verbose:
        print msg

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
def __initDrone():
    log("\033[0;34;47m")
    log("      -------------------====+====-------------------    ")
    log("                          {_| |_}                        ")
    log("                        /| _|_|_ |\                      ")
    log("                       ( |/_____\| )                     ")
    log("                    |--`/_/  |  \_\\'--|                  ")
    log("                ____   //( ) |    \\\\   ____              ")
    log("               | ++ |==|\___/ \___/|==| ++ |             ")
    log("                \__/   |  ___ ___  |   \__/              ")
    log("                      __\/oo X []\/__                    ")
    log("                     || [\__/_\__/] ||                   ")
    log("                    ~~~~           ~~~~                  ")
    log("")
    log("                 === Welcome to P3N15 ===                ")
    log("                  Automous Drone Flight                  \033[0m")
    log("")
    log("initiating startup procedure...")

    drone = ps_drone.Drone()				# Start using drone
    drone.startup()							# Connects to drone and starts subprocesses
    drone.reset()							# Always good, at start

    while drone.getBattery()[0] == -1:
        time.sleep(0.1)		                # Waits until the drone has done its reset
    time.sleep(0.5)							# Give it some time to fully awake

    log("done")
    log("Drone online! Battery: " +str(drone.getBattery()[0])+"%  Status: "+str(drone.getBattery()[1]))
    return drone

def __videoFeed(threadName, dely, drone):
    '''Starts video feed from the drone'''
    if drone == None:
        printRed("[Error] Drone not initialized")

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

    cv2.namedWindow( "Display");

    print 'ready!'

    imcount = drone.VideoImageCount
    time_tmp = time.time()
    while True:
        # wait for next frame
        while imcount == drone.VideoImageCount:
            time.sleep(0.01)
        imcount = drone.VideoImageCount

        t_now = time.time()
        t_diff = t_now - time_tmp
        time_tmp = t_now
        fps = (1 / t_diff) if t_diff > 0 else 0

        mat = drone.VideoImage;

        cv2.rectangle(mat,(180, 30), (0, 0), (0,0,0), -1)
        cv2.putText(mat,'{: <2} FPS'.format(fps),(10,20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(255,255,255),2)
        cv2.imshow("Display", mat)
        cv2.waitKey(1)


def __controlLoop(drone):
    '''Gives manual control of the drone to the user'''
    if drone == None:
        printRed("[Error] Drone not initialized")

    stop = False
    while not stop:
    	key = drone.getKey()
        if key == " ":
            if drone.NavData["demo"][0][2] and not drone.NavData["demo"][0][3]:
                drone.takeoff()
            else:
                drone.land()
        elif key == "0":	drone.hover()
        elif key == "w":	drone.moveForward()
        elif key == "s":	drone.moveBackward()
        elif key == "a":	drone.moveLeft()
        elif key == "d":	drone.moveRight()
        elif key == "q":	drone.turnLeft()
        elif key == "e":	drone.turnRight()
        elif key == "7":	drone.turnAngle(-10,1)
        elif key == "9":	drone.turnAngle( 10,1)
        elif key == "4":	drone.turnAngle(-45,1)
        elif key == "6":	drone.turnAngle( 45,1)
        elif key == "1":	drone.turnAngle(-90,1)
        elif key == "3":	drone.turnAngle( 90,1)
        elif key == "8":	drone.moveUp()
        elif key == "2":	drone.moveDown()
        elif key == "*":	drone.doggyHop()
        elif key == "+":	drone.doggyNod()
        elif key == "-":	drone.doggyWag()
        elif key == "y":    thread.start_new_thread(__videoFeed,("VideoFeed", 1, drone))
        elif key != "":		stop = True



def main():
    drone = __initDrone()
    __controlLoop(drone)        #hand manual control to the user

    sys.exit(EXIT_SUCCESS)

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
    main()
