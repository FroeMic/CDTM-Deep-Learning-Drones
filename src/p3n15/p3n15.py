import time
import sys
import threading
import cv2
import pygame
import numpy

from ps_drone import Drone
from flightController import PenisController as controller


# =======================================
#     CDTM AUTONOMOUS DRONE ELECTIVE
#              August 2016
# =======================================

# ------------------------
# -- MARK: Drone Class
# ------------------------
class P3N15(Drone):

    FRONTCAM = 0
    GROUNDCAM = 1

    def __init__(self):
        Drone.__init__(self)
        self.flightController = controller(self)
        self.camera = P3N15.FRONTCAM
        self.outputImage = None
        self.__nativeImage = True
        self.__printBanner()

    # ----------------------
    # -- Private Methods
    # ----------------------
    def __printBanner(self):
        '''prints a banner'''
        print("\033[0;34;47m")
        print("")
        print("      -------------------====+====-------------------    ")
        print("                          {_| |_}                        ")
        print("                        /| _|_|_ |\                      ")
        print("                       ( |/_____\| )                     ")
        print("                    |--`/_/  |  \_\\'--|                  ")
        print("                ____   //( ) |    \\\\   ____              ")
        print("               | ++ |==|\___/ \___/|==| ++ |             ")
        print("                \__/   |  ___ ___  |   \__/              ")
        print("                      __\/oo X []\/__                    ")
        print("                     || [\__/_\__/] ||                   ")
        print("                    ~~~~           ~~~~                  ")
        print("")
        print("                 === Welcome to P3N15 ===                ")
        print("              Potent Autonomous Drone Flight             ")
        print("\033[0m")

    # ----------------------
    # -- Public Methods
    # ----------------------
    def startup(self):
        '''configures and starts the drone'''
        Drone.startup(self)						# Connects to drone and starts subprocesses
        Drone.reset(self)						# Always good, at start

        while (Drone.getBattery(self)[0] == -1):
            time.sleep(0.1)		                # Waits until the drone has done its reset
        print "=>", Drone.getBattery(self)
        time.sleep(0.5)							# Give it some time to fully awake


    def  shutdown(self):
        ''' shutsdown the drone and ends all threads'''
        self.endAutonomousFlight()
        Drone.shutdown(self)

    def initCamera(self, camera = FRONTCAM):
        Drone.setConfigAllID(self)                                  # Go to multiconfiguration-mode

        Drone.sdVideo(self)                                         # Choose lower resolution (hdVideo() for...well, guess it)
        Drone.saveVideo(self,False)

        if camera == P3N15.FRONTCAM:
            self.camera = camera
            Drone.frontCam(self)
        elif camera == P3N15.GROUNDCAM:
            self.camera = camera
            Drone.groundCam(self)
        else:
            print "[Error] Invalid camera (%d) specified." %(camera)
            return

        CDC = self.ConfigDataCount
        while CDC == self.ConfigDataCount:
            time.sleep(0.0001)                                      #Wait until it is done (after resync is done)

        Drone.startVideo(self)                                      # Start video-function

    def toggleCamera(self):
        if self.camera == P3N15.FRONTCAM:
            self.camera = P3N15.GROUNDCAM
            self.groundCam()
        elif self.camera == P3N15.GROUNDCAM:
            self.camera = P3N15.FRONTCAM
            self.frontCam()

    def getVideoFrame(self):
        if (self.__nativeImage or self.outputImage is None):
            return self.VideoImage
        else:
            return self.outputImage


    def startAutonomousFlight(self):
        '''Hands control to the flight controller'''
        if self.flightController == None:
            print("[Error] Cannot start autonomous flight. No flight controller specified!")
            return
        else:
            print "Start Thread"
            self.flightController = self.flightController.newInstance()
            self.flightController.start()
            self.__nativeImage = False

    def endAutonomousFlight(self):
        '''Ends the autonomous flight by terminating the flight controller'''
        self.__nativeImage = True
        self.outputImage = None
        if self.flightController != None:
            self.flightController.terminate()



# ------------------------
# -- MARK: Main Program Cycle
# ------------------------
def run(drone, cycleTime = 0.01):
    SCALE = 1
    # init pygame
    pygame.init()
    pygame.display.set_caption("P3N15 Demo")
    screen = pygame.display.set_mode((640*SCALE, 360*SCALE))
    screen = pygame.display.get_surface()

    # ==> Action!
    IMC =    drone.VideoImageCount                               # Number of encoded videoframes
    stop =   False

    while True:
        if handleEvents(drone):
            break

        if drone.VideoImageCount != IMC:                        # no new frame available
            IMC = drone.VideoImageCount
            updateUI(drone, screen)

        time.sleep(cycleTime)

    # Goodbye
    print "=> Goodbye"
    drone.shutdown()
    pygame.quit()


def updateUI(drone, screen):
    frame = prepareFrame(drone)
    screen.blit(frame, (0,0))
    pygame.display.flip()

def prepareFrame(drone):
    frame = drone.getVideoFrame()
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    cv2.rectangle(frame,(240, 30), (0, 0), (0,0,0), -1)
    cv2.putText(frame,"P3N15: Command Interface",(10,20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(255,255,255),2)

    frame = numpy.rot90(frame)
    frame = numpy.flipud(frame)
    return pygame.surfarray.make_surface(frame)

def handleEvents(drone):
    finished = False

    for event in pygame.event.get():
        # exit if necessary
        if (event.type == pygame.QUIT or
            (event.type == pygame.KEYUP and event.key == pygame.K_ESCAPE)):
            return True

        # handle keyup
        if event.type == pygame.KEYUP:
            handleKeyUp(event.key, drone)
        # handle keydown
        elif event.type == pygame.KEYDOWN:
            drone.endAutonomousFlight()
            handleKeyDown(event.key, drone)

        # handle turns seperately
        handleTurns(drone)

    return finished

def handleKeyUp(key, drone):
    if key == pygame.K_ESCAPE:
        drone.land()
        finished = True
    elif key == pygame.K_SPACE:
        if drone.NavData["demo"][0][2] and not drone.NavData["demo"][0][3]:
            drone.takeoff()
        else:
            drone.land()
    elif key == pygame.K_p:
        drone.startAutonomousFlight()
    else:
        drone.hover()

def handleKeyDown(key, drone):
    if key == pygame.K_w:
        drone.moveForward()
    elif key == pygame.K_s:
        drone.moveBackward()
    elif key == pygame.K_a:
        drone.moveLeft()
    elif key == pygame.K_d:
        drone.moveRight()
    elif key == pygame.K_UP:
        drone.moveUp(1)
    elif key == pygame.K_DOWN:
        drone.moveDown(1)
    elif key == pygame.K_c:
        drone.toggleCamera()
    elif key == pygame.K_m:
        filename = "img/" + str(drone.VideoDecodeTimeStamp) + '.png'
        cv2.imwrite(filename, drone.VideoImage)
        print "saved {}".format(filename)

def handleTurns(drone):
    keys = pygame.key.get_pressed()
    if keys[pygame.K_q]:
        drone.turnLeft(1.0)
    elif keys[pygame.K_e]:
        drone.turnRight(1.0)

if __name__ == '__main__':
    drone = P3N15()
    drone.startup()
    drone.useMDemoMode(False)
    drone.initCamera(camera = P3N15.GROUNDCAM)
    run(drone)
