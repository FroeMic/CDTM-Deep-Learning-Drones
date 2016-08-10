import time
import sys
import threading
import cv2
import pygame
import numpy
from ps_drone import Drone

# =======================================
#     CDTM AUTONOMOUS DRONE ELECTIVE
#              August 2016
# =======================================

# MARK Drone Class
class P3N15(Drone):

    FRONTCAM = 0
    GROUNDCAM = 1

    def __init__(self):
        Drone.__init__(self)
        self.flightController = FlightController(self)
        self.camera = P3N15.FRONTCAM
        self.outputImage = None
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
        print("                  Automous Drone Flight                  ")
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
        print Drone.getBattery(self)
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
        print "toggleCamera"
        drone.groundVideo(True)
        if self.camera == P3N15.FRONTCAM:
            print "Groundcam"
            self.camera = P3N15.GROUNDCAM
            self.groundCam()
        elif self.camera == P3N15.GROUNDCAM:
            print "frontcam"
            self.camera = P3N15.FRONTCAM
            self.frontCam()


    def startAutonomousFlight(self):
        '''Hands control to the flight controller'''
        if self.flightController == None:
            print("[Error] Cannot start autonomous flight. No flight controller specified!")
            return
        else:
            print "Start Thread"
            self.flightController = self.flightController.newInstance()
            self.flightController.start()

    def endAutonomousFlight(self):
        '''Ends the autonomous flight by terminating the flight controller'''
        if self.flightController != None:
            self.flightController.terminate()

# Mark: Controller Class
class FlightController(threading.Thread):
    ''' Base class to control the control the drone autonomously.

        Implement
            * newInstance(),
            * run()
            * terminate()
        in all subclasses.
    '''
    def __init__(self, drone):
        threading.Thread.__init__(self)
        self.drone = drone
        self.finished = False

    def newInstance(self):
        return FlightController(self.drone)

    def run(self):
        i = 0
        while not self.finished:
            i = i + 1
        else:
            print "Ended Autonomous Flight after %d cycles" %(i)


    def terminate(self):
        self.finished = True

def run(drone):
    aut = False

    pygame.init()
    pygame.display.set_caption("P3N15 Demo")
    screen = pygame.display.set_mode((640, 360))
    screen = pygame.display.get_surface()

    # config drone video settings
    drone.initCamera()

    cv2.namedWindow("w1");

    finished = False
    imcount = drone.VideoImageCount
    time_tmp = time.time()
    while not finished:
        # wait for next frame and calculate framerate
        while imcount == drone.VideoImageCount:
            time.sleep(0.01)
        imcount = drone.VideoImageCount

        t_now = time.time()
        t_diff = t_now - time_tmp
        time_tmp = t_now
        fps = (1 / t_diff) if t_diff > 0 else 0

        frame = drone.VideoImage;

        cv2.rectangle(frame,(180, 30), (0, 0), (0,0,0), -1)
        cv2.putText(frame,'{: <2} FPS'.format(fps),(10,20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(255,255,255),2)

        myframe = numpy.rot90(frame)
        myframe = numpy.flipud(myframe)
        surface = pygame.surfarray.make_surface(myframe)
        screen.blit(surface, (0,0))
        pygame.display.flip()

        # handle events
        for event in pygame.event.get():
            # handle quit
            if event.type == pygame.QUIT:
                finished = True

            if aut and (event.type == pygame.KEYUP or event.type == pygame.KEYDOWN):
                print "Not Aut"
                drone.endAutonomousFlight()
                aut = False

            if not aut:
                # handle keyup
                if event.type == pygame.KEYUP:
                    if event.key == pygame.K_ESCAPE:
                        drone.land()
                        finished = True
                    elif event.key == pygame.K_SPACE:
                        if drone.NavData["demo"][0][2] and not drone.NavData["demo"][0][3]:
                            drone.takeoff()
                        else:
                            drone.land()
                    elif event.key == pygame.K_p:
                        print "aut true"
                        aut = True
                        drone.startAutonomousFlight()
                    else:
                        drone.hover()
                # handle keydown
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_w:
                        drone.moveForward()
                    elif event.key == pygame.K_s:
                        drone.moveBackward()
                    elif event.key == pygame.K_a:
                        drone.moveLeft()
                    elif event.key == pygame.K_d:
                        drone.moveRight()
                    elif event.key == pygame.K_UP:
                        drone.moveUp()
                    elif event.key == pygame.K_DOWN:
                        drone.moveDown()
                    elif event.key == pygame.K_c:
                        drone.toggleCamera()

        if not aut:
             #handle continues key_input
            keys = pygame.key.get_pressed()
            if keys[pygame.K_q]:
                drone.turnLeft()
            elif keys[pygame.K_e]:
                drone.turnRight()


    drone.shutdown()
    time.sleep(1)
    pygame.quit()
    cv2.destroyAllWindows()

def capture(drone):
    # init pygame
    drone.initCamera(camera = P3N15.GROUNDCAM)

    pygame.init()
    pygame.display.set_caption("P3N15 Demo")
    screen = pygame.display.set_mode((640, 360))
    screen = pygame.display.get_surface()


    ##### And action !
    print "Use <space> to capture groundcamera frame, any other key to stop"
    IMC =    drone.VideoImageCount                               # Number of encoded videoframes
    stop =   False

    while True:
        if handleEvents(drone):
            break

        if drone.VideoImageCount != IMC:                        # no new frame available
            IMC = drone.VideoImageCount
            updateUI(drone, screen)


def updateUI(drone, screen):
    frame = prepareFrame(drone)
    screen.blit(frame, (0,0))
    pygame.display.flip()

def prepareFrame(drone):
    frame = numpy.rot90(drone.VideoImage)
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
        drone.moveUp()
    elif key == pygame.K_DOWN:
        drone.moveDown()
    elif key == pygame.K_c:
        drone.toggleCamera()
    elif key == pygame.K_m:
        filename = "img/" + str(drone.VideoDecodeTimeStamp) + '.png'
        cv2.imwrite(filename, drone.VideoImage)
        print "saved {}".format(filename)

def handleTurns(drone):
    keys = pygame.key.get_pressed()
    if keys[pygame.K_q]:
        drone.turnLeft()
    elif keys[pygame.K_e]:
        drone.turnRight()

if __name__ == '__main__':
    drone = P3N15()
    drone.startup()
    drone.useDemoMode(True)
    capture(drone)
    #run(drone)
