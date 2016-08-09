import time
import sys
import thread
import cv2
import pygame
import numpy
from ps_drone import Drone

# =======================================
#     CDTM AUTONOMOUS DRONE ELECTIVE
#              August 2016
# =======================================

class P3N15(Drone):

    def __init__(self):
        Drone.__init__(self)
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
        Drone.startup(self)						 # Connects to drone and starts subprocesses
        Drone.reset(self)							         # Always good, at start

        while self.getBattery()[0] == -1:
            time.sleep(0.1)		                # Waits until the drone has done its reset
        time.sleep(0.5)							# Give it some time to fully awake


def run(drone):
    pygame.init()
    pygame.display.set_caption("P3N15 Demo")
    screen = pygame.display.set_mode((640, 360))
    screen = pygame.display.get_surface()

    # config drone video settings
    drone.sdVideo()
    drone.frontCam()
    CDC = drone.ConfigDataCount
    while CDC == drone.ConfigDataCount:
        time.sleep(0.0001)

    drone.videoFPS(10)
    drone.startVideo()

    cv2.namedWindow("w1");

    finished = False
    imcount = drone.VideoImageCount
    time_tmp = time.time()
    while not finished:
        # wait for next frame
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
        #cv2.imshow("w1", frame)
        #cv2.waitKey(1)

        myframe = numpy.rot90(frame)
        myframe = numpy.flipud(myframe)
        surface = pygame.surfarray.make_surface(myframe)
        screen.blit(surface, (0,0))
        pygame.display.flip()

        for event in pygame.event.get():
            print "got event"

    pygame.quit()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    drone = P3N15()
    drone.startup()
    run(drone)
