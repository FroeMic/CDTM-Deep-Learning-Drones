import threading
import cv2
import numpy
import math
import time

# ------------------------
# -- MARK: Controller Classes
# ------------------------

# Base class
class FlightController(threading.Thread):
    ''' Base class to control the drone autonomously.

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

# Face controller class
class FaceController(FlightController):
    ''' Controller to follow a face autonomously.'''
    def __init__(self, drone):
        FlightController.__init__(self, drone)
        self.face_cascade = cv2.CascadeClassifier('lib/haarcascade_frontalface_default.xml')

    def newInstance(self):
        return FaceController(self.drone)

    def run(self):
        IMC = self.drone.VideoImageCount
        while not self.finished:
            if self.drone.VideoImageCount != IMC:
                IMC = self.drone.VideoImageCount
                t_start = time.time()
                self.drone.outputImage = self.analyzeImage()
                #print "Image Recognition Cycle: ",  time.time() - t_start
            time.sleep(0.01)
        else:
            print "Ended Autonomous Face Flight"

    def analyzeImage(self):
        img = self.drone.VideoImage

        faces, new_img = self.detectFaces(img)

        if len(faces) >  0:
            self.moveDrone(faces[0])

        return new_img

    def detectFaces(self, img):
        '''returns faces array (x,y,w,h) and altered image'''
        img = self.drone.VideoImage
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        faces = self.face_cascade.detectMultiScale(gray, 1.3, 5)
        red = True
        for face in faces:
            if red:
                self.drawRectangle(face, img)
            else:
                self.drawRectangle(face, img, (255,0,0))

        if len(faces) > 0:
            self.saveImage(img)

        return faces, img

    def drawRectangle(self, box, img, color=(0,0,255)):
        '''Draws a rectangle around the the specified box.'''
        (x,y,w,h) = box
        cv2.rectangle(img, (x,y), (x+w, y+h), color, 2)

    def saveImage(self, img, path="img/", title=None):
        if not title:
            title = str(self.drone.VideoDecodeTimeStamp) + '.png'

        filepath = path + "/" + title
        cv2.imwrite(filepath, img)


    def moveDrone(self, box):
        (x,y,w,h) = box
        print "BoxSize ", w*h, ", Position ", (x, y)


# Penis controller class
class PenisController(FlightController):
    ''' Controller to follow a Penis autonomously.'''

    def __init__(self, drone):
        FlightController.__init__(self, drone)
        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

    def newInstance(self):
        return PenisController(self.drone)

    def run(self):
        IMC = self.drone.VideoImageCount
        while not self.finished:
            if self.drone.VideoImageCount != IMC:
                IMC = self.drone.VideoImageCount
                t_start = time.time()
                try:
                    self.drone.outputImage = self.analyzeImage()
                except:
                    print "Error"
                print "Image Recognition Cycle: ", time.time() - t_start
            time.sleep(0.01)
        else:
            print "Ended Autonomous Face Flight"

    def analyzeImage(self):
        img = self.drone.VideoImage

        hsl = cv2.cvtColor(img, cv2.COLOR_RGB2HLS_FULL)[:,:,1]

        # enhance contrast
        hsl = self.clahe.apply(hsl)
        # binary image
        blur = cv2.GaussianBlur(hsl, (9, 9), 100)
        flag, thresh = cv2.threshold(blur, 200, 255, cv2.THRESH_BINARY)

        # get biggest contour
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contour = sorted(contours, key=cv2.contourArea, reverse=True)[0]

        # clip to rotated rectangle
        rect = cv2.minAreaRect(contour)
        box = cv2.cv.BoxPoints(rect)
        box = numpy.int0(box)
        mask = 0 * hsl
        cv2.drawContours(mask, [box], -1, (255), -1)
        hsl = cv2.bitwise_and(hsl, mask) + cv2.bitwise_not(0 * mask, mask)

        # binary image
        blur = cv2.GaussianBlur(hsl, (9, 9), 100)
        flag, thresh = cv2.threshold(blur, 200, 255, cv2.THRESH_BINARY)
        thresh = 255 - thresh
        thresh_annot = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)
        cv2.drawContours(thresh_annot, [box], -1, (0, 0, 255), 1)

        # Get biggest contour
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contour = sorted(contours, key=cv2.contourArea, reverse=True)[0]
        # cv2.drawContours(thresh_annot, contour, -1, (255,0,0), cv2.CV_AA)

        # Fit a line (to get angle)
        rows, cols = thresh.shape[:2]
        [vx, vy, x, y] = cv2.fitLine(contour, cv2.cv.CV_DIST_L2, 0, 0.01, 0.01)
        lefty = int((-x * vy / vx) + y)
        righty = int(((cols - x) * vy / vx) + y)
        cv2.line(thresh_annot, (cols - 1, righty), (0, lefty), (0, 255, 0), 2)
        angle = math.atan2(vy, vx)

        # compute the center of the contour
        M = cv2.moments(contour)
        X = int(M["m10"] / M["m00"])
        Y = int(M["m01"] / M["m00"])
        cv2.circle(thresh_annot, (X, Y), 8, (0, 0, 255), -1)

        print "angle: {}, position: {}, {}".format(angle, X, Y)

        return thresh_annot
