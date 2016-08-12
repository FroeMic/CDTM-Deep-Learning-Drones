import threading
import cv2
import numpy
import math
import time
import sys
from pid import PID

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

    def saveImage(self, img, path="img/", title=None):
        if not title:
            title = str(self.drone.VideoDecodeTimeStamp) + '.png'

        filepath = path + "/" + title
        cv2.imwrite(filepath, img)


# Face controller class
class FaceController(FlightController):
    ''' Controller to follow a face autonomously.'''
    def __init__(self, drone):
        FlightController.__init__(self, drone)
        self.face_cascade = cv2.CascadeClassifier('lib/haarcascade_frontalface_default.xml')
        # init pids
        self.rotation_pid = PID(0.6,0.6,0.05)
        self.height_pid = PID(0.1,0.1,0.01)
        self.dist_pid = PID(0.2,0.25,0.1)

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

        if len(faces) > 0:
            self.moveDrone(faces[0], new_img) #only follow first face
        else:
            self.drone.hover()

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


    def moveDrone(self, box, img):
        (x,y,w,h) = box
        f_h, f_w = img.shape[:2]                # image height and width

        leftright_step = 0.0
        backforward_step = dist_pid.step(self.distanceError(h))
        updown_step = height_pid.step(self.verticalError(f_h, h))
        turnleftright_step = rotation_pid.step(self.horizontalError(f_w, w))

        drone.move(leftright_step,-float(backforward_step),-float(updown_step),float(turnleftright_step))

    def verticalError(self, imageHeight, boxHeight):
        return (imageHeight - boxHeight / 2.0) / (boxHeight / 2.0)

    def horizontalError(self, imageWidth, boxWidth):
        return (imageWidth - boxWidth / 2.0) / (boxWidth / 2.0)

    def distanceError(self, h):
        desiredBoxHeight = 75
        return (h - desiredBoxHeight) / desiredBoxHeight

# Penis controller class
class PenisController(FlightController):
    ''' Controller to follow a Penis autonomously.'''

    def __init__(self, drone):
        FlightController.__init__(self, drone)
        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

        #cv2.namedWindow('config', cv2.WINDOW_NORMAL)
        #cv2.createTrackbar("Yaw P", 'config', 50, 100, self.onTrackbarYawPChange)
        #cv2.createTrackbar("Yaw D", 'config', 0, 100, self.onTrackbarYawDChange)
        #cv2.createTrackbar("Dist P", 'config', 20, 100, self.onTrackbarDistPChange)
        #cv2.createTrackbar("Dist D", 'config', 5, 100, self.onTrackbarDistDChange)

    def onTrackbarYawPChange(self, value):
        self.Kp_r = value / 10000.0
    def onTrackbarYawDChange(self, value):
        self.Kd_r = value / 10000.0
    def onTrackbarDistPChange(self, value):
        self.Kp_d = value / 1000.0
    def onTrackbarDistDChange(self, value):
        self.Kd_d = value / 10000.0

    def newInstance(self):
        return PenisController(self.drone)

    def run(self):
        IMC = self.drone.VideoImageCount

        NAVC = self.drone.NavDataCount

        pos_r_soll = 0
        pos_r_ist = 0
        pos_r_int = 0
        pos_r_vel = 0
        pos_d_soll = 0
        pos_d_ist = 0
        pos_d_int = 0
        pos_d_vel = 0
        pos_d_phi = 0

        self.Kp_r = 100/10000.0
        self.Kd_r = -10/10000.0
        self.Kp_d = 5/10000.0
        self.Kd_d = -50/10000.0

        last_video_timestamp = self.drone.VideoDecodeTimeStamp

        while not self.finished:
            if self.drone.VideoImageCount != IMC:
                # new image arrived
                IMC = self.drone.VideoImageCount
                t_start = time.time()

                try:
                    t_diff = self.drone.VideoDecodeTimeStamp - last_video_timestamp
                    last_video_timestamp = self.drone.VideoDecodeTimeStamp

                    self.drone.outputImage, x, y, w = self.analyzeImage(self.drone.VideoImage)

                    if not (x is None or y is None or w is None):

                        print "x, y, w: ", x, y, w

                        turn = 0
                        if x < -100:
                            turn = -0.1
                        elif x > 100:
                            turn = 0.1

                        up = 0
                        if y > 100:
                            up = 0.1
                        elif y < -100:
                            up = -0.1

                        forward = 0
                        if w > 200:
                            forward = -0.1
                        elif w < 100:
                            forward = 0.1

                        right = 0

                        print "right, forward, up, turn:", right, forward, up, turn

                        self.drone.move(right, forward, up, turn)
                    else:
                        self.drone.stop()

                    #print "Image Recognition Cycle: ", time.time() - t_start
                except:
                    pass
                    #print str(sys.exc_traceback)

            if self.drone.NavDataCount != NAVC:
                # new nav data arrived
                NAVC = self.drone.NavDataCount

        else:
            print "Ended Autonomous Flight"

    def analyzeImage(self, img):
        hsl = cv2.cvtColor(img, cv2.COLOR_RGB2HLS_FULL)[:, :, 1]
        # cv2.imshow('hls', hsl)

        # enhance contrast
        hsl = self.clahe.apply(hsl)
        # cv2.imshow('clahe', hsl)

        # binary image
        blur = cv2.GaussianBlur(hsl, (9, 9), 100)
        # cv2.imshow('blur', blur)
        flag, thresh = cv2.threshold(blur, 200, 255, cv2.THRESH_BINARY)

        # get biggest contour
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) == 0:
            #print "could not find contour of paper"
            #cv2.imshow('info', thresh)
            return thresh, None, None

        # iterate over contours
        contours_temp = contours
        for contour0 in sorted(contours_temp, key=cv2.contourArea, reverse=True):

            # clip to rotated rectangle
            rect = cv2.minAreaRect(contour0)
            box = cv2.cv.BoxPoints(rect)
            box = numpy.int0(box)
            mask = 0 * hsl
            cv2.drawContours(mask, [box], -1, (255), -1)
            hsl_masked = cv2.bitwise_and(hsl, mask) + cv2.bitwise_not(0 * mask, mask)

            # binary image
            blur = cv2.GaussianBlur(hsl_masked, (9, 9), 100)
            flag, thresh = cv2.threshold(blur, 200, 255, cv2.THRESH_BINARY)
            thresh = 255 - thresh
            # thresh_annot = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)
            annot = img
            cv2.drawContours(annot, [box], -1, (0, 0, 255), 4)

            # check ratio of rectangle, it must be roughly A4
            a = math.hypot(box[0][0] - box[1][0], box[0][1] - box[1][1])
            b = math.hypot(box[1][0] - box[2][0], box[1][1] - box[2][1])

            ratio = a / b if a < b else b / a
            correct_ratio = 210.0 / 297.0
            ratio_deviation = abs((ratio - correct_ratio) / correct_ratio)

            if ratio_deviation > 0.15:
                #print "ratio_deviation is above 15% ({}%)".format(100 * ratio_deviation)
                # print a, b
                #cv2.imshow('info', annot)
                #return annot, None, None
                continue

            # Get biggest contour
            tmp = thresh.copy()
            contours, hierarchy = cv2.findContours(tmp, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            if len(contours) == 0:
                #print "could not find contour of male sign"
                #cv2.imshow('info', annot)
                #return annot, None, None
                continue
            contour = sorted(contours, key=cv2.contourArea, reverse=True)[0]
            # cv2.drawContours(annot, contour, -1, (255, 0, 0), 3)

            # get center of the full shape
            rect = cv2.minAreaRect(contour)
            boundingCenter = rect[0]

            # Fit a line (to get (ambiguous) angle)
            rows, cols = thresh.shape[:2]
            indicatorLine = cv2.fitLine(contour, cv2.cv.CV_DIST_L2, 0, 0.01, 0.01)
            [vx, vy, x, y] = indicatorLine
            angle = math.atan2(vy, vx) + math.pi

            # mask inner contour
            mask = numpy.zeros(thresh.shape, numpy.uint8)
            cv2.drawContours(mask, [contour], -1, (255), -1)
            thresh = cv2.bitwise_and(255 - thresh, mask)
            # cv2.imshow('thresh', thresh)

            # get inner circle shape
            tmp = thresh.copy()
            contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            if len(contours) == 0:
                #print "could not find contour of inner circle"
                #cv2.imshow('info', annot)
                #return annot, None, None
                continue
            contour = sorted(contours, key=cv2.contourArea, reverse=True)[0]
            cv2.drawContours(annot, contour, -1, (255, 0, 0), 3)

            # get circle circularity
            circumference = cv2.arcLength(contour, True)
            area = cv2.contourArea(contour)
            circularity = circumference ** 2 / (4 * math.pi * area)

            if circularity > 1.2:
                #print "circularity of inner circle too low ({} < 1.2)!".format(circularity)
                #return annot, None, None
                continue

            # compute the center of the circle contour
            M = cv2.moments(contour)
            x = int(M["m10"] / M["m00"])
            y = int(M["m01"] / M["m00"])
            circleCenter = (x, y)
            cv2.circle(annot, circleCenter, 10, 0, -1)
            cv2.circle(annot, circleCenter, 8, (0, 0, 255), -1)

            # get direction of line and draw it
            direction = self.calcVectorDirection(circleCenter, boundingCenter, indicatorLine)
            arrowHead = (x + 100 * direction * vx, y + 100 * direction * vy)
            cv2.line(annot, circleCenter, arrowHead, cv2.cv.CV_RGB(0, 0, 0), 5)
            cv2.line(annot, circleCenter, arrowHead, cv2.cv.CV_RGB(0, 255, 0), 3)

            # get distance of circle center from image center
            imgH, imgW, _ = img.shape
            x = x - imgW / 2.0
            y = y - imgH / 2.0
            dist = math.hypot(x, y)

            # print dist

            #cv2.imshow('info', annot)

            if direction < 0:
                angle -= math.pi

            while angle > math.pi:
                angle -= 2 * math.pi
            while angle < -math.pi:
                angle += 2 * math.pi

            #print "angle: {}, position: {}, {}".format(angle * 180.0 / math.pi, x, y)

            phi = math.atan2(y,x)

            w = a if a > b else b

            return annot, x, y, w

    def calcVectorDirection(self, circleCenter, boundingCenter, indicatorLine):
        (x_c, y_c) = circleCenter
        (x_b, y_b) = boundingCenter
        (vx_l, vy_l, x_l, y_l) = indicatorLine

        x = x_b - x_c
        y = y_b - y_c

        skal = x * vx_l + y * vy_l
        if skal < 0:
            return -1
        else:
            return 1


    def turnDrone(self,angle):
        deg = math.degree(angle)
        self.turnAngle(deg, 1)

if __name__ == "__main__":
    penis = PenisController(None)
    img = cv2.imread("img/up.png")
    img, x, y = penis.analyzeImage(img)
    print img
    if img is not None:
        cv2.imshow('info', img)
    cv2.waitKey(0)
    exit(0)
