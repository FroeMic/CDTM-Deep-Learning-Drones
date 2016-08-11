import threading
import cv2
import numpy
import math
import time
import sys
import matplotlib.pyplot as plt

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


    def moveDrone(self, box):
        (x,y,w,h) = box
        print "BoxSize ", w*h, ", Position ", (x, y)


# Penis controller class
class PenisController(FlightController):
    ''' Controller to follow a Penis autonomously.'''

    def __init__(self, drone):
        FlightController.__init__(self, drone)
        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

        cv2.namedWindow('config', cv2.WINDOW_NORMAL)
        cv2.createTrackbar("Yaw P", 'config', 500, 1000, self.onTrackbarYawPChange)
        cv2.createTrackbar("Yaw D", 'config', 500, 1000, self.onTrackbarYawDChange)
        cv2.createTrackbar("Dist P", 'config', 500, 1000, self.onTrackbarDistPChange)
        cv2.createTrackbar("Dist D", 'config', 500, 1000, self.onTrackbarDistDChange)

        self.hplot, = plt.plot([],[])

    def onTrackbarYawPChange(self, value):
        self.Kp_r = value / 100.0
    def onTrackbarYawDChange(self, value):
        self.Kd_r = value / 100.0
    def onTrackbarDistPChange(self, value):
        self.Kp_d = value / 100.0
    def onTrackbarDistDChange(self, value):
        self.Kd_d = value / 100.0

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

        self.Kp_r = 1
        self.Kd_r = 1e-2
        self.Kp_d = 1
        self.Kd_d = 1

        last_video_timestamp = self.drone.VideoDecodeTimeStamp

        while not self.finished:
            if self.drone.VideoImageCount != IMC:
                # new image arrived
                IMC = self.drone.VideoImageCount
                t_start = time.time()

                try:
                    t_diff = self.drone.VideoDecodeTimeStamp - last_video_timestamp
                    last_video_timestamp = self.drone.VideoDecodeTimeStamp

                    self.drone.outputImage, dyaw, dist = self.analyzeImage(self.drone.VideoImage)

                    if dyaw is not None:
                        #yaw_int += (yaw_ist - yaw_soll) * t_diff

                        dyaw -= math.pi/2
                        if dyaw > math.pi:
                            dyaw -= 2*math.pi
                        if dyaw < -math.pi:
                            dyaw += 2*math.pi

                        print "angle: {}, dist: {}".format(dyaw * 180.0 / math.pi, dist)

                        r_diff_to_last_time = dyaw - pos_r_ist
                        pos_r_vel = 0.99 * pos_r_vel + 0.01 * r_diff_to_last_time
                        pos_r_ist = dyaw

                        self.hplot.set_xdata(numpy.append(self.hplot.get_xdata(), last_video_timestamp))
                        self.hplot.set_ydata(numpy.append(self.hplot.get_ydata(), dyaw))


                    if dist is not None:
                        dist_diff_to_last_time = dist - pos_d_ist
                        pos_d_vel = 0.99 * pos_d_vel + 0.01 * dist_diff_to_last_time
                        pos_d_ist = dist

                    #print "Image Recognition Cycle: ", time.time() - t_start
                except:
                    print sys.exc_traceback

            if self.drone.NavDataCount != NAVC:
                # new nav data arrived
                NAVC = self.drone.NavDataCount


                print "RawMeasures [X,Y,Z]:          "+str(self.drone.NavData["raw_measures"][0])
                print "Gyros Offset [X,Y,Z]:         "+str(self.drone.NavData["gyros_offsets"])


                try:
                    #pos_r_vel = self.drone.NavData["raw_measures"][1][2] # gyro_z, filtered (LSBs)

                    #print pos_r_vel

                    pos_r_err = pos_r_soll - pos_r_ist
                    pos_d_err = pos_d_soll - pos_d_ist
                    pos_r_out = self.Kp_r * pos_r_err + self.Kd_r * pos_r_vel
                    pos_d_out = self.Kp_d * pos_d_err + self.Kd_d * pos_d_vel

                    pos_x_out = pos_d_out * math.cos( pos_r_err )
                    pos_y_out = pos_d_out * math.sin( pos_r_err )

                    print "D:", pos_d_out, "R: ", pos_r_out, "X: ", pos_x_out, "Y: ", pos_y_out

                    self.drone.move(pos_x_out, pos_y_out, 0, pos_r_out)

                except:
                    print sys.exc_info()

            plt.draw()
            cv2.waitKey(1)

        else:
            print "Ended Autonomous Face Flight"

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

        # get biggest contou
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) == 0:
            print "could not find contour of paper"
            cv2.imshow('info', thresh)
            return thresh, None, None
        contour = sorted(contours, key=cv2.contourArea, reverse=True)[0]

        # TODO: iterate over contours

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
            print "ratio_deviation is above 15% ({}%)".format(100 * ratio_deviation)
            # print a, b
            cv2.imshow('info', annot)
            return annot, None, None

        # Get biggest contour
        tmp = thresh.copy()
        contours, hierarchy = cv2.findContours(tmp, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contour = sorted(contours, key=cv2.contourArea, reverse=True)[0]
        if len(contours) == 0:
            print "could not find contour of male sign"
            cv2.imshow('info', annot)
            return annot, None, None
        # cv2.drawContours(annot, contour, -1, (255, 0, 0), 3)

        # get center of the full shape
        rect = cv2.minAreaRect(contour)
        boundingCenter = rect[0]

        # Fit a line (to get (ambiguous) angle)
        rows, cols = thresh.shape[:2]
        indicatorLine = cv2.fitLine(contour, cv2.cv.CV_DIST_L2, 0, 0.01, 0.01)
        [vx, vy, x, y] = indicatorLine
        angle = math.atan2(vy, vx)

        # todo: check on which side of the line our arrow is

        # mask inner contour
        mask = numpy.zeros(thresh.shape, numpy.uint8)
        cv2.drawContours(mask, [contour], -1, (255), -1)
        thresh = cv2.bitwise_and(255 - thresh, mask)
        # cv2.imshow('thresh', thresh)

        # get inner circle shape
        tmp = thresh.copy()
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) == 0:
            print "could not find contour of inner circle"
            cv2.imshow('info', annot)
            return annot, None, None
        contour = sorted(contours, key=cv2.contourArea, reverse=True)[0]
        cv2.drawContours(annot, contour, -1, (255, 0, 0), 3)

        # get circle circularity
        circumference = cv2.arcLength(contour, True)
        area = cv2.contourArea(contour)
        circularity = circumference ** 2 / (4 * math.pi * area)

        if circularity > 1.2:
            print "circularity of inner circle too low ({} < 1.2)!".format(circularity)

        # compute the center of the circle contour
        M = cv2.moments(contour)
        x = int(M["m10"] / M["m00"])
        y = int(M["m01"] / M["m00"])
        circleCenter = (x, y)
        cv2.circle(annot, circleCenter, 10, 0, -1)
        cv2.circle(annot, circleCenter, 8, (0, 0, 255), -1)

        # get direction of line and draw it
        direction = self.calcVectorDirection(circleCenter, boundingCenter, indicatorLine)
        arrowHead = (x + 200 * direction * vx, y + 200 * direction * vy)
        cv2.line(annot, circleCenter, arrowHead, cv2.cv.CV_RGB(0, 0, 0), 5)
        cv2.line(annot, circleCenter, arrowHead, cv2.cv.CV_RGB(0, 255, 0), 3)

        # get distance of circle center from image center
        imgH, imgW, _ = img.shape
        dist = math.hypot(x - imgW / 2.0, y - imgH / 2.0)

        # print dist

        cv2.imshow('info', annot)

        angle += direction * math.pi

        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi

        print "angle: {}, position: {}, {}".format(angle * 180.0 / math.pi, x, y)

        return annot, angle, dist

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
