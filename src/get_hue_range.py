#! /usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import time

capture = cv2.VideoCapture(0)
capture.open(0)

cv2.namedWindow("Display")
cv2.namedWindow("Display, Hue")
cv2.namedWindow("Display, Hue, Thresholded")

h_target = 127
h_thresh = 100

def onChangeTarget(val):
    global h_target
    h_target = val
def onChangeThresh(val):
    global h_thresh
    h_thresh = val

cv2.createTrackbar("Target Hue", "Display, Hue, Thresholded", 127, 255, onChangeTarget)
cv2.createTrackbar("Target Range", "Display, Hue, Thresholded", 100, 255, onChangeThresh)

while cv2.waitKey(1) & 0xFF != 27:
    ret, img = capture.read()
    cv2.imshow("Display", img)

    color_low = np.array([h_target - h_thresh/2, 50, 50])
    color_high = np.array([h_target + h_thresh/2, 255, 255])

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    hue = hsv.copy()
    hue[:] = (255, 255, 255)
    hue[:,:,0] = hsv[:,:,0]
    cv2.imshow("Display, Hue", hue)

    mask = cv2.inRange(hsv, color_low, color_high)
    res = cv2.bitwise_and(img, img, mask=mask)

    cv2.imshow("Display, Hue, Thresholded", res)

    time.sleep(0.05)

cv2.destroyAllWindows()