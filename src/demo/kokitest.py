#!/usr/bin/env python
import pykoki
import cv, cv2
import sys


cap = cv2.VideoCapture(0)
koki = pykoki.PyKoki()

while cv2.waitKey(1) & 0xFF != ord('q'):
    img = cv.QueryFrame(cap)
    cv2.cvtColor(img, img, cv2.COLOR_BGR2GRAY)
    cv2.imshow('img',img)


    params = pykoki.CameraParams(Point2Df( img.width/2, img.height/2 ),
                          Point2Df(571, 571),
                          Point2Di( img.width, img.height ))

    print koki.find_markers( img, 0.1, params )
