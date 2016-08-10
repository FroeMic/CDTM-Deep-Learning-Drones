#! /usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import time
import sys, getopt


def main(argv):
    inputfile = 'video.mp4'
    if len(argv) > 0:
        inputfile = argv[0]

    cap = cv2.VideoCapture(inputfile)

    while(True):
        # Capture frame-by-frame
        ret, frame = cap.read()

        if not ret:
            print "No frame found"
            exit(0)

        # Our operations on the frame come here
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Display the resulting frame
        cv2.imshow('frame',gray)
        if cv2.waitKey(0) & 0xFF == ord('q'):
            break

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main(sys.argv[1:])
