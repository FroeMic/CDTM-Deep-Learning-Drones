##! /usr/bin/env python
# -*- coding: utf-8 -*-

import time, sys
import cv2
from matplotlib import pyplot as plt

def main(argv):
    file = 'test.png'
    if len(argv) != 0:
        file = argv[0]

    orig = cv2.imread('original.img',0)
    img = cv2.imread(file)

    sift = cv2.SIFT()

    kp1, des1 = sift.detectAndCompute(orig, None)
    kp2, des2 = sift.detectAndCompute(img, None)

#    surf_detector = cv2.FeatureDetector_create("SURF")
#    surf_descriptor = cv2.DescriptorExtractor_create("SURF")
#    kp, des = surf_detector.detectAndCompute(img,None)
#    cv2.imshow('SURF', cv2.drawKeypoints(img, kp, None, (255,0,0), 4))

    FLANN_INDEX_KDTREE = 0
    index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
    search_params = dict(checks = 50)

    flann = cv2.FlannBasedMatcher(index_params, search_params)

    matches = flann.knnMatch(des1,des2,k=2)

    # store all the good matches as per Lowe's ratio test.
    good = []
    for m,n in matches:
        if m.distance < 0.7*n.distance:
            good.append(m)

    MIN_MATCH_COUNT = 10
    if len(good) > MIN_MATCH_COUNT:
        src_pts = np.float32([kp1[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
        dst_pts = np.float32([kp2[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)

        M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
        matchesMask = mask.ravel().tolist()

        h, w = img1.shape
        pts = np.float32([[0, 0], [0, h - 1], [w - 1, h - 1], [w - 1, 0]]).reshape(-1, 1, 2)
        dst = cv2.perspectiveTransform(pts, M)

        img2 = cv2.polylines(img2, [np.int32(dst)], True, 255, 3, cv2.LINE_AA)

    else:
        print "Not enough matches are found - %d/%d" % (len(good), MIN_MATCH_COUNT)
        matchesMask = None

    draw_params = dict(matchColor=(0, 255, 0),  # draw matches in green color
                       singlePointColor=None,
                       matchesMask=matchesMask,  # draw only inliers
                       flags=2)

    img3 = cv2.drawMatches(img1, kp1, img2, kp2, good, None, **draw_params)

    cv2.imshow('img', img3)
    #plt.imshow(img3, 'gray'), plt.show()

    while cv2.waitKey(1) & 0xFF != ord("q"):
        pass

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main(sys.argv[1:])
