# Mandatory imports
import cv2
import numpy as np

# Plotting and visualization
import matplotlib.pyplot as plt

# path manipulations
import sys, os
from os.path import join as pjoin

class ImageSimilarity:
    def __init__(self, img1, img2):
        self.img1 = cv2.imread(img1, 0)
        self.img2 = cv2.imread(img2, 0)
        self.detector = cv2.xfeatures2d.SIFT_create()
        FLANN_INDEX_KDTREE = 1
        index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
        search_params = dict(checks = 50)
        self.matcher = cv2.FlannBasedMatcher(index_params, search_params)
    
    def extractKeypoints(self):
        key1, des1 = self.detector.detectAndCompute(self.img1, None)
        key2, des2 = self.detector.detectAndCompute(self.img2, None)
        return (key1, des1, key2, des2)
    
    def match(self, key1, des1, key2, des2):
        matches = self.matcher.knnMatch(des1, des2, k=2)
        return matches
    
    def consistency_check(self, matches):
        good = []
        for m, n in matches:
            if m.distance < 0.7*n.distance:
                good.append([m])
        return good
    
    def visualize(self, good, key1, key2):
        im = cv2.drawMatchesKnn(self.img1, key1, self.img2, key2, good, None,flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
        cv2.imwrite("image_matching.jpg", im)

def unit_test(img1, img2):
    im_sim = ImageSimilarity(img1, img2)
    key1, des1, key2, des2 = im_sim.extractKeypoints()
    matches = im_sim.match(key1, des1, key2, des2)
    good_matches = im_sim.consistency_check(matches)
    im_sim.visualize(good_matches, key1, key2)
    print("============Unit Test Passed=============")

if __name__=="__main__":
    img_2 = pjoin(os.getcwd(), "2013-09-cover.ngsversion.1505759607435.adapt.1900.1.jpg")
    img_1 = pjoin(os.getcwd(), "afsiuh.jpg")

    unit_test(img_1, img_2)