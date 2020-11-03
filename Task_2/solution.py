# Mandatory imports
import cv2
import numpy as np

# plotting libs
import matplotlib.pyplot as plt

# File handling imports
from os.path import join as pjoin
import sys, os

class Segment:
    def __init__(self, image):
        """
        Class segment default constructor.
        Args:
            image: a .jpg, .jpeg or .png file
        """
        self.image = cv2.imread(image, 0)
    
    def basic_segment(self):
        ret, thresh = cv2.threshold(self.image, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        return thresh

    def additional_features(self):
        edge_map = cv2.Canny(self.image, 100, 200)
        return edge_map

    def visualize(self, image):
        save_path = pjoin(os.getcwd(), "result.jpg")
        cv2.imwrite(save_path, image)
    
def unit_test(image):
    segment = Segment(image)
    thresh_image = segment.basic_segment()
    edge_map = segment.additional_features()
    segment.visualize(edge_map)
    print("================Unit Test Passed=============")

if __name__=="__main__":
    image_path = pjoin(os.getcwd(), "125356504.jpg")
    unit_test(image_path)