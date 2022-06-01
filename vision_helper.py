#!/usr/bin/env python3

import numpy as np
import math

def euclidean_distance(pt1, pt2):
    return math.sqrt((pt2[0] - pt1[0])**2 + (pt2[1] - pt1[1])**2)
    
def distance_from_camera(focal_length, real_width, observed_width):
    # compute and return the distance from the maker to the camera
    if observed_width == 0:
        return 0
    return (real_width * focal_length) / observed_width
    
def brighten_grayscale(image, brighten):
    mask = (255 - image) < brighten
    return np.where((255 - image) < brighten, 255, image + brighten)
