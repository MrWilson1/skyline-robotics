from __future__ import division

import math
import random
import decimal

import find_targets

def original(actual_height, image_height, pixel_height, vertical_fov):
    return (((actual_height * image_height)/pixel_height)/2.0) / math.tan(((vertical_fov*math.pi)/180.0)/2.0)
    
def concise(actual_height, image_height, pixel_height, vertical_fov):
    
    return actual_height * image_height / (pixel_height * 2.0) * 1 / math.tan(vertical_fov * math.pi / 360.0)
    
def test_centerpoint():
    C = find_targets.Coordinate
    X = find_targets.find_center_point
    print X(C(0,4), C(4,4), C(4,0), C(0,0))
    print X(C(0,4), C(6,4), C(6,2), C(0,0))
    print X(C(0,4), C(8,8), C(8,4), C(0,0))
    
def test():
    image_height = 240
    actual_height = 10 # in inches
    vertical_fov = 50
    for i in xrange(10):
        pixel_height = random.randrange(10, 200)
        o = original(actual_height, image_height, pixel_height, vertical_fov)
        n = concise(actual_height, image_height, pixel_height, vertical_fov)
        if o != n:
            print pixel_height, o, n
    

    
    
if __name__ == '__main__':
    test_centerpoint()
            