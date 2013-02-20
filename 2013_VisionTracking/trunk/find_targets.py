from __future__ import division

import collections
import math
import os

import rr

## Configuration variables ##

VERTICAL_FOV = 39.75
HORIZONTAL_FOV = 52

IMAGE_PIXEL_HEIGHT = rr.GetVariable("IMAGE_HEIGHT")
IMAGE_PIXEL_LENGTH = rr.GetVariable("IMAGE_LENGTH")

KNOWN_RATIOS = {
    'high': (62, 20),
    'medium': (62, 29),
    'low': (37, 32),
    'test': (12, 6),
}


class Coordinate(object):
    def __init__(self, x, y):
        self.x = x
        self.y = y
        
    def __add__(self, other):
        return Coordinate(self.x + other.x, self.y + other.y)
        
    def __sub__(self, other):
        return Coordinate(self.x - other.x, self.y - other.y)
        
    def __rmul__(a, b):
        '''Communitive multiplication'''
        if isinstance(a, Coordinate):
            self = a
            other = b
        else:
            self = b
            other = a
        return Coordinate(self.x * other, self.y * other)
        
        
    def __repr__(self):
        return 'Coordinate({0}, {1})'.format(self.x, self.y)
        
    def __str__(self):
        return '({0}, {1})'.format(self.x, self.y)
        
def crossproduct(a, b):
    return a.x * b.y - a.y * b.x
        

class Target(object):
    def __init__(self, p1=None, p2=None, p3=None, p4=None, ratio=None, center=None,
            offset=None, rect_type=None, distance=None):
        self.p1 = p1
        self.p2 = p2
        self.p3 = p3
        self.p4 = p4
        self.ratio = ratio
        self.center = center
        self.offset = offset # in degrees, not pixels
        self.rect_type = rect_type
        self.distance = distance
        self.exists = False


def get_raw_corner_coordinates(variable='BFR_COORDINATES'):
    corners = rr.GetArrayVariable(variable)
    for i in range(int(len(corners) / 8)):
        p1 = Coordinate(corners[i * 8 + 0], corners[i * 8 + 1])
        p2 = Coordinate(corners[i * 8 + 2], corners[i * 8 + 3])
        p3 = Coordinate(corners[i * 8 + 4], corners[i * 8 + 5])
        p4 = Coordinate(corners[i * 8 + 6], corners[i * 8 + 7])
        
        yield (p1, p2, p3, p4)
      
      
def find_ratio(p1, p2, p3, p4):
    pixel_height = ((p1.y - p4.y) + (p2.y - p3.y)) / 2
    pixel_length = ((p1.x - p2.x) + (p4.x - p3.x)) / 2
    
    return pixel_length/pixel_height

"""    
def find_ratio_2(p1, p2, p3, p4):
    '''
    Courtesy of http://stackoverflow.com/a/1222855/646543
    '''
    def sqr(val):
        return val*val
    
    # find the center (principal) point
    v = Coordinate(IMAGE_PIXEL_LENGTH/2, IMAGE_PIXEL_HEIGHT/2)

    # transform the image so the principal point is at (0,0)
    p1.x = p4.x - v.x;
    p1.y = p4.y - v.y;
    p2.x = p3.x - v.x;
    p2.y = p3.y - v.y;
    p3.x = p1.x - v.x;
    p3.y = p1.y - v.y;
    p4.x = p2.x - v.x;
    p4.y = p2.y - v.y;

    # temporary variables k2, k3
    k2 = (((p1.y - p4.y) * p3.x - (p1.x - p4.x) * p3.y + (p1.x * p4.y) - (p1.y * p4.x)) /
          ((p2.y - p4.y) * p3.x - (p2.x - p4.x) * p3.y + (p2.x * p4.y) - (p2.y * p4.x)))

    k3 = (((p1.y - p4.y) * p2.x - (p1.x - p4.x) * p2.y + (p1.x * p4.y) - (p1.y * p4.x)) / 
          ((p3.y - p4.y) * p2.x - (p3.x - p4.x) * p2.y + (p3.x * p4.y) - (p3.y * p4.x)))

    # if k2==1 AND k3==1, then the focal length equation is not solvable 
    # but the focal length is not needed to calculate the ratio.
    # k2 and k3 seems to become 1 when the rectangle is not distorted by 
    # perspective (viewed straight on).
    if (k2 == 1 and k3 == 1): 
        hwRatio = math.sqrt( 
            (sqr(p2.y - p1.y) + sqr(p2.x - p1.x)) / 
            (sqr(p3.y - p1.y) + sqr(p3.x - p1.x))
        )
    else:
        # f_squared is the focal length of the camera, squared
        # if k2==1 OR k3==1 then this equation is not solvable
        # if the focal length is known, then this equation is not needed
        # in that case assign f_squared= sqr(focal_length)    
        f_squared = -(((k3 * p3.y - p1.y) * (k2 * p2.y - p1.y) + 
                       (k3 * p3.x - p1.x) * (k2 * p2.x - p1.x)) / 
                      ((k3 - 1) * (k2 - 1)))

        # The height/width ratio of the original rectangle
        a = (sqr(k2 - 1) + sqr(k2 * p2.y - p1.y) / f_squared + sqr(k2 * p2.x - p1.x) / f_squared)
        b = (sqr(k3 - 1) + sqr(k3 * p3.y - p1.y) / f_squared + sqr(k3 * p3.x - p1.x) / f_squared) 
        hwRatio = math.sqrt(a / b)

    # Get the width/height ratio
    whRatio = 1 / hwRatio
    return whRatio
"""

class InvalidRectangle(Exception): pass
    
def find_center_point(p1, p2, p3, p4):
    '''
    Courtesy of <http://stackoverflow.com/a/565282/646543>
    '''
    coord1 = p1
    coord2 = p4
    vector1 = p3 - p1
    vector2 = p4 - p2
    numerator = crossproduct(coord2 - coord1, vector2)
    denominator = crossproduct(vector1, vector2)
    if numerator == 0:
        raise InvalidRectangle('The diagonals are colinear')
    if denominator == 0:
        raise InvalidRectangle('The diagonals are parallel')
    scale = numerator / denominator
    return coord1 + scale * vector1
    
def find_offset(center):
    v = Coordinate(IMAGE_PIXEL_LENGTH / 2, IMAGE_PIXEL_HEIGHT / 2)
    x_offset = (center.x - v.x) / v.x * (HORIZONTAL_FOV / 2)
    y_offset = (center.y - v.y) / v.y * (VERTICAL_FOV / 2)
    return Coordinate(x_offset, y_offset)
    
    
def find_rect_type(ratio):
    closest = min((abs(ratio - a / b), key) for key, (a, b) in KNOWN_RATIOS.items())[1]
    return closest
    

def find_distance(rect_type, p1, p2, p3, p4):
    actual_length, actual_height = KNOWN_RATIOS[rect_type]
    pixel_height = ((p1.y - p4.y) + (p2.y - p3.y)) / 2
    return actual_height * IMAGE_PIXEL_HEIGHT / (pixel_height * 2.0) * 1 / math.tan(VERTICAL_FOV * math.pi / 360.0)
    
def get_targets():
    targets = []
    for (p1, p2, p3, p4) in get_raw_corner_coordinates():
        target = Target()
        target.p1 = p1
        target.p2 = p2
        target.p3 = p3
        target.p4 = p4
        target.ratio = find_ratio(p1, p2, p3, p4)
        target.center = find_center_point(p1, p2, p3, p4)
        target.rect_type = find_rect_type(target.ratio)
        target.distance = find_distance(target.rect_type, p1, p2, p3, p4)
        target.exists = True
        targets.append(target)
    return targets
    

def return_best_distance_and_best_offset(targets):
    best_distance = float('infinity')
    best_distance_target = Target()
    best_offset = float('infinity')
    best_offset_target = Target()
    image_center = Coordinate(IMAGE_PIXEL_LENGTH / 2, IMAGE_PIXEL_HEIGHT / 2)
    for target in targets:
        if not target.exists:
            continue
        if target.distance < best_distance:
            best_distance = target.distance
            best_distance_target = target
        offset = image_center - target.center
        if offset.x**2 + offset.y**2:
            best_offset = offset.x**2 + offset.y**2
            best_offset_target = target
    return best_distance_target, best_offset_target
    
    
    
def find_overlap_start(master, test):
    master = [m.rect_type for m in master]
    test = [t.rect_type for t in test]
    test_length = len(test)
    
    for i in range(len(master) - test_length + 1):
        if master[i:i + test_length] == test:
            return i
    return None
        
def prepare_targets_for_networktables(targets):
    goal_targets = ['high', 'medium', 'low']
    goals = [t for t in targets if t.rect_type in goal_targets]
    extras = [t for t in targets if t.rect_type not in goal_targets]
    
    goals = sorted(goals, key=lambda t: t.center.x)
    actual = [Target(rect_type=x) for x in ['middle', 'high', 'middle', 'low']]
    index = find_overlap_start(actual, goals)
    if index is not None:
        actual[index: index + len(goals)] = goals
    
    counter = 0
    for t in actual:
        if t.rect_type == 'middle':
            if counter == 0:
                t.rect_type = 'middle_left'
                counter += 1
            elif counter == 1:
                t.rect_type = 'middle_right'
    actual.extend(extras)
    return actual
    
def set_single_nonexistant_target_to_default_values(target):
    if not target.exists:
        target.p1 = Coordinate(0, 0)
        target.p2 = Coordinate(0, 0)
        target.p3 = Coordinate(0, 0)
        target.p4 = Coordinate(0, 0)
        target.ratio = 1
        target.center = Coordinate(0, 0)
        target.offset = 0
        target.distance = 0
    return target
    
def set_nonexistant_targets_to_default_values(targets):
    output = []
    for target in targets:
        target = set_single_nonexistant_target_to_default_values(target)
        output.append(target)
    return output
    
    
def make_name(*args):
    return '_'.join(args)
    
def create_single_variable_for_networktables(target, custom_name=None):
    if custom_name is None:
        name = target.rect_type
    else:
        name = custom_name
    rr.SetVariable(r"/VisionTable/" + make_name(name, 'center', 'x'), target.center.x)
    rr.SetVariable(r"/VisionTable/" + make_name(name, 'center', 'y'), target.center.y)
    rr.SetVariable(r"/VisionTable/" + make_name(name, 'distance'), target.distance)
    rr.SetVariable(r"/VisionTable/" + make_name(name, 'distance_feet'), target.distance/12)
    
def create_variables_for_networktables(targets):
    for target in targets:
        create_single_variable_for_networktables(target)
    
    
def main():
    targets = get_targets()
    best_distance, best_offset = return_best_distance_and_best_offset(targets)
    targets = prepare_targets_for_networktables(targets)
    targets = set_nonexistant_targets_to_default_values(targets)
    create_variables_for_networktables(targets)
    
    best_distance = set_single_nonexistant_target_to_default_values(best_distance)
    best_offset = set_single_nonexistant_target_to_default_values(best_offset)
    
    create_single_variable_for_networktables(best_distance, 'best_distance')
    create_single_variable_for_networktables(best_offset, 'best_offset')
    
main()
        
    
        