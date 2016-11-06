#!/usr/bin/python

from panda3d.core import Point3

EPSILON = 0.002

def getIntersectionBetweenPoints(pt0, pt1, pt2, pt3):

    denominator = (pt3.y - pt2.y)*(pt1.x - pt0.x) - (pt3.x - pt2.x)*(pt1.y - pt0.y)
    if denominator == 0.0:
        return None  # the two lines are parallel
    Ua = ((pt3.x - pt2.x)*(pt0.y - pt2.y) - (pt3.y - pt2.y)*(pt0.x - pt2.x)) / denominator
    return Point3(pt0.x + Ua*(pt1.x - pt0.x), pt0.y + Ua*(pt1.y - pt0.y), 0.0)

