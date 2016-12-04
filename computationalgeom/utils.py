#!/usr/bin/python

from panda3d.core import Point3

EPSILON = 0.002


def getCenterOfPoints3D(points):
    n = len(points)
    x = 0
    y = 0
    z = 0
    for i in points:
        x = x + i.x
        y = y + i.y
        z = z + i.z
    return Point3(x/n, y/n, z/n)


def getIntersectionBetweenPoints(pt0, pt1, pt2, pt3):

    denominator = (pt3.y - pt2.y)*(pt1.x - pt0.x) - (pt3.x - pt2.x)*(pt1.y - pt0.y)
    if denominator == 0.0:
        return None  # the two lines are parallel
    Ua = ((pt3.x - pt2.x)*(pt0.y - pt2.y) - (pt3.y - pt2.y)*(pt0.x - pt2.x)) / denominator
    return Point3(pt0.x + Ua*(pt1.x - pt0.x), pt0.y + Ua*(pt1.y - pt0.y), 0.0)


def getLeftPt(pt, ptPair):
    """Takes the center of two points then returns the left point as viewed from a third point (1st parameter)."""
    midPt = getCenterOfPoints3D(ptPair)
    vecToMid = midPt - pt
    vecToPt1 = ptPair[0] - pt
    # the point on the leftVec has a negative z in its cross product with the middle point
    if vecToPt1.cross(vecToMid).z < 0:
        return ptPair[0]
    else:
        return ptPair[1]


def isPointInWedge(pt, line1, line2, inclusive=True):
    """Returns True, if the given point is inside the infinite wedge formed
    by the two lines (inclusive = True considers points on the lines to be in the wedge.)"""
    lftVec, rtVec = makeWedge(line1, line2)
    for i in range(0, 2):
        if line1[i] in line2:
            shared1 = i
    ptVec = pt - line1[shared1]
    # right cross pt should be up. Left cross pt should be down, if the point is inside the infinite wedge.
    if inclusive:  # points on the edge of the wedge count as in the wedge
        return rtVec.cross(ptVec).z >= 0 >= lftVec.cross(ptVec).z
    else:
        return rtVec.cross(ptVec).z > 0 > lftVec.cross(ptVec).z


def makeWedge(line1, line2):
    """Makes a wedge formed by two supplied edges, and returns that wedge in the form [leftVector, rightVector]"""
    # this expects a point and two lines.
    # The lines are presented as two points each, and one of those must be in both lines.
    shared1 = shared2 = notShared1 = notShared2 = -1
    for i in range(0, 2):
        if line1[i] in line2:
            shared1 = i
        else:
            notShared1 = i

        if line2[i] in line1:
            shared2 = i
        else:
            notShared2 = i
    # print "makeWedge", line1, line2

    if shared1 == -1 or shared2 == -1:  # redundant but Oh well.
        sr = "makeWedge(): The two lines must share a point. Given points:\n" + str(line1) + "\n" + str(line2)
        raise StandardError(sr)

    # find which edge-end is on the left and which is on the right
    lftPt = getLeftPt(line1[shared1], [line1[notShared1], line2[notShared2]])
    if lftPt == line1[notShared1]:
        rtPt = line2[notShared2]
    else:
        rtPt = line1[notShared1]

    rtVec = rtPt - line1[shared1]
    lftVec = lftPt - line1[shared1]
    return [lftVec, rtVec]
