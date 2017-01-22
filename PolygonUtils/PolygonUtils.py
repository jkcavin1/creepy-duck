__author__ = 'Lab Hatter'


from panda3d.core import Vec3, Vec4, Point3
from math import sqrt, pow


def getDistance(pt1, pt2):
    return sqrt(pow(pt1.x - pt2.x, 2) + pow(pt1.y - pt2.y, 2) + pow(pt1.z - pt2.z, 2))


def getDistance2d(pt1, pt2):
    return sqrt(pow(pt1.x - pt2.x, 2) + pow(pt1.y - pt2.y, 2))


def makeTriangleCcw(tri):
    rightVec = tri[1] - tri[0]
    leftVec = tri[2] - tri[0]
    if rightVec.cross(leftVec).z < 0:
        tmp = tri[1]
        tri[1] = tri[2]
        tri[2] = tmp
    return tri
    # the following was an attempt to make a polygon ccw

    # cntr = getCenterOfPoints3D(verts)
    # if cntr == Vec3(0, 0, 0):
    #     cntr += .01
    # q = PriorityQueue()
    # for i in verts:
    #     ang = getTrueAngleXYBetweenPoints(cntr, i)
    #     q.put((ang, i))
    #
    # # print cntr, "makeTriangleCcw" ###############  PRINT
    # verts = []
    # while q.qsize() != 0:
    #     n = q.get_nowait()
    #     # print n, getTrueAngleXYBetweenPoints(cntr, n[1]), "makeTriangleCcw"
    #     verts.append(n[1])
    # raise NotImplementedError("makeTriangleCcw is not implemented")
    # return verts


def triangleContainsPoint(pt, tri):
    """Takes a convex polygon and returns true, if the given point is inside the polygon"""
    triangle = makeTriangleCcw([tri[0], tri[1], tri[2]])
    # it needs to take the cross product of every point in the polygon and the next point
    # the final point needs to take the cross product of the beginning point
    triangle.append(triangle[0])
    foundOutside = False
    for p in range(0, len(triangle) - 1):
        edgeVec = triangle[p + 1] - triangle[p]
        vecToPt = pt - triangle[p]
        if edgeVec.cross(vecToPt).z < 0:
            # if all of the cross products show vecToPt is to the left of the edgeVec
            # the point is inside the polygon
            # if one is to the right, the point is outside of the polygon
            foundOutside = True
            break
    # reverse the logic (not) because above we were looking for a point outside of the polygon
    return not foundOutside

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


def getDistToLine(pt, linePt1, linePt2):
    """Given a point and two points on the line, return the distance from the point to the line."""
    # http://mathworld.wolfram.com/Point-LineDistance2-Dimensional.html
    # TODO: change getDistToLine to take a two point list for the line like all the other line functions [pt1, pt2]
    numerator = abs((linePt2.x - linePt1.x)*(linePt1.y - pt.y) - (linePt1.x - pt.x)*(linePt2.y - linePt1.y))
    return numerator / sqrt((linePt2.x - linePt1.x)**2 + (linePt2.y - linePt1.y)**2)


def scaleVec3(scale, vec):
    """Multiplies a scalar by each components of a vector and returns the resulting vector."""
    # print scale, vec
    return Vec3(scale*vec.x, scale*vec.y, scale*vec.z)

counter = -1
def getNearestPointOnLine( pt, line, asLineSeg=False):
    """Given a point and a line represented as two points,
    this returns nearest point on line to the given point. A line segment only works on the XY plane."""
    lineVec = line[1] - line[0]
    vecToPt = pt - line[0]
    # projection of vecToPt onto lineVec
    proj = scaleVec3((lineVec[0] * vecToPt[0] + lineVec[1] * vecToPt[1]) / (lineVec[0]**2 + lineVec[1]**2), lineVec)
    ptOnLine = Point3(proj + line[0])
    # if we want to constrain it to the line segment, set the nearest pt to the closest end of the line
    # unless it's already within the segment
    if asLineSeg:
        # TODO: make getNearestPointOnLine()'s line segment feature work in 3D, if possible
        lengthOfLine = getDistance(line[0], line[1])
        # if the distance from either point on the line to the point calculated is longer than the line itself,
        # then the point lies outside of the line segment
        if getDistance(ptOnLine, line[1]) > lengthOfLine or\
            getDistance(ptOnLine, line[0]) > lengthOfLine:
            if getDistance(ptOnLine, line[0]) > getDistance(ptOnLine, line[1]):
                ptOnLine = line[1]
            else:
                ptOnLine = line[0]

    return ptOnLine


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


def isPointInWedge(pt, line1, line2, inclusive=True):
    """Returns True, if the given point is inside the infinite wedge formed
    by the two lines (inclusive = True considers points on the lines to be in the wedge.)"""
    lftVec, rtVec = makeWedge(line1, line2)
    # print "isPointInWedge pt", pt, " line1 ", line1, " line2 ", line2
    # print "lftVec ", lftVec, " rtVec ", rtVec
    for i in range(0, 2):
        if line1[i] in line2:
            shared1 = i
    ptVec = pt - line1[shared1]
    # right cross pt should be up. Left cross pt should be down, if the point is inside the infinite wedge.
    if inclusive:  # points on the edge of the wedge count as in the wedge
        return rtVec.cross(ptVec).z >= 0 >= lftVec.cross(ptVec).z
    else:
        return rtVec.cross(ptVec).z > 0 > lftVec.cross(ptVec).z


# def doesEdgeCrossWedge(edge, wedgeSide1, wedgeSide2):
#     """Returns True, if either point in the edge lies within the infinite wedge formed by the two wedge sides,
#     or returns True, if the edge crosses over the infinite wedge."""
#     if isPointInWedge(edge[0], wedgeSide1, wedgeSide2) or isPointInWedge(edge[1], wedgeSide1, wedgeSide2):
#         return True
#     # check to make sure that both points of the edge aren't behind the wedge
#     # then make sure the right point is outside the right side and the left is outside the left side
