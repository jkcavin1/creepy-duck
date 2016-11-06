#!/usr/bin/python
import exceptions
import math

from panda3d.core import Point3, Vec3, Mat3

import utils as utilities

class SimpleCircle(object):
    __slots__ = ('center', 'radius', )

    @classmethod
    def getCirclesBorder(cls, circle, numSlices=32, closed=False):
        degPerSlice = 360.0 / numSlices
        zVec = Vec3.unitZ()
        rotMat = Mat3.rotateMat(degPerSlice, zVec)
        projVec = Vec3.unitY() + circle.center
        projVec.normalize()
        border = [Point3(projVec * circle.radius + circle.center), ]
        projVec = rotMat.xform(projVec)
        for i in range(1, numSlices):
            pt = Point3(projVec * circle.radius + circle.center)
            border.append(pt)
            projVec = rotMat.xform(projVec)


        if closed and border:
            border.append(border[0])

        return border

    def __init__(self, center, radius):
        super(SimpleCircle, self).__init__()
        self.center = center
        self.radius = radius

    def getBorder(self, numSlices=32, closed=False):
        return SimpleCircle.getCirclesBorder(self, numSlices, closed)


    def isPointInside(self, point, tolerance=0.0):
        return (point - self.center).length() - self.radius < tolerance

    def getIntersectionsWithLine(self, point1, point2, tolerance=utilities.EPSILON):
        # http://mathworld.wolfram.com/Circle-LineIntersection.html
        # the solution requires a centered circle. So, get the difference
        zeroPt = Point3(0.0)
        toCenter = zeroPt - self.center  # overkill but readable
        # translate the points
        pointTranslated1 = point1 + toCenter
        pointTranslated2 = point2 + toCenter
        dx = pointTranslated2.x - pointTranslated1.x
        dy = pointTranslated2.y - pointTranslated1.y
        dr = math.sqrt(dx**2 + dy**2)
        D = pointTranslated1.x * pointTranslated2.y - pointTranslated2.x * pointTranslated1.y
        determinate = self.radius**2 * dr**2 - D**2

        # BLOG DRY principle and functional programming (sgn used more than once, thus it's name not lambda)
        def sgn(x):
            if x < 0:
                return -1
            else:
                return 1

        def quadratic(d, y, x, funcOfY, sign=-1):
            # Subtraction is defined in terms of addition. So, multiply by 1 or -1, for the sign, and add in either case.
            return (D * d + sign * funcOfY(y) * x * math.sqrt(self.radius**2 * dr**2 - D**2)) / dr**2

        # TODO Check for error between determinate and quadratic()
        xes = ys = []
        # blindly solve for the two x values (there might not be an intersection) and the two y values
        # NOTE: all resulting points can be on the circle eg (xPos, yPos), (xPos, yNeg), (xNeg, yPos), (xNeg, yNeg)
        try:
            xPos = quadratic(dy, dy, dx, sgn, sign=1)
        except ValueError:
            xPos = float('NaN')
        else:
            xes.append(xPos)
        try:
            xNeg = quadratic(dy, dy, dx, sgn, sign=-1)
        except ValueError:
            if math.isnan(xPos):
                assert determinate < 0  # no intersection
                return ()
            xNeg = float('NaN')
        else:
            xes.append(xNeg)

        # ditto y values
        try:
            yPos = quadratic(-1*dx, dy, 1, abs, sign=1)
        except ValueError:
            yPos = float('NaN')
        else:
            ys.append(yPos)
        try:
            yNeg = quadratic(-1*dx, dy, 1, abs, sign=-1)
        except ValueError:
            if math.isnan(yPos):
                assert determinate < 0
                return ()
            yNeg = float('Nan')
        else:
            ys.append(yNeg)

        def isNearlyEqual(p1, p2):
            return abs(p1.x - p2.x) < tolerance and abs(p1.y - p2.y) < tolerance

        if determinate < 0:
            return ()  # no intersection

        elif determinate == 0:  # one intersection, and it's a tangent
            for x in xes:
                for y in ys:
                    pt = Point3(x, y, 0.0)
                    if (zeroPt - pt).length() - self.radius < tolerance:
                        return Point3(pt - toCenter)  # translate back to object space and make Point3
        else:  # determinate > 0 >> two intersections (secant line)
            pts = []
            for x in xes:
                for y in ys:
                    pt = Point3(x, y, 0.0) - toCenter
                    if (pt - self.center).length() - self.radius < tolerance:
                        isIn = False
                        for p in pts:
                            if isNearlyEqual(p, pt):
                                isIn = True
                                break
                        if isIn:
                            continue
                        pts.append(Point3(pt))  # translate back to object space and make Point3
            if len(pts) != 2:  # enforce the 2 intersections expectation
                exToThrow = exceptions.ArithmeticError(
                    "simpleCircle.getIntersectionsWithLine() should intersect only twice\n\t." +
                    " self {0} point1 {1} point2 {2} tol {3} result {4}".format(self, point1, point2, tolerance, pts)
                )
                if len(pts) < 2:  # no known (to me) solution otherwise
                    raise exToThrow
                def slope(p1, p2):
                    return (p2.y - p1.y)/(p2.x - p1.x)
                # all points may be on the circle's border but not more than two form the proper line
                # find the first point and calculate the slope to determine which points we need
                if point1.x < point2.x:
                    m = slope(point1, point2)
                elif point2.x < point1.x:
                    m = slope(point2, point1)
                elif yPos != yNeg:  # Same x (undefined slope). Any y creating a point on the border will do
                    p1 = pts[0]
                    for i in range(1, len(pts)):
                        if p1.y != pts[i].y:
                            return (p1, pts[i])
                    raise exToThrow
                else:  # equal Ys and equal Xs which is wrong
                    raise exToThrow
                # handle the slope == m cases
                for i in range(0, len(pts)):
                    for j in range(i + 1, len(pts)):
                        if pts[i].x == pts[j].x:  # undefined slope (div by zero)
                            continue
                        if pts[i].x < pts[j].x:
                            low = pts[i]
                            hi = pts[j]
                        else:
                            low = pts[j]
                            hi = pts[i]
                        if slope(low, hi) == m:
                            return low, hi
                raise exToThrow
            return pts
        raise exceptions.ArithmeticError("simpleCircle.getIntersectionsWithLine() defaulted without exiting." +
                                         " self {0} point1 {1} point2 {2} tol {3}".format(self, point1, point2,
                                                                                          tolerance))

    def __str__(self):
        return "radius {0}, center: {1}".format(self.radius, self.center)

    def __eq__(self, other):
        return self.center == other.center and self.radius == other.radius
