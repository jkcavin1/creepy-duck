__author__ = 'Lab Hatter'
from direct.showbase.ShowBase import ShowBase
from panda3d.core import Mat4, Mat3, Vec3, Point3, NodePath, NodePathCollection
"""
TODO: add functions for initializing Panda objects such as
loading Models, Actors
setting up GUI objects (later make 2D editor)
setting up/(maybe)switching input i.e. keystrokes
setting up collision nodes w/ solids
including tasks???
"""

def PanditorDisableMouseFunc():
    base.disableMouse()


def PanditorEnableMouseFunc():
    mat = Mat4(camera.getMat())
    mat.invertInPlace()
    base.mouseInterfaceNode.setMat(mat)
    base.enableMouse()


def TranslateWrtNPFunc( wrtNP, translatingNP, directVec, velocity, time, normalize=True):
    if normalize is True:
        directVec.normalize()

    toPoint = directVec * velocity * time

    translatingNP.setPos(wrtNP, toPoint.getX(), toPoint.getY(), toPoint.getZ())


def TraverserLevelFirst(startNP, funcOpOnNP, *args, **kwargs):
    npCollection = NodePathCollection()
    currNP = startNP
    npCollection.addPath(currNP)
    notEmtpy = True
    while notEmtpy:
            if currNP.countNumDescendants() == 0:
                # no descendants to add to the stack
                pass
            else:
                # add the currNP's children to stack
                currChildColl = currNP.getChildren()
                npCollection.extend(currChildColl)

            if npCollection.getNumPaths() > 0:
                # operate on the current node
                funcOpOnNP(currNP, *args, **kwargs)
                npCollection.removePath(currNP)
                # get currNP's sister
                if npCollection.getNumPaths() > 0:
                    currNP = npCollection.getPath(0)
            else:
                notEmtpy = False  # scene has been walked in level order
                currNP = None


#######
####### http://www.panda3d.org/forums/viewtopic.php?t=5817

    # Classes and functions for polygons, convex hulls (including
    # bounding container convex hulls), point-in-polygon test and so on.

    #import psyco
    ##psyco.full()
    #psyco.profile()


def isConvex(hull):
    '''Returns True if the given points form convex hull with vertices
    in ccw order.'''

    def _isLeft(q, r, p):
        return (r[0]-q[0])*(p[1]-q[1]) - (p[0]-q[0])*(r[1]-q[1])

    i = -2
    j = -1
    for k in range(len(hull)):
        p = hull[i]
        q = hull[j]
        r = hull[k]
        if _isLeft(p, q, r) <= 0:
            return False
        i = j
        j = k
    return True

def monotone_chain(points):
    '''Returns a convex hull for an unordered group of 2D points.

    Uses Andrew's Monotone Chain Convex Hull algorithm.'''

    def _isLeft(q, r, p):
        return (r[0]-q[0])*(p[1]-q[1]) - (p[0]-q[0])*(r[1]-q[1])

    # Remove duplicates (this part of code is useless for Panda's
    # Point2 or Point3! In their case set() doesn't remove duplicates;
    # this is why internally this class has all points as (x,y) tuples).
    points = list(set(points))

    # Sort points first by X and then by Y component.
    points.sort()
    # Now, points[0] is the lowest leftmost point, and point[-1] is
    # the highest rightmost point. The line through points[0] and points[-1]
    # will become dividing line between the upper and the lower groups
    # of points.

    p0x, p0y = points[0]
    p1x, p1y = points[-1]

    # Initialize upper and lower stacks as empty lists.
    U = []
    L = []

    # For each point:
    for p in points:

        # First, we check if the point in # i.e. points is left or right or
        # collinear to the dividing line through points[0] and points[-1]:
        cross = (p1x-p0x)*(p[1]-p0y) - (p[0]-p0x)*(p1y-p0y)

        # If the point is lower or colinear, test it for inclusion
        # into the lower stack.
        if cross <= 0:
            # While L contains at least two points and the sequence
            # of the last two points in L and p does not make
            # a counter-clockwise turn:
            while len(L) >= 2 and _isLeft(L[-2], L[-1], p) <= 0:
                L.pop()
            L.append(p)

        # If the point is higher or colinear, test it for inclusion
        # into the upper stack.
        if cross >= 0:
            # While U contains at least two points and the sequence
            # of the last two points in U and p does not make
            # a clockwise turn:
            while len(U) >= 2 and _isLeft(U[-2], U[-1], p) >= 0:
                U.pop()
            U.append(p)

    L.pop()
    U.reverse()
    U.pop()

    return L+U

def isLeft(qx, qy, rx, ry, px, py):
    '''Returns 2D cross product:
        >0 for p left of the infinite line through q and r,
        =0 for p on the line,
        <0 for p right of the line.

    Requires 6 integers or floats for 3 consecutive points: point1_x, point1_y,
    point2_x, point2_y, point3_x, point3_y.'''

    return (rx-qx)*(py-qy) - (px-qx)*(ry-qy)

def isLeft2(q, r, p):
    '''Returns 2D cross product:
        >0 for p left of the infinite line through q and r,
        =0 for p on the line,
        <0 for p right of the line.

    Requires three consecutive points with (X, Y) as input. If present,
    Z component is ignored.'''

    return (r[0]-q[0])*(p[1]-q[1]) - (p[0]-q[0])*(r[1]-q[1])

class AABB2D():
    '''Axis-aligned bounding box for 2D shapes.

    Represents minimum and maximum X and Y coordinates for the contained
    shape.'''

    def __init__(self, points):
        '''To be constructed, AABB requires a list of points with two
        coordinates (X, Y) each. If present, Z coordinate is ignored.'''

        self.minX = self.maxX = points[0][0]
        self.minY = self.maxY = points[0][1]

        self._calc(points)

    def _calc(self, points):
        i = 1 # Not from 0!
        while i < len(points):
            px, py = points[i]
            self.minX = min(self.minX, px)
            self.maxX = max(self.maxX, px)
            self.minY = min(self.minY, py)
            self.maxY = max(self.maxY, py)
            i += 1

    def getCenter(self):
        '''Returns the center of the AABB.'''

        x = (self.minX + self.maxX) / 2.0
        y = (self.minY + self.maxY) / 2.0
        return (x, y)

    def isInside(self, point):
        '''Returns True if the given point is inside of this AABB.'''

        px, py = point
        if (px - self.minX) * (px - self.maxX) < 0:
            return False
        if (py - self.minY) * (py - self.maxY) < 0:
            return False
        return True

    def intersect(self, aabb):
        '''Tests if this AABB intersects with another AABB.

        Returns a tuple of values that describe the area of intersection:
        (leftmost X of intersection, rightmost X of itersection,
        lowest Y of intersection, highest Y of itersection),
        or (min X, max X, min Y, max Y).
        Or 'False' if they don't intersect.'''

        if self.minX > aabb.maxX or self.maxX < aabb.minX:
            return False
        if self.minY > aabb.maxY or self.maxY < aabb.minY:
            return False
        minX = max(self.minX, aabb.minX)
        maxX = min(self.maxX, aabb.maxX)
        minY = max(self.minY, aabb.minY)
        maxY = min(self.maxY, aabb.maxY)
        return (minX, maxX, minY, maxY)

class ConvexPolygon():
    '''Class for convex polygons in 2D.

    To be constructed, requires 3 or more different points/vertices.
    Assumes that polygon is convex, and all vertices are given in
    counter-clockwise order.

    Acceptable format for points is "(x, y)", Panda's built-in Point2 or
    Point3 (but internally they all are transformed into "(x, y)" tuple).

    It can be used to create a bounding convex hull for an unordered group
    of points. To do this, pass keyword argument "create=True" to
    the constructor. Then duplicate and redundant points will be removed
    and the new "clean" bounding convex hull will be created by applying
    Andrew's Monotone Chain Convex Hull algorithm:
    http://www.algorithmist.com/index.php/Monotone_Chain_Convex_Hull'''

    def __init__(self, *args, **kwargs):
        '''Pass it least 3 vertices into constructor.

        Vertices are assumed to be on 2D plane and be given in
        counter-clockwise order.'''

        if len(args) == 1:
            args = args[0]
        points = []
        for v in args:
            if not (isinstance(v, tuple) or isinstance(v, list)):
                v = (v[0], v[1])
            points.append(v)

        # Initially, it is assumed that given points are vertices of
        # a convex hull in ccw order:
        self.vertices = points

        if "create" in kwargs:
            if kwargs["create"]:
                # Create bounding convex hull:
                self.vertices = monotone_chain(points)

        self.numVertices = len(self.vertices)
        # Create bounding box:
        self.aabb = AABB2D(self.vertices)

    def __repr__(self):
        vl = self.vertices
        for i in range(self.numVertices):
            vl[i] = str(vl[i])
        res = "\n".join(vl)
        return res

    def isInside(self, point):
        '''Simplified winding number algorithm for convex polys.

        If the point is within the poly then the endVertex of any edge
        must always be to the left from the infinite line through point and
        startVertex. If they are colinear and the point is inside of
        the AABB of the edge, the point is actually on the edge.

        Currently, this implementation is the fastest one (2.4 seconds
        for 1 mln executions on my computer).'''

        px = point[0]
        py = point[1]

        i = -1
        j = 0
        while j < self.numVertices:
            qx, qy = self.vertices[i]
            rx, ry = self.vertices[j]

            x = (rx-qx)*(py-qy) - (px-qx)*(ry-qy)
            if x <= 0:
                if x == 0: # for colinear
                    if (px - qx) * (px - rx) > 0:
                        return False
                    if (py - qy) * (py - ry) > 0:
                        return False
                    return True
                return False # for right turns

            i = j
            j += 1

        return True

    def windingNumber(self, point):
        '''Winding number point-in-polygon test.

        Has some difficulties when the point is located on an upward edge.
        Reference:
        http://softsurfer.com/Archive/algorithm_0103/algorithm_0103.htm'''

        def _isLeft(qx, qy, rx, ry, px, py):
            return (rx-qx)*(py-qy) - (px-qx)*(ry-qy)

        wn = 0
        px = point[0]
        py = point[1]

        i = -1
        j = 0
        while j < self.numVertices:
            qx, qy = self.vertices[i]
            rx, ry = self.vertices[j]

            if (px, py) == (qx, qy):
                return True

            if qy <= py:
                if ry > py:
                    cross = _isLeft(qx, qy, rx, ry, px, py)
                    if cross > 0:
                        wn += 1
            else:
                if ry <= py:
                    cross = _isLeft(qx, qy, rx, ry, px, py)
                    if cross < 0:
                        wn -= 1

            i = j
            j += 1

        if wn == 0:
            return False
        return True

    def crossingNumber(self, point):
        '''Crossing number point-in-polygon test.

        Has some difficulties when the point is located on an upward edge.
        Reference:
        http://softsurfer.com/Archive/algorithm_0103/algorithm_0103.htm'''

        cn = 0
        px = point[0]
        py = point[1]

        i = -1
        j = 0
        while j < self.numVertices:
            qx, qy = self.vertices[i]
            rx, ry = self.vertices[j]

            if (px, py) == (qx, qy):
                return True

            if (    ((qy <= py) and (ry > py)) or \
                    ((qy > py) and (ry <= py))    ):
                vt = (py - qy) / (ry - qy)
                if (px < qx + vt * (rx - qx)):
                    cn += 1

            i = j
            j += 1

        return cn%2


"""
if __name__ == '__main__':
    class HelperTester(ShowBase):
        def __init__(self):
            ShowBase.__init__(self)
            PanditorDisableMouseFunc()
            # Load the environment model.
            self.environ = self.loader.loadModel("models/environment")
            # tag recommended by https://www.panda3d.org/manual/index.php/Clicking_on_3d_objects
            #self.environ.setTag( 'Blah', '1' )
            # TODO: See chessboard for collision testing
            # Reparent the model to render.
            self.environ.reparentTo(self.render)
            # Apply scale and position transforms on the model.
            self.environ.setScale(0.25, 0.25, 0.25)
            self.environ.setPos(-8, 42, 0)

        def EnableMouse(self):
            PanditorEnableMouseFunc()

    tst = HelperTester()
    tst.run()
"""
