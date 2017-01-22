__author__ = 'Lab Hatter'

# from panda3d.core import Triangulator
from panda3d.core import Point2D, Point3, Vec4, Vec3
from panda3d.core import GeomVertexFormat, GeomVertexData, GeomLines, Triangulator
from panda3d.core import Geom, GeomNode, GeomTriangles, GeomVertexWriter, ModelNode, NodePath
from math import sqrt, pow
from PolygonUtils import getDistance, isPointInWedge
from Triangle import Triangle, getSharedEdgeStr


def copyAdjLstElement(adjLstEl):
    cpy = AdjLstElement(adjLstEl.tri, adjLstEl.selfInd, adjLstEl.n12, adjLstEl.n23, adjLstEl.n13)
    cpy.par = adjLstEl.par
    cpy.g = adjLstEl.g
    cpy.f = adjLstEl.f
    return cpy


class AdjLstElement(Triangle):
    def __init__(self, triOrPts, slfInd, n12Ind=None, n23Ind=None, n13Ind=None):
        """If points are past they must be in a tuple-like object"""
        if isinstance(triOrPts, Triangle):
            Triangle.__init__(self, triOrPts.pt1, triOrPts.pt2, triOrPts.pt3)
        else:
            Triangle.__init__(self, triOrPts[0], triOrPts[1], triOrPts[2])

        self.selfInd = slfInd
        self.n12 = n12Ind  # neighbour on 12's edge
        self.w2313 = -1  # width when crossing edges 23 & 13
        self.n23 = n23Ind
        self.w1213 = -1
        self.n13 = n13Ind
        self.w1223 = -1
        self.par = None
        self.g = 100000
        self.f = 100000

    def getNaybs(self):
        n = []
        if self.n12 is not None:
            n.append(self.n12)
        if self.n23 is not None:
            n.append(self.n23)
        if self.n13 is not None:
            n.append(self.n13)
        return n

    def getDistanceToCentersOrPoint(self, other):
        if isinstance(other, tuple((AdjLstElement, Triangle))):
            pt1 = self.getCenter()
            pt2 = other.getCenter()
        else:
            pt1 = self.getCenter()
            pt2 = other

        return getDistance(pt1, pt2)

    def getNearestPointTo(self, pt):
        minInd = 0
        minDist = getDistance(self.tri[0], pt)
        for i in range(0, len(pt)):
            dist = getDistance(self.tri[i], pt)
            if dist < minDist:
                minDist = dist
                minInd = i

        return self.tri[minInd]


    def isConstrained(self, pt):
        """Returns true if the point is on an edge with no neighbour"""
        if pt not in self.tri:
            raise Exception("isConstrained: Point must be in this triangle." +
                            self.__repr__() + " pt " + str(pt))

        e12 = self.tri[:-1]
        e23 = self.tri[1:]
        e13 = [self.tri[0], self.tri[2]]
        if pt in e12 and self.n12 is None:
            return True
        elif pt in e23 and self.n23 is None:
            return True
        elif pt in e13 and self.n13 is None:
            return True
        else:
            #print "false: pt", pt, "self", self, "  naybs ", self.n12, self.n23, self.n13
            return False

    def getOppositePoints(self, pt):
        pts = []
        for p in self.tri:
            if p != pt:
                pts.append(p)
        return pts


    def __eq__(self, other):
        return self.selfInd == other.selfInd


    def __ne__(self, other):
        return self.selfInd != other.selfInd


    def __repr__(self):
        sr = "tri: < " + str(self.getTri()) +\
             " >   < slf: " + str(self.selfInd) +\
            " n12: " + str(self.n12) +\
            " n23: " + str(self.n23) +\
            " n13: " + str(self.n13) +\
            " >   < par: " + str(self.par) #+\
            # ", g: " + str(self.g) +\
            # ", f: " + str(self.f) + " >"

        return sr


class AdjacencyList(object):
    def __init__(self, triangles):
        triLst = []
        self.adjLst = [] # triangles
        if isinstance(triangles, Triangulator):
            # get the full list of triangles because we can't search a partial list
            for i in range(0, triangles.getNumTriangles()):
                v0 = triangles.getVertex(triangles.getTriangleV0(i))
                v1 = triangles.getVertex(triangles.getTriangleV1(i))
                v2 = triangles.getVertex(triangles.getTriangleV2(i))
                self.adjLst.append(AdjLstElement(Triangle(Point3(v0.x, v0.y, 0),
                                                      Point3(v1.x, v1.y, 0),
                                                      Point3(v2.x, v2.y, 0)), i))
        elif isinstance(triangles, AdjacencyList):
            for i in range(0, len(triangles.adjLst)):
                self.adjLst.append(AdjLstElement(triangles.adjLst[i].tri, i))
        else:  # should be a list or a Triangulator
            for i in range(0, len(triangles)):
                self.adjLst.append(AdjLstElement(triangles[i].tri, i))

        # adjLst in the form [(triangle, n12, n23, n13)] i.e. n12 is the triInd across verts 1 & 2
        # find the neighbours
        for j in range(0, len(self.adjLst)):
            for k in range(0, len(self.adjLst)):
                if j != k:
                    nayb = getSharedEdgeStr(self.adjLst[j], self.adjLst[k])
                    if nayb == '12':
                        self.adjLst[j].n12 = k
                    if nayb == '23':
                        self.adjLst[j].n23 = k
                    if nayb == '13':
                        self.adjLst[j].n13 = k

    def at(self, ind):
        return self.adjLst[ind]

    def getNaybsAt(self, ind):
        return self.adjLst[ind].getNaybs()



def makeTriMesh( verts, holeVerts=[[]]):
    pointmap = (lambda x, y: (x, y, 0))
    if not holeVerts:
        holeVerts = [[]]

    if not hasattr(holeVerts[0], '__iter__'):
        holeVerts = [holeVerts]

    frmt = GeomVertexFormat.getV3n3cp()
    vdata = GeomVertexData('triangle', frmt, Geom.UHDynamic)

    vertex = GeomVertexWriter(vdata, 'vertex')
    normal = GeomVertexWriter(vdata, 'normal')
    color = GeomVertexWriter(vdata, 'color')

    bl = Vec4(50, 50, 50, 255)
    gr = Vec4(256/2 - 1, 256/2 - 1, 256/2 - 1, 255)

    trilator = Triangulator()
    zUp = Vec3(0, 0, 1)

    for i in verts:
        #print "verts", verts
        trilator.addPolygonVertex(trilator.addVertex(i.x, i.y))
        vertex.addData3f(pointmap(i.x, i.y))
        normal.addData3f(zUp)
        color.addData4f(bl)
    #if len(holeVerts) != 1 and holeVerts[0] != []:
    for w in holeVerts:
        trilator.beginHole()
        print "new hole"
        for j in w:
            # print(j)  # ###################### PRINT #######################
            trilator.addHoleVertex(trilator.addVertex(j.x, j.y))
            vertex.addData3f(pointmap(j.x, j.y))
            normal.addData3f(zUp)
            color.addData4f(gr)

    # try:
    trilator.triangulate()
    triVerts = trilator.getVertices()
    p = trilator.getTriangleV0(0)
    print "trilator return", p
    # except AssertionError:
    #     pass
    # TODO:re-triangulate here and change AdjacencyList to expect a list of triangles rather than call Triangulator funcs
    trilator = makeDelaunayTriangulation(trilator)


    prim = GeomTriangles(Geom.UHStatic)
    if isinstance(trilator, Triangulator):
        # HACK just to switch back and forth from the non-Delaunay to the Delaunay for school
        for n in xrange(trilator.getNumTriangles()):
            prim.addVertices(trilator.getTriangleV0(n),
                             trilator.getTriangleV1(n),
                             trilator.getTriangleV2(n))
    else:  # it's an adjacency list

        for n in xrange(len(trilator.adjLst)):
            for v in range(0, len(triVerts)):
                print "\ncurr", triVerts[v], "\nv0", Point2D(trilator.adjLst[n].tri[0].x, trilator.adjLst[n].tri[0].y),\
                    "\nv1", Point2D(trilator.adjLst[n].tri[1].x, trilator.adjLst[n].tri[1].y),\
                    "\nv2", Point2D(trilator.adjLst[n].tri[2].x, trilator.adjLst[n].tri[2].y)
                    # trilator.adjLst[n].tri[0].getXy(),\
                    # "\nv1", trilator.adjLst[n].tri[1].getXy(),\
                    # "\nv2", trilator.adjLst[n].tri[2].getXy()
                # v0 = v1 = v2 = -1
                if Point2D(trilator.adjLst[n].tri[0].x, trilator.adjLst[n].tri[0].y) == triVerts[v]:
                    v0 = v  # we need indices into the vertex pool
                    print "found v0", v0

                if Point2D(trilator.adjLst[n].tri[1].x, trilator.adjLst[n].tri[1].y) == triVerts[v]:
                    v1 = v
                    print "found v1", v1

                if Point2D(trilator.adjLst[n].tri[2].x, trilator.adjLst[n].tri[2].y) == triVerts[v]:
                    v2 = v
                    print "found v2", v2
            # if v0 == -1 or v1 == -1 or v2 == -1:
            #     print "pass", v0, v1, v2
            #     pass
            # else:
            #     print "add"

            # i = triVerts[v0]
            # vertex.addData3f(pointmap(i.x, i.y))
            # normal.addData3f(zUp)
            # color.addData4f(bl)
            # i = triVerts[v1]
            # vertex.addData3f(pointmap(i.x, i.y))
            # normal.addData3f(zUp)
            # color.addData4f(bl)
            # i = triVerts[v2]
            # vertex.addData3f(pointmap(i.x, i.y))
            # normal.addData3f(zUp)
            # color.addData4f(bl)
            print "v 1 2 3", v0, v1, v2
            prim.addVertices(v0, v1, v2)

    prim.closePrimitive()
    geom = Geom(vdata)
    geom.addPrimitive(prim)
    node = GeomNode('gnode')
    node.addGeom(geom)

    # print trilator.isLeftWinding()
    # HACK just for school
    if hasattr(trilator, "adLst"):
        return tuple((node, trilator.adjLst))
    else:
        return tuple((node, trilator))

def makeDelaunayTriangulation(triangulator):
    """Takes a triangulation and turns it into a Delaunay triangulation"""
    # http://www.cs.uu.nl/geobook/interpolation.pdf
    # TODO: use this in a Triangulator object instead http://www.geom.uiuc.edu/~samuelp/del_project.html
    #return triangulator
    def getMinAngle(tri1, tri2):
        tri1vec1 = tri1[0] - tri1[1]
        tri1vec2 = tri1[1] - tri1[2]
        tri1vec3 = tri1[0] - tri1[2]
        # reverse the vector, so we don't get 180 degrees off by using the opposite vector
        tri1ang12 = abs(tri1vec1.relativeAngleDeg(-tri1vec2))
        tri1ang23 = abs(tri1vec2.relativeAngleDeg(tri1vec3))
        tri1ang13 = abs(tri1vec1.relativeAngleDeg(-tri1vec3))

        minAng = min(tri1ang12, tri1ang23, tri1ang13)

        tri2vec1 = tri2[0] - tri2[1]
        tri2vec2 = tri2[1] - tri2[2]
        tri2vec3 = tri2[0] - tri2[2]
        tri2ang12 = abs(tri2vec1.relativeAngleDeg(-tri2vec2))
        tri2ang23 = abs(tri2vec2.relativeAngleDeg(tri2vec3))
        tri2ang13 = abs(tri2vec1.relativeAngleDeg(-tri2vec3))

        if minAng > min(tri2ang12, tri2ang23, tri2ang13):
            minAng = min(tri2ang12, tri2ang23, tri2ang13)

        return minAng

    triLst = []
    for i in range(0, triangulator.getNumTriangles()):
        v0 = triangulator.getVertex(triangulator.getTriangleV0(i))
        v1 = triangulator.getVertex(triangulator.getTriangleV1(i))
        v2 = triangulator.getVertex(triangulator.getTriangleV2(i))
        triLst.append(Triangle(Point3(v0.x, v0.y, 0),
                                      Point3(v1.x, v1.y, 0),
                                      Point3(v2.x, v2.y, 0)))
    triLst = AdjacencyList(triLst)
    invalidFound = True
    while invalidFound:
        print "whild True"
        for i in range(0, len(triLst.adjLst)):
            needsReset = False
            currTri = triLst.adjLst[i]
            print "currTri\n", currTri
            if triLst.adjLst[i].n12:  # if there's a neighbour on the 12 edge get it
                n1 = triLst.adjLst[triLst.adjLst[i].n12]
            else:
                n1 = None

            if triLst.adjLst[i].n23:  # if there's a neighbour on the 23 edge get it
                n2 = triLst.adjLst[triLst.adjLst[i].n23]
            else:
                n2 = None

            if triLst.adjLst[i].n13:  # if there's a neighbour on the 12 edge get it
                n3 = triLst.adjLst[triLst.adjLst[i].n13]
            else:
                n3 = None
            # TODO: arange these from highest to lowest according to their longest shared edge
            naybs = [n1, n2, n3]
            for n in naybs:
                if n is not None:
                    sharedPts = currTri.getSharedPoints(n)  # two points shared between the triangles
                    notSharedPt = currTri.getNonSharedPoint(n)  # one point in the current triangle, not in the other
                    notSharedPtN = n.getNonSharedPoint(currTri)  # one point in the other, not in the current
                    print "nayb not None"
                    # make sure this is a convex quadrilateral (else we would cut outside of the two triangles)
                    # the non-shared point in the other triangle should be in the wedge formed by this triangle
                    if isPointInWedge(notSharedPtN,
                                      [sharedPts[0], notSharedPt],
                                      [sharedPts[1], notSharedPt], inclusive=False):  # don't include points on the edge
                        # make two new triangles out of the polygon with a test-slice
                        newCurr = copyAdjLstElement(currTri)
                        newCurr.setTri(notSharedPt, notSharedPtN, sharedPts[0])
                        newN = copyAdjLstElement(n)
                        newN.setTri(notSharedPt, notSharedPtN, sharedPts[1])
                        print "point in wedge old new curr\n", currTri.tri, "\n", n.tri
                        minAngOld = getMinAngle(currTri.tri, n.tri)
                        minAngNew = getMinAngle(newCurr.tri, newN.tri)
                        if minAngNew > minAngOld:
                            print "minAngNew > minAngOld"
                            # maximize the minimum angle == Delaunay Triangulation
                            triLst.adjLst[i] = newCurr
                            triLst.adjLst[n.selfInd] = newN
                            # we've changed the adjacency list
                            # build a new one
                            triLst = AdjacencyList(triLst.adjLst)
                            # start over from the beginning
                            needsReset = True
                            break
            if needsReset:
                break
        # if we went through the whole list and didn't find an illegal triangle we're done
        if i == len(triLst.adjLst) - 1 and not needsReset:
            invalidFound = False

    return triLst
