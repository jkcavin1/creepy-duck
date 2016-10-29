#!/usr/bin/python
from collections import namedtuple

from panda3d.core import Geom, GeomVertexData, GeomVertexFormat, GeomVertexReader, GeomVertexRewriter
from panda3d.core import Point3

from simpleCircle import SimpleCircle  # for the circumcircle
from utilities import getIntersectionBetweenPoints
from PolygonUtils.PolygonUtils import getDistance


class PrimitiveInterface(object):
    @classmethod
    def readData3f(cls, ind, vreader):
        vreader.setRow(ind)
        return vreader.getData3f()

    @classmethod
    def getCcwOrder(cls, ind0, ind1, ind2, vreader):
        pt0 = Point3(*cls.readData3f(ind0, vreader))
        pt1 = Point3(*cls.readData3f(ind1, vreader))
        pt2 = Point3(*cls.readData3f(ind2, vreader))

        rightVec = pt1 - pt0
        leftVec = pt2 - pt0
        if rightVec.cross(leftVec).z <= 0:
            tmp = ind1
            ind1 = ind2
            ind2 = tmp

        return ind0, ind1, ind2


    """Handles interfacing with GeomVertexData objects as well as GeomPrimitives"""
    def __init__(self, vdata, primitives):
        # TODO find a way to make only one per vdata
        self.vdata = vdata
        self.primitives = primitives

    def getTriangleAsPoints(self, ind, vreader=None):
        if vreader is None:
            vreader = GeomVertexReader(self.vdata, 'vertex')
        st = self.primitives.getPrimitiveStart(ind)
        end = self.primitives.getPrimitiveEnd(ind)
        pts = []
        for p in range(st, end):
            vi = self.primitives.getVertex(p)
            vreader.setRow(vi)
            pt = vreader.getData3f()
            pts.append(Point3(*pt))
        return Triangle.TriangleTuple(pts[0], pts[1], pts[2])


class Triangle(object):
    """A triangle object to help with triangle related calculations."""

    TriangleTuple = namedtuple('TriangleTuple', 'point0 point1 point2')
    # keep Triangle ignorant of other triangles as much as possible
    __slots__ = ('_selfIndex', '_primitiveInterface', '_rewriter')

    # TODO may implement descriptor for attribute access:
    # https://docs.python.org/2/reference/datamodel.html#implementing-descriptors

    def __init__(self, vindex0, vindex1, vindex2, vertexData, geomTriangles, rewriter):
        super(Triangle, self).__init__()
        inds = PrimitiveInterface.getCcwOrder(vindex0, vindex1, vindex2, rewriter)
        geomTriangles.addVertices(*inds)
        geomTriangles.closePrimitive()
        self._selfIndex = geomTriangles.getNumPrimitives() - 1
        self._primitiveInterface = PrimitiveInterface(vertexData, geomTriangles)
        self._rewriter = rewriter


    def asPointsEnum(self):
        return self._primitiveInterface.getTriangleAsPoints(self._selfIndex, vreader=self._rewriter)

    @property
    def point0(self):
        return self.asPointsEnum().point0

    @property
    def point1(self):
        return self.asPointsEnum().point1

    @property
    def point2(self):
        return self.asPointsEnum().point2

    def getMidPoint0(self):
        slf = self.asPointsEnum()
        return (slf.point0 + slf.point1) / 2.0

    def getMidPoint1(self):
        slf = self.asPointsEnum()
        return (slf.point1 + slf.point2) / 2.0

    def getMidPoint2(self):
        slf = self.asPointsEnum()
        return (slf.point2 + slf.point0) / 2.0

    def getVec0(self):
        slf = self.asPointsEnum()
        return slf.point1 - slf.point0

    def getVec1(self):
        slf = self.asPointsEnum()
        return slf.point2 - slf.point1

    def getVec2(self):
        slf = self.asPointsEnum()
        return slf.point0 - slf.point2

    def getCircumcircle(self):
        slf = self.asPointsEnum()
        # Get 2 edge mid points:
        # 0 to use to get the center after the projection by adding this point to the vector pointing at the center.
        # 1 to project onto the tangent of the 1st edge
        midPt0 = (slf.point0 + slf.point1) / 2  # edge with tangent
        midPt1 = (slf.point0 + slf.point2) / 2  # midVec (projected onto tangent)
        # get the vectors of the sides corresponding to the midpoints
        vec0 = slf.point1 - slf.point0  # tangent
        vec1 = slf.point2 - slf.point0  # midPt1
        # get the normal
        norm = vec0.cross(vec1)
        norm.normalize()
        # get the tangent
        tanToVec0 = norm.cross(vec0)
        # BOTTOM of http://paulbourke.net/geometry/pointlineplane/  ####################################
        tanToVec1 = norm.cross(vec1)
        print "tan to 0", tanToVec0, "\ntan to 1", tanToVec1
        pt1 = Point3(tanToVec0 + midPt0)
        pt2 = Point3(tanToVec1 + midPt1)
        center = getIntersectionBetweenPoints(midPt0, pt1, pt2, midPt1)
        return SimpleCircle(center, (center - slf.point0).length())

    def __str__(self):
        # slf = self.asPointsEnum()
        s = "Triangle index {0} ".format(self._selfIndex)
        s += ' point0 {0}  point1 {1} point2 {2}'.format(self.point0, self.point1, self.point2)
        return s


class TriangulatorConstrainedDelaunay(object):
    """Creates a Constrained Delaunay Triangulation"""
    def __init__(self, name='ConstrainedDelaunayTriangles', geomVertexDataObj=None,
                 format=GeomVertexFormat.getV3(), usage=Geom.UHDynamic,
                 onVertexCreationCallback=None, universalZ=0):

        if geomVertexDataObj is None:
            geomVertexDataObj = GeomVertexData(name, format, usage)
        self.vertexData = geomVertexDataObj
        # self.vertexRewriter = GeomVertexRewriter(self.vertexData, 'vertex')

        if onVertexCreationCallback is None:
            onVertexCreationCallback = lambda vdata: None
        self._onVertexCreationCallback = onVertexCreationCallback

        self.universalZ = universalZ

    def addPolygonVertex(self, ind):
        pass