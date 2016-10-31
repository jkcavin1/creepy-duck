#!/usr/bin/python
from collections import namedtuple
import struct

from panda3d.core import Thread
from panda3d.core import Geom, GeomVertexData, GeomVertexFormat, GeomVertexReader, GeomVertexRewriter
from panda3d.core import Point3

from simpleCircle import SimpleCircle  # for the circumcircle
from utilities import getIntersectionBetweenPoints


class PrimitiveInterface(object):
    """Handles interfacing with GeomVertexData objects as well as GeomPrimitives"""
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


    def __init__(self, vdata, primitives):
        # TODO find a way to make only one per vdata
        self.vdata = vdata
        self.primitives = primitives

    def getTriangleAsPoints(self, ind, vreader=None):
        if vreader is None:
            vreader = GeomVertexReader(self.vdata, 'vertex')
        pts = []
        for vi in self.getTriangleVertexIndices(ind):
            vreader.setRow(vi)
            pt = vreader.getData3f()
            pts.append(Point3(*pt))
        return Triangle.TriangleTuple(pts[0], pts[1], pts[2])

    def getTriangleVertexIndices(self, index):
        st = self.primitives.getPrimitiveStart(index)
        end = self.primitives.getPrimitiveEnd(index)
        vertexIndicies = []
        for i in range(st, end):
            vertexIndicies.append(self.primitives.getVertex(i))
        return vertexIndicies

    def setTrianglePointIndex(self, triangleIndex, pointIndex, newVertexIndex):
        triangleArry = self.primitives.modifyVertices()
        triangleArry = triangleArry.modifyHandle(Thread.getCurrentThread())  # releases the array when deleted
        bytesPerVert = triangleArry.getArrayFormat().getTotalBytes()

        # BLOG C string to Python struct conversion https://docs.python.org/2/library/struct.html#format-characters
        fmtStr = triangleArry.getArrayFormat().getFormatString(False)  # True pads the bytes
        if fmtStr[0] != '=':
            fmtStr = '=' + fmtStr  # use standard sizing w/ = or native w/ @

        readerWriter = struct.Struct(fmtStr)  # creating the class instance saves on compiling the format string
        packed = readerWriter.pack(newVertexIndex)
        triangleArry.setSubdata(triangleIndex * bytesPerVert * 3 + pointIndex * bytesPerVert, bytesPerVert, packed)



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

    def asIndexList(self):
        return self._primitiveInterface.getTriangleVertexIndices(self._selfIndex)

    @property
    def point0(self):
        return self.asPointsEnum().point0

    @property
    def point1(self):
        return self.asPointsEnum().point1

    @property
    def point2(self):
        return self.asPointsEnum().point2

    @property
    def pointIndex0(self):
        return self._primitiveInterface.getTriangleVertexIndices(self._selfIndex)[0]

    @property
    def pointIndex1(self):
        return self._primitiveInterface.getTriangleVertexIndices(self._selfIndex)[1]

    @property
    def pointIndex2(self):
        return self._primitiveInterface.getTriangleVertexIndices(self._selfIndex)[2]

    @pointIndex0.setter
    def pointIndex0(self, value):
        self._primitiveInterface.setTrianglePointIndex(self._selfIndex, 0, value)

    @pointIndex1.setter
    def pointIndex1(self, value):
        self._primitiveInterface.setTrianglePointIndex(self._selfIndex, 1, value)

    @pointIndex2.setter
    def pointIndex2(self, value):
        self._primitiveInterface.setTrianglePointIndex(self._selfIndex, 2, value)

    def getAngleDeg0(self):
        slf = self.asPointsEnum()
        edge1 = slf.point1 - slf.point0
        edge2 = slf.point2 - slf.point0
        edge1.normalize()
        edge2.normalize()
        return edge1.angleDeg(edge2)

    def getAngleDeg1(self):
        slf = self.asPointsEnum()
        edge1 = slf.point0 - slf.point1
        edge2 = slf.point2 - slf.point1
        edge1.normalize()
        edge2.normalize()
        return edge1.angleDeg(edge2)

    def getAngleDeg2(self):
        slf = self.asPointsEnum()
        edge1 = slf.point0 - slf.point2
        edge2 = slf.point1 - slf.point2
        edge1.normalize()
        edge2.normalize()
        return edge1.angleDeg(edge2)

    def getCircumcircle(self):
        slf = self.asPointsEnum()

        edge1 = slf.point1 - slf.point0
        edge2 = slf.point2 - slf.point0

        norm = edge1.cross(edge2)
        norm.normalize()

        tanToVec1 = norm.cross(edge1)
        tanToVec2 = norm.cross(edge2)

        # the circumcircle is centered at the intersecting tangents at the midpoints of each edge (we need only 2)
        midPt1 = (slf.point0 + slf.point1) / 2
        midPt2 = (slf.point0 + slf.point2) / 2

        pt1 = Point3(tanToVec1 + midPt1)
        pt2 = Point3(tanToVec2 + midPt2)

        center = getIntersectionBetweenPoints(midPt1, pt1, pt2, midPt2)
        return SimpleCircle(center, (center - slf.point0).length())

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

    def reverse(self):
        tmp = self.pointIndex1
        self.pointIndex1 = self.pointIndex2
        self.pointIndex2 = tmp

    def __str__(self):
        return "Triangle {0}:\n\tpoint0 {1}  point1 {2} point2 {3}".format(self._selfIndex,
                                                                           self.point0, self.point1, self.point2)


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
