#!/usr/bin/python
from collections import namedtuple
import struct

from panda3d.core import Geom, GeomVertexData, GeomVertexFormat, GeomVertexReader, GeomVertexRewriter
from panda3d.core import Thread
from panda3d.core import Point3


from simpleCircle import SimpleCircle  # for the circumcircle
from utils import getIntersectionBetweenPoints, EPSILON


class PrimitiveInterface(object):
    """Handles interfacing with GeomVertexData objects as well as GeomPrimitives"""

    @classmethod
    def readData3f(cls, ind, vreader):
        vreader.setRow(ind)
        return vreader.getData3f()

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
        vertexIndices = []
        for i in range(st, end):
            vertexIndices.append(self.primitives.getVertex(i))
        return vertexIndices

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

    @classmethod
    def getCcwOrder(cls, ind0, ind1, ind2, vreader):
        pt0, pt1, pt2 = cls.makeDummy(ind0, ind1, ind2, vreader)
        rightVec = pt1 - pt0
        leftVec = pt2 - pt0
        if rightVec.cross(leftVec).z <= 0:
            tmp = ind1
            ind1 = ind2
            ind2 = tmp

        return ind0, ind1, ind2

    @classmethod
    def getDummyMinAngleDeg(cls, ind0, ind1, ind2, vreader):
        ind0, ind1, ind2 = cls.getCcwOrder(ind0, ind1, ind2, vreader)  # ??? needs to be ccw
        pt0, pt1, pt2 = cls.makeDummy(ind0, ind1, ind2, vreader)
        v0 = pt1 - pt0
        v1 = pt1 - pt2  # reverse of triangle eg cw winding
        v2 = pt2 - pt0  # reverse of triangle
        v0.normalize()
        v1.normalize()
        v2.normalize()
        deg0 = v0.angleDeg(v2)
        deg1 = (-v2).angleDeg(v1)
        deg2 = (-v1).angleDeg(-v0)
        assert abs(deg0 + deg1 + deg2 - 180) < EPSILON
        return min(deg0, deg1, deg2)

    @classmethod
    def makeDummy(cls, ind0, ind1, ind2, vreader):
        pt0 = Point3(*PrimitiveInterface.readData3f(ind0, vreader))
        pt1 = Point3(*PrimitiveInterface.readData3f(ind1, vreader))
        pt2 = Point3(*PrimitiveInterface.readData3f(ind2, vreader))
        return pt0, pt1, pt2

    def __init__(self, vindex0, vindex1, vindex2, vertexData, geomTriangles, rewriter):
        super(Triangle, self).__init__()
        if Triangle.getDummyMinAngleDeg(vindex0, vindex1, vindex2, rewriter) <= 0:
            rewriter.setRow(vindex0)
            pt0 = rewriter.getData3f()
            rewriter.setRow(vindex1)
            pt1 = rewriter.getData3f()
            rewriter.setRow(vindex2)
            pt2 = rewriter.getData3f()
            raise ValueError("Colinear degenerate triangle points: {0} {1} {2}".format(pt0, pt1, pt2))

        inds = Triangle.getCcwOrder(vindex0, vindex1, vindex2, rewriter)
        geomTriangles.addVertices(*inds)

        self._selfIndex = geomTriangles.getNumPrimitives() - 1
        self._primitiveInterface = PrimitiveInterface(vertexData, geomTriangles)
        self._rewriter = rewriter

    def asPointsEnum(self):
        return self._primitiveInterface.getTriangleAsPoints(self._selfIndex, vreader=self._rewriter)

    def asIndexList(self):
        return self._primitiveInterface.getTriangleVertexIndices(self._selfIndex)

    def containsPoint(self, point, includeEdges=True):
        slf = self.asPointsEnum()
        v1 = slf.point2 - slf.point0
        pv1 = point - slf.point0
        v2 = slf.point1 - slf.point2
        pv2 = point - slf.point2
        v3 = slf.point0 - slf.point1
        pv3 = point - slf.point1
        if not self.isLeftWinding():
            v1 = -v1
            v2 = -v2
            v3 = -v3
        if includeEdges:
            return v1.cross(pv1).z <= 0 and v2.cross(pv2).z <= 0 and v3.cross(pv3).z <= 0
        else:
            return v1.cross(pv1).z < 0 and v2.cross(pv2).z < 0 and v3.cross(pv3).z < 0

    @property
    def edge0(self):
        slf = self.asPointsEnum()
        return slf.point0, slf.point1

    @property
    def edge1(self):
        slf = self.asPointsEnum()
        return slf.point1, slf.point2

    @property
    def edge2(self):
        slf = self.asPointsEnum()
        return slf.point2, slf.point0

    @property
    def edgeIndices0(self):
        return self.pointIndex0, self.pointIndex1

    @property
    def edgeIndices1(self):
        return self.pointIndex1, self.pointIndex2

    @property
    def edgeIndices2(self):
        return self.pointIndex2, self.pointIndex0

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
        # return SimpleCircle(center, (center - slf.point0).length())
        return SimpleCircle(center, (center - slf.point0).length())

    def getEdgeIndices0(self):
        return self.pointIndex0, self.pointIndex1

    def getEdgeIndices1(self):
        return self.pointIndex2, self.pointIndex1

    def getEdgeIndices2(self):
        return self.pointIndex0, self.pointIndex2

    def getGeomVertex(self, i):
        self._rewriter.setRow(i)
        return self._rewriter.getData3f()

    def getPointIndices(self):
        return self.pointIndex0, self.pointIndex1, self.pointIndex2

    def getIntersectionsWithCircumcircle(self, point1, point2, tolerance=EPSILON):
        circle = self.getCircumcircle()
        return circle.getIntersectionsWithLine(point1, point2, tolerance=tolerance)

    def getMidPoint0(self):
        slf = self.asPointsEnum()
        return (slf.point0 + slf.point1) / 2.0

    def getMidPoint1(self):
        slf = self.asPointsEnum()
        return (slf.point1 + slf.point2) / 2.0

    def getMidPoint2(self):
        slf = self.asPointsEnum()
        return (slf.point2 + slf.point0) / 2.0

    def getMinAngleDeg(self):
        minAng = self.getAngleDeg0()
        ang1 = self.getAngleDeg1()
        if ang1 < minAng:
            minAng = ang1
        return min(self.getAngleDeg2(), minAng)

    def getNumGeomVertices(self):
        return self._primitiveInterface.vdata.getNumRows()

    def getOccupiedEdge(self, point, slf=None):
        if not isinstance(slf, Triangle.TriangleTuple):
            slf = self.asPointsEnum()

        edge0 = slf.point1 - slf.point0
        edge1 = slf.point2 - slf.point1
        edge2 = slf.point0 - slf.point2
        # BLOG coding defensively. (This would actually be optimal if it shortcuts, but it's best to test assumptions.)
        onEdge = ''
        if edge0.cross(point - slf.point0).z == 0.0:  # only occurs if the point is on the line.
            onEdge += '0'
        if edge1.cross(point - slf.point1).z == 0.0:
            onEdge += '1'
        if edge2.cross(point - slf.point2).z == 0.0:
            onEdge += '2'

        return onEdge

    def getSharedFeatures(self, other):
        """
        returns namedtuple version of {
        'numSharedPoints': int,'point(N)': T/F, 'edge(N)': T/F,
        'indicesNotShared': (...), 'otherIndicesNotShared': (...)
        }
        or {}
        """
        inds = other.getPointIndices()
        selfInds = self.getPointIndices()

        shared = ''
        d = {
            'numSharedPoints': 0,
            'point0': False, 'point1': False, 'point2': False,
            'edge0': False, 'edge1': False, 'edge2': False,
            'indicesNotShared': self.getPointIndices(),
            'otherIndicesNotShared': other.getPointIndices(),
            'other': other,
        }
        if selfInds[0] in inds:
            d['point0'] = True
            shared += '0'
            d['numSharedPoints'] += 1
            d['indicesNotShared'] = filter(lambda i: i != selfInds[0], d['indicesNotShared'])
            d['otherIndicesNotShared'] = filter(lambda i: i != selfInds[0], d['otherIndicesNotShared'])

        if selfInds[1] in inds:
            if shared:
                d['edge0'] = True
            d['point1'] = True
            shared += '1'
            d['numSharedPoints'] += 1
            d['indicesNotShared'] = filter(lambda i: i != selfInds[1], d['indicesNotShared'])
            d['otherIndicesNotShared'] = filter(lambda i: i != selfInds[1], d['otherIndicesNotShared'])

        if selfInds[2] in inds:
            if shared == '0':
                d['edge2'] = True
            elif shared == '1':
                d['edge1'] = True
            elif shared == '01':
                d['edge0'] = True
                d['edge1'] = True
                d['edge2'] = True
            d['point2'] = True
            d['numSharedPoints'] += 1
            d['indicesNotShared'] = filter(lambda i: i != selfInds[2], d['indicesNotShared'])
            d['otherIndicesNotShared'] = filter(lambda i: i != selfInds[2], d['otherIndicesNotShared'])
        t = namedtuple('SharedNamedTuple', d.keys())
        nt = t(*d.values())
        return nt
        # if shared:
        #     return nt
        # else:
        #     return

    def getVec0(self):
        slf = self.asPointsEnum()
        return slf.point1 - slf.point0

    def getVec1(self):
        slf = self.asPointsEnum()
        return slf.point2 - slf.point1

    def getVec2(self):
        slf = self.asPointsEnum()
        return slf.point0 - slf.point2

    @property
    def index(self):
        return self._selfIndex

    def isLeftWinding(self):
        slf = self.asPointsEnum()
        v1 = slf.point1 - slf.point0
        v2 = (slf.point2 + slf.point1) / 2 - slf.point0
        return v1.cross(v2).z > 0

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

    def reverse(self):
        tmp = self.pointIndex1
        self.pointIndex1 = self.pointIndex2
        self.pointIndex2 = tmp

    def setIndex(self, value):
        self._selfIndex = value

    def __gt__(self, other):
        if isinstance(other, Triangle):
            return self._selfIndex > other.index
        else:
            return self._selfIndex > other

    def __ge__(self, other):
        if isinstance(other, Triangle):
            return self._selfIndex >= other.index
        else:
            return self._selfIndex >= other

    def __eq__(self, other):
        if isinstance(other, Triangle):
            return self._selfIndex == other.index
        else:
            return self._selfIndex == other

    def __ne__(self, other):
        if isinstance(other, Triangle):
            return self._selfIndex != other.index
        else:
            return self._selfIndex != other

    def __le__(self, other):
        if isinstance(other, Triangle):
            return self._selfIndex <= other.index
        else:
            return self._selfIndex <= other

    def __lt__(self, other):
        if isinstance(other, Triangle):
            return self._selfIndex < other.index
        else:
            return self._selfIndex < other

    def __str__(self):
        return str(self.__class__).split('.')[-1][:-2] + " {0}:\n\t{1}, {2}, {3}\n\tind: {4} {5} {6}".format(
            self._selfIndex,
            self.point0, self.point1, self.point2,
            self.pointIndex0, self.pointIndex1, self.pointIndex2
        )
