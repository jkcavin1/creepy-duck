#!/usr/bin/python
import sys
import heapq, collections
from collections import namedtuple

from panda3d.core import Geom, GeomNode
from panda3d.core import GeomVertexData, GeomVertexFormat, GeomTriangles, GeomVertexReader, GeomVertexRewriter
from panda3d.core import Thread
from panda3d.core import Point3
from direct.directnotify.DirectNotify import DirectNotify

from triangle import Triangle
from utils import getIntersectionBetweenPoints

notify = DirectNotify().newCategory("DelaunayTriangulator")
# selfEdgeCallCount = otherEdgeCallCount = 0


class ConstrainedDelaunayAdjacencyTriangle(Triangle):
    __slots__ = ('_neighbor0', '_neighbor1', '_neighbor2', )

    class AdjacencyTuple(namedtuple("AdjacencyTuple", 'index neighbor0 neighbor1 neighbor2')):
        __slots__ = ()

        # def asAdjacencyTriangle(self, vdata, geomTriangles, rewriter, triangle=None):
        #     try:
        #         return triangle.resetFromTuple(self, vdata, geomTriangles, rewriter)
        #     except TypeError:
        #         return ConstrainedDelaunayAdjacencyTriangle(self.neighbor0, self.neighbor1, self.neighbor2,
        #                                                     vdata, geomTriangles, rewriter, self.index)


    def __init__(self, vindex0, vindex1, vindex2, vertexData, geomTriangles, rewriter, asAdjTuple=None):
        super(ConstrainedDelaunayAdjacencyTriangle, self).__init__(vindex0, vindex1, vindex2,
                                                                   vertexData, geomTriangles, rewriter)
        if asAdjTuple is not None:
            self._neighbor0 = asAdjTuple.neighbor0
            self._neighbor1 = asAdjTuple.neighbor1
            self._neighbor2 = asAdjTuple.neighbor2
        else:
            self._neighbor0 = None
            self._neighbor1 = None
            self._neighbor2 = None

    # ##################### NEW For Make Delaunay ##########################
    def legalizeEdge0(self, _triangleList):
        """
        Maximizes the minimum angle by swapping the edge, if need be.
        Returns the other triangle if a swap occurred, None if not.
        """
        # BLOG optimal throw (most of the time there will be a neighbor, hence try rather than if)
        # global notify
        # notify.warning("legalizeEdge0 self {0}:{1} n:{2}".format(self.index,
        #                                                          self.getIndices(),
        #                                                          self.getNeighbors()))
        try:
            other = _triangleList[self._neighbor0]
        except TypeError:
            if self._neighbor0 is None:
                return None
            else:
                # http://nedbatchelder.com/blog/200711/rethrowing_exceptions_in_python.html (confirmed to raise original)
                raise
        # get the shared edge. So, we can make 'ghost' triangles
        shared = self.getSharedFeatures(other)
        # maximize the minimum edge
        # it's not legal if the smallest angle using the other edge is larger than the current smallest angle

        # global notify
        # notify.warning("legalizeEdge0 self {0}:{1} n:{2} | other  {3}:{4} n:{5}".format(self.index,
        #                                                                                 self.getIndices(),
        #                                                                                 self.getNeighbors(),
        #                                                                                 other.index,
        #                                                                                 other.getIndices(),
        #                                                                                 other.getNeighbors(),))
        if not self.isLegal(other, shared):
            edges = self.swapEdge0(other, shared, _triangleList)


    def legalizeEdge1(self, _triangleList):
        """
        Maximizes the minimum angle by swapping the edge, if need be.
        Returns the other triangle if a swap occurred, None if not.
        """
        # see legalizeEdge0 for comments
        try:
            other = _triangleList[self._neighbor1]
        except TypeError:
            if self._neighbor1 is None:
                return None
            else:
                raise
        shared = self.getSharedFeatures(other)
        # global notify
        # notify.warning("legalizeEdge1 self {0}:{1} n:{2}".format(self.index,
        #                                                          self.getIndices(),
        #                                                          self.getNeighbors()))
        if not self.isLegal(other, shared):
            edges = self.swapEdge1(other, shared, _triangleList)

    def legalizeEdge2(self, _triangleList):
        """
        Maximizes the minimum angle by swapping the edge, if need be.
        Returns the other triangle if a swap occurred, None if not.
        """
        # see legalizeEdge0 for comments
        try:
            other = _triangleList[self._neighbor2]
        except TypeError:
            if self._neighbor2 is None:
                return None
            else:
                raise
        shared = self.getSharedFeatures(other)
        # global notify
        # notify.warning("legalizeEdge2 self {0}:{1} n:{2}".format(self.index,
        #                                                          self.getIndices(),
        #                                                          self.getNeighbors()))
        if not self.isLegal(other, shared):
            edges = self.swapEdge2(other, shared, _triangleList)

    def _getDummiesAndAngles(self, sharedFeatures):
        assert sharedFeatures.numSharedPoints == 2
        if sharedFeatures.edge0:
            sharedEdgeIndices = self.getEdgeIndices0()
        elif sharedFeatures.edge1:
            sharedEdgeIndices = self.getEdgeIndices1()
        elif sharedFeatures.edge2:
            sharedEdgeIndices = self.getEdgeIndices2()
        ghostInds1 = Triangle.getCcwOrder(sharedEdgeIndices[0],
                                          sharedFeatures.otherIndicesNotShared[0],
                                          sharedFeatures.indicesNotShared[0],
                                          self._rewriter)
        ghostInds2 = Triangle.getCcwOrder(sharedFeatures.otherIndicesNotShared[0],
                                          sharedEdgeIndices[1],
                                          sharedFeatures.indicesNotShared[0],
                                          self._rewriter)

        ghostTriMin1 = Triangle.getDummyMinAngleDeg(ghostInds1[0], ghostInds1[1], ghostInds1[2],
                                                    self._rewriter)
        ghostTriMin2 = Triangle.getDummyMinAngleDeg(ghostInds2[0], ghostInds2[1], ghostInds2[2],
                                                    self._rewriter)
        return ghostInds1, ghostInds2, ghostTriMin1, ghostTriMin2

    def isLegal(self, other, sharedFeatures=None):
        if sharedFeatures is None:
            sharedFeatures = self.getSharedFeatures(other)
        currentMinAng = min(self.getMinAngleDeg(), other.getMinAngleDeg())
        ghostInds1, ghostInds2, ghostTriMin1, ghostTriMin2 = self._getDummiesAndAngles(sharedFeatures)
        lowestOther = min(ghostTriMin1, ghostTriMin2)
        return lowestOther < currentMinAng

    def swapEdge0(self, other, sharedFeatures, _triangleList, saved=None):
        # global notify
        # notify.warning("in swapEdge0\nself {0}:{1} n:{2} | other  {3}:{4} n:{5}".format(self.index,
        #                                                                                 self.getIndices(),
        #                                                                                 self.getNeighbors(),
        #                                                                                 other.index,
        #                                                                                 other.getIndices(),
        #                                                                                 other.getNeighbors(),))
        otherShared = other.getSharedFeatures(self)
        self.pointIndex1 = sharedFeatures.otherIndicesNotShared[0]
        if saved is not None:
            if self._neighbor0 is not None:
                assert _triangleList[self._neighbor0].setNewNeighbor(self) != 0
            self._neighbor0 = saved
            self._neighbor1 = other.index
        else:
            lost = self._neighbor1
            if otherShared.edge0:
                other.swapEdge0(self, otherShared, _triangleList, saved=lost)
            elif otherShared.edge1:
                other.swapEdge1(self, otherShared, _triangleList, saved=lost)
            elif otherShared.edge2:
                other.swapEdge2(self, otherShared, _triangleList, saved=lost)
            self._neighbor0 = other.getNeighborOnEdge(self.pointIndex0, sharedFeatures.otherIndicesNotShared[0])
            if self._neighbor0 is not None:
                assert _triangleList[self._neighbor0].setNewNeighbor(self) != 0
            self._neighbor1 = other.index

    def swapEdge1(self, other, sharedFeatures, _triangleList, saved=None):
        # global notify
        # notify.warning("in swapEdge1\nself {0}:{1} n:{2} | other  {3}:{4} n:{5}".format(self.index,
        #                                                                                 self.getIndices(),
        #                                                                                 self.getNeighbors(),
        #                                                                                 other.index,
        #                                                                                 other.getIndices(),
        #                                                                                 other.getNeighbors(),))
        otherShared = other.getSharedFeatures(self)
        self.pointIndex2 = sharedFeatures.otherIndicesNotShared[0]
        if saved is not None:
            if self._neighbor1 is not None:
                assert _triangleList[self._neighbor1].setNewNeighbor(self) != 0
            self._neighbor1 = saved
            self._neighbor2 = other.index
        else:
            lost = self._neighbor2
            if otherShared.edge0:
                other.swapEdge0(self, otherShared, _triangleList, saved=lost)
            elif otherShared.edge1:
                other.swapEdge1(self, otherShared, _triangleList, saved=lost)
            elif otherShared.edge2:
                other.swapEdge2(self, otherShared, _triangleList, saved=lost)
            self._neighbor1 = other.getNeighborOnEdge(self.pointIndex0, sharedFeatures.otherIndicesNotShared[0])
            if self._neighbor1 is not None:
                assert _triangleList[self._neighbor1].setNewNeighbor(self) != 0
            self._neighbor2 = other.index

    def swapEdge2(self, other, sharedFeatures, _triangleList, saved=None):
        # global notify
        # notify.warning("in swapEdge2\nself {0}:{1} n:{2} | other  {3}:{4} n:{5}".format(self.index,
        #                                                                                 self.getIndices(),
        #                                                                                 self.getNeighbors(),
        #                                                                                 other.index,
        #                                                                                 other.getIndices(),
        #                                                                                 other.getNeighbors(),))
        otherShared = other.getSharedFeatures(self)
        self.pointIndex2 = sharedFeatures.otherIndicesNotShared[0]
        if saved is not None:
            if self._neighbor2 is not None:
                assert _triangleList[self._neighbor2].setNewNeighbor(self) != 0
            self._neighbor2 = saved
            self._neighbor1 = other.index
        else:
            lost = self._neighbor1
            if otherShared.edge0:
                other.swapEdge0(self, otherShared, _triangleList, saved=lost)
            elif otherShared.edge1:
                other.swapEdge1(self, otherShared, _triangleList, saved=lost)
            elif otherShared.edge2:
                other.swapEdge2(self, otherShared, _triangleList, saved=lost)
            self._neighbor1 = other.index
            self._neighbor2 = other.getNeighborOnEdge(self.pointIndex0, sharedFeatures.otherIndicesNotShared[0])
            if self._neighbor2 is not None:
                assert _triangleList[self._neighbor2].setNewNeighbor(self) != 0

    def getEdgeWithPoints(self, ind1, ind2):
        edge0 = self.getEdgeIndices0()
        edge1 = self.getEdgeIndices1()
        edge2 = self.getEdgeIndices2()
        if ind1 in edge0 and ind2 in edge0:
            return 0
        if ind1 in edge1 and ind2 in edge1:
            return 1
        if ind1 in edge2 and ind2 in edge2:
            return 2
        raise LookupError(
            "Both points must be in this triangle ind1: {0} ind2: {1} \n\tedges:0: {2} 1: {3} 2: {4}".format(ind1,
                                                                                                             ind2,
                                                                                                             edge0,
                                                                                                             edge1,
                                                                                                             edge2)
        )

    def getNeighborOnEdge(self, ind1, ind2):
        edge = self.getEdgeWithPoints(ind1, ind2)
        if edge == 0:
            return self._neighbor0
        if edge == 1:
            return self._neighbor1
        if edge == 2:
            return self._neighbor2

    # ##################### NEW For Make Delaunay  ABOVE ##########################

    def getNeighbors(self, includeEmpties=True):
        if includeEmpties:
            return self._neighbor0, self._neighbor1, self._neighbor2
        else:
            return filter(None, (self._neighbor0, self._neighbor1, self._neighbor2))

    # useful if I use only short lived triangles (a few that expire after triangulate() with list of tuples only)
    # def resetFromTuple(self, tup, vdata, geomTriangles, rewriter):
    #     self._selfIndex = tup.index
    #     self._neighbor0 = tup.neighbor0
    #     self._neighbor1 = tup.neighbor1
    #     self._neighbor2 = tup.neighbor2
    #
    #     self._primitiveInterface.vdata = vdata
    #     self._primitiveInterface.primitives = geomTriangles
    #     self._rewriter = rewriter

    def setNewNeighbor(self, newNeighbor):
        shared = self.getSharedFeatures(newNeighbor)
        numSet = 0
        if shared.edge0:
            self._neighbor0 = newNeighbor.index
            numSet += 1
        if shared.edge1:
            self._neighbor1 = newNeighbor.index
            numSet += 1
        if shared.edge2:
            self._neighbor2 = newNeighbor.index
            numSet += 1
        return numSet

    def reverse(self):
        super(ConstrainedDelaunayAdjacencyTriangle, self).reverse()
        tmp = self._neighbor0
        self._neighbor0 = self._neighbor2
        self._neighbor2 = tmp

    def _split(self, pointIndex, triangle1Position, triangle2Position):
        """
        Where i is the pointIndex position, this only supports:
        Tri 1 | Tri 2
        0,1,i | i,1,2
        0,1,i | 0,i,2
        0,i,2 | i,1,2
        """
        # BLOG herding cats: Use defaults with exceptions and other tricks to keep things on the expected track--test assumptions
        # BLOG the biggest Pain in the Ass Function (for my user) that I've ever written
        if triangle2Position == 0:
            newListTriangle = ConstrainedDelaunayAdjacencyTriangle(pointIndex, self.pointIndex1, self.pointIndex2,
                                                                   self._primitiveInterface.vdata,
                                                                   self._primitiveInterface.primitives,
                                                                   self._rewriter)
        elif triangle2Position == 1:
            newListTriangle = ConstrainedDelaunayAdjacencyTriangle(self.pointIndex0, pointIndex, self.pointIndex2,
                                                                   self._primitiveInterface.vdata,
                                                                   self._primitiveInterface.primitives,
                                                                   self._rewriter)
        else:
            raise IndexError("Position index must be 0 or 1")

        if triangle1Position == 1:
            assert triangle2Position == 0
            self.pointIndex1 = pointIndex
        elif triangle1Position == 2:
            assert triangle2Position == 0 or triangle2Position == 1
            self.pointIndex2 = pointIndex
        else:
            raise IndexError("Position index must be 1 or 2")

        return newListTriangle

    def triangulatePoint(self, pointIndex, _triangleList):
        """
        Triangulates a point that lays within this triangle, either on its interior or its edge.
        Then it returns the new triangles.
        """
        self._rewriter.setRow(pointIndex)
        point = self._rewriter.getData3f()
        slf = self.asPointsEnum()
        newTriangles = []
        # global notify
        # notify.warning("CALLING either selfI: {0} indices {1} neighbors {2}".format(self.index,
        #                                                                             self.getIndices(),
        #                                                                             self.getNeighbors()))
        if self.containsPoint(point, includeEdges=False):
            newTriangles.extend(self._triangulateSelf(pointIndex, _triangleList))
        else:
            # if the point is on the edge
            assert self.getOccupiedEdge(point, slf=slf)
            newTriangle, onEdge = self._triangulateSelfEdge(pointIndex, point, slf, _triangleList)
            if onEdge == '2' and self._neighbor2 is not None:  # triangulate the neighbor on the edge incident to the point
                newTriangles.extend(_triangleList[self._neighbor2]._triangulateOtherEdge(pointIndex,
                                                                                         point, onEdge,
                                                                                         self, newTriangle,
                                                                                         _triangleList))
            elif onEdge == '1' and self._neighbor1 is not None:
                newTriangles.extend(_triangleList[self._neighbor1]._triangulateOtherEdge(pointIndex,
                                                                                         point, onEdge,
                                                                                         self, newTriangle,
                                                                                         _triangleList))
            elif onEdge == '0' and self._neighbor0 is not None:
                newTriangles.extend(_triangleList[self._neighbor0]._triangulateOtherEdge(pointIndex,
                                                                                         point, onEdge,
                                                                                         self, newTriangle,
                                                                                         _triangleList))
            else:
                newTriangles.extend((newTriangle,))  # the edge was none
        return newTriangles

    def _triangulateSelf(self, pointIndex, _triangleList):
        """Triangulate the triangle when the dividing point occurs in the interior of this triangle."""
        # the new point always takes the original triangle's point1
        pInd2 = self.pointIndex2
        pInd1 = self.pointIndex1
        pInd0 = self.pointIndex0

        self.pointIndex1 = pointIndex
        # create the new triangles
        newTriangle1 = ConstrainedDelaunayAdjacencyTriangle(pInd0, pInd1, pointIndex,
                                                            self._primitiveInterface.vdata,
                                                            self._primitiveInterface.primitives,
                                                            self._rewriter)
        newTriangle2 = ConstrainedDelaunayAdjacencyTriangle(pointIndex, pInd1, pInd2,
                                                            self._primitiveInterface.vdata,
                                                            self._primitiveInterface.primitives,
                                                            self._rewriter)
        # fix the neighbor assignments
        # newTriangle1 is always on edge0. And, newTriangle2 is always on  edge1.
        # Any original neighbors on those edges need notified of the new triangle
        if self._neighbor0 is not None:
            _triangleList[self._neighbor0].setNewNeighbor(newTriangle1)
        if self._neighbor1 is not None:
            _triangleList[self._neighbor1].setNewNeighbor(newTriangle2)
        # the rest of the assignments are between the original and the two new triangles
        newTriangle1._neighbor2 = self.index
        newTriangle1._neighbor1 = newTriangle2.index
        newTriangle1._neighbor0 = self._neighbor0
        self._neighbor0 = newTriangle1.index

        newTriangle2._neighbor0 = newTriangle1.index
        newTriangle2._neighbor1 = self._neighbor1
        self._neighbor1 = newTriangle2.index
        newTriangle2._neighbor2 = self.index
        # global notify
        # notify.warning("_triangulateSelf\nself:", self, "\nnew1", newTriangle1, "\nnew2", newTriangle2)
        # return the new triangles in numerical order according to their triangle index
        if newTriangle1 < newTriangle2:
            return newTriangle1, newTriangle2
        else:
            return newTriangle2, newTriangle1

    def _triangulateSelfEdge(self, pointIndex, point, slf, _triangleList):
        """Triangulate the triangle when the dividing point lies in the boundary."""
        # global selfEdgeCallCount
        # selfEdgeCallCount += 1
        onEdge = self.getOccupiedEdge(point, slf)
        if onEdge == '2':
            newTriangle = self._split(pointIndex, 2, 0)
            newTriangle._neighbor1 = self._neighbor1
            if self._neighbor1 is not None:
                _triangleList[self._neighbor1].setNewNeighbor(newTriangle)
            self._neighbor1 = newTriangle.index
            newTriangle._neighbor0 = self.index
        elif onEdge == '1':
            newTriangle = self._split(pointIndex, 2, 1)
            newTriangle._neighbor2 = self._neighbor2
            if self._neighbor1 is not None:
                _triangleList[self._neighbor2].setNewNeighbor(newTriangle)
            self._neighbor2 = newTriangle.index
            newTriangle._neighbor0 = self.index
        elif onEdge == '0':
            newTriangle = self._split(pointIndex, 1, 0)
            newTriangle._neighbor1 = self._neighbor1
            if self._neighbor1 is not None:
                _triangleList[self._neighbor1].setNewNeighbor(newTriangle)
            self._neighbor1 = newTriangle.index
            newTriangle._neighbor2 = self.index
        elif len(onEdge) == 0:
            raise ValueError("Triangulation of point that is not on this triangle's edge: " +
                             str(point) + " triangle: " + self.__str__())
        elif len(onEdge) > 1:
            raise ValueError("Triangulation of point that's already a triangulated point: " +
                             str(point) + " triangle: " + self.__str__())
        else:
            raise ValueError("Unkown Error with point on edge point:" + str(point) + " edge: " + onEdge + self.__str__())
        # global notify
        # notify.warning("in _triangulateSelfEdge onEdge: {0}\n\tself: {1}\n\tnew T: {2}\ncallCount{3}".format(onEdge,
        #                                                                                                   self,
        #                                                                                                   newTriangle,
        #                                                                                                   selfEdgeCallCount))
        return newTriangle, onEdge

    def _triangulateOtherEdge(self, pointIndex, point,
                              originatorsEdge, originator, originatorsNewTriangle,
                              _triangleList):
        """Triangulate self when another triangle is initiating the triangulation"""
        # global notify, otherEdgeCallCount
        # otherEdgeCallCount += 1
        # notify.warning("in _triangulateOtherEdge() count {0}".format(otherEdgeCallCount))
        thisNewTriangle, onEdge = self._triangulateSelfEdge(pointIndex, point, self.asPointsEnum())
        edgeWithOriginator = self.getSharedFeatures(originator)
        # BLOG beautiful: only need to test for a shared edge on 1 of 4 possible relationships
        if edgeWithOriginator:
            # Thus, the two new triangles share and edge. Plus, both, new and old, always need set on the same side
            if onEdge == '0':
                self._neighbor0 = originator.index
                thisNewTriangle._neighbor0 = originatorsNewTriangle.index
            elif onEdge == '1':
                self._neighbor1 = originator.index
                thisNewTriangle._neighbor1 = originatorsNewTriangle.index
            elif onEdge == '2':
                self._neighbor2 = originator.index
                thisNewTriangle._neighbor2 = originatorsNewTriangle.index

            if originatorsEdge == '2':
                originator._neighbor2 = self.index
                originatorsNewTriangle._neighbor2 = thisNewTriangle.index
            elif originatorsEdge == '1':
                originator._neighbor1 = self.index
                originatorsNewTriangle._neighbor1 = thisNewTriangle.index
            elif originatorsEdge == '0':
                originator._neighbor0 = self.index
                originatorsNewTriangle._neighbor0 = thisNewTriangle.index
        else:  # it must share an edge with the other new triangle (just invert the assignments)
            if onEdge == '0':
                self._neighbor0 = originatorsNewTriangle.index
                thisNewTriangle._neighbor0 = originator.index
            elif onEdge == '1':
                self._neighbor1 = originatorsNewTriangle.index
                thisNewTriangle._neighbor1 = originator.index
            elif onEdge == '2':
                self._neighbor2 = originatorsNewTriangle.index
                thisNewTriangle._neighbor2 = originator.index

            if originatorsEdge == '2':
                originator._neighbor2 = thisNewTriangle.index
                originatorsNewTriangle._neighbor2 = self.index
            elif originatorsEdge == '1':
                originator._neighbor1 = thisNewTriangle.index
                originatorsNewTriangle._neighbor1 = self.index
            elif originatorsEdge == '0':
                originator._neighbor0 = thisNewTriangle.index
                originatorsNewTriangle._neighbor0 = self.index
        # global notify
        # notify.warning(
        #     "_triangulateOtherEdge onEdge: {0} originatorsEdge: {1}\n\tself: {2}\n\tnew T: {3}\n\t\toriginator {4}".format(
        #         onEdge, originatorsEdge, self, thisNewTriangle, originator
        #     )
        # )
        if originatorsNewTriangle < thisNewTriangle:
            return originatorsNewTriangle, thisNewTriangle
        else:
            return thisNewTriangle, originatorsNewTriangle

    def __str__(self):
        st = super(ConstrainedDelaunayAdjacencyTriangle, self).__str__()
        return st + ' neighbors: {0}, {1}, {2}'.format(self._neighbor0, self._neighbor1, self._neighbor2)


class ConstrainedDelaunayAdjacencyHoleTriangle(ConstrainedDelaunayAdjacencyTriangle):
    __slots__ = ()

    def __init__(self, vindex0, vindex1, vindex2, vertexData, geomTriangles, rewriter):
        super(ConstrainedDelaunayAdjacencyHoleTriangle, self).__init__(vindex0, vindex1, vindex2,
                                                                       vertexData, geomTriangles, rewriter)


class ConstrainedDelaunayTriangulator(object):
    """Creates a Constrained Delaunay Triangulation"""
    def __init__(self, vertexName='ConstrainedDelaunayTriangles', vertexFormat=GeomVertexFormat.getV3(),
                  usage = Geom.UHDynamic, onVertexCreationCallback = None, universalZ = 0.0):
        self._vertexData = GeomVertexData(vertexName, vertexFormat, Geom.UHDynamic)
        self._geomTriangles = GeomTriangles(usage)
        self._geomTrianglesHoles = GeomTriangles(usage)
        self._vertexRewriter = GeomVertexRewriter(self._vertexData, 'vertex')  # user cannot have control of a writer

        if onVertexCreationCallback is None:
            onVertexCreationCallback = lambda x, y, z: None  # something to call without checking existence later
        self._vertexCallback = onVertexCreationCallback

        self._universalZ = universalZ
        self.__holes = [[]]
        self.__polygon = []
        self.bounds = {
            'minX': 50000.0,
            'maxX': -50000.0,
            'minY': 50000.0,
            'maxY': -50000.0,
        }
        self.lastStaticVertexIndex = -1

    def addHoleVertex(self, index):
        """Adds the next consecutive vertex of the current hole."""
        # we might not have holes but we should always have triangles after triangulation
        self.__holes[-1].append(index)

    def addPolygonVertex(self, index):
        """Adds the next consecutive vertex of the polygon."""
        self.__polygon.append(index)

    def _addVertex(self, x, y, z):
        # BLOG track bounds to create the encapsulating triangle rather than lexicographic ordering
        # BLOG could have used a heap while adding verts, then popped as we processed each vertex
        if x < self.bounds['minX']:
            self.bounds['minX'] = x
        if x > self.bounds['maxX']:
            self.bounds['maxX'] = x

        if y < self.bounds['minY']:
            self.bounds['minY'] = y
        if y > self.bounds['maxY']:
            self.bounds['maxY'] = y

        if not self._vertexRewriter.isAtEnd():
            self._vertexRewriter.setWriteRow(self._vertexData.getNumRows())
        n = self._vertexRewriter.getWriteRow()
        self._vertexRewriter.addData3f(x, y, self._universalZ)
        self._vertexCallback(x, y, self._universalZ)
        return n

    def addVertex(self, pointOrX, y=None, z=None):
        """Adds a new vertex to the vertex pool."""
        if hasattr(pointOrX, 'y'):  # if the first argument is a point expand and call the backing function
            return self._addVertex(*pointOrX)
        return self._addVertex(pointOrX, y, z)

    def addVertexToPolygon(self, pointOrX, y, z):
        """Adds a vertex to the pool and then adds its index to the polygon vertex index list."""
        n = self.addVertex(pointOrX, y, z)
        self.addPolygonVertex(n)
        return n

    def addVertexToHole(self, pointOrX, y, z):
        """Adds a vertex to the pool and then adds its index to the polygon vertex index list."""
        n = self.addVertex(pointOrX, y, z)
        self.addHoleVertex(n)
        return n
    
    def beginHole(self):
        """Finishes the previous hole, if any, and prepares to add a new hole."""
        if self.__holes[-1]:  # if the last hole (list of vertices) is empty use it as the next hole
            self.__holes.append([])  # if it's not empty append a new hole
    
    def clear(self):
        """Removes all vertices and polygon specifications from the Triangulator, and prepares it to start over."""
        raise NotImplementedError("""ConstrainedDelaunayTriangulator.clear() is not implemented.""")
    
    def clearPolygon(self):
        """Removes the current polygon definition (and its set of holes), but does not clear the vertex pool."""
        raise NotImplementedError("""ConstrainedDelaunayTriangulator.clearPolygon() is not implemented.""")

    def findContainingTriangle(self, point, startTriangle, fullList):
        triangles = collections.deque([])
        triangles.append(startTriangle)
        while triangles:
            tri = triangles.popleft()
            if tri.containsPoint(point):
                return tri
            triangles.extend([fullList[n] for n in tri.getNeighbors(includeEmpties=False)])
        raise ValueError("Point added that's outside of the bounded space {0}".format(point))

    def getGeomNode(self, name='ConstrainedDelaunayTriangles'):
        """returns a GeomNode, with the provided name, sufficient to put in the scene and draw."""
        # BLOG My TODO legend
        # DOC 1) (adding to scene) create a Geom and add primitives of like base-type i.e. triangles and triangle strips
        geom = Geom(self._vertexData)
        geom.addPrimitive(self._geomTriangles)
        # DOC 2) create a GeomNode to hold the Geom(s) and add the Geom(s)
        gn = GeomNode(name)
        gn.addGeom(geom)
        # DOC 3) attach the node to the scene (in the calling code)
        # gnNodePath = render.attachNewNode(gn)
        return gn
    
    def getNumTriangles(self):
        """Returns the number of triangles generated by the previous call to triangulate()."""
        if not self.isTriangulated():
            raise ValueError("Vertices must be added and triangulate() must be called before calling getNumTriangles()")
        return len(self.__polygon)
    
    def getNumVertices(self):
        """Returns the number of vertices in the pool."""
        return self._vertexData.getNumRows()

    def getVertexReader(self):
        """Returns a reader for the vertex column."""
        return GeomVertexReader(self._vertexData, 'vertex')
    
    def getTriangleV0(self, n):
        """Returns vertex 0 of the nth triangle generated by the previous call to triangulate()."""
        try:
            self.__polygon[n].pointIndex0()
        except AttributeError:  # BLOG switching errors to clarify the cause
            raise LookupError("Must call triangulate() before querying for a triangle's vertices.")
    
    def getTriangleV1(self, n):
        """Returns vertex 1 of the nth triangle generated by the previous call to triangulate()."""
        try:
            self.__polygon[n].pointIndex1()
        except AttributeError:  # BLOG switching errors to clarify the cause
            raise LookupError("Must call triangulate() before querying for a triangle's vertices.")
    
    def getTriangleV2(self, n):
        """Returns vertex 2 of the nth triangle generated by the previous call to triangulate()."""
        try:
            self.__polygon[n].pointIndex2()
        except AttributeError:  # BLOG switching errors to clarify the cause
            raise LookupError("Must call triangulate() before querying for a triangle's vertices.")
    
    def getVertex(self, n):
        """Returns the nth vertex."""
        self._vertexRewriter.setRow(n)
        return self._vertexRewriter.getData3f()

    def getGeomVertexData(self):
        return self._vertexData

    def getTriangleList(self):
        assert self.isTriangulated()
        return self.__polygon
    
    def getVertices(self):
        """Returns a list of vertices."""
        verts = []
        for i in range(0, self._vertexData.getNumRows()):
            self._vertexRewriter.setRow(i)
            verts.append(self._vertexRewriter.getData3f())
        return verts
    
    def isLeftWinding(self):
        """Returns true if the polygon vertices are listed in counterclockwise order,
        or false if they appear to be listed in clockwise order."""
        assert self.isTriangulated()
        return self.__polygon[0].isLeftWinding()

    def isTriangulated(self):
        """Guesses whether the polygon has been triangulated."""
        return len(self.__polygon) > 0 and isinstance(self.__polygon[0], ConstrainedDelaunayAdjacencyTriangle)
    
    def triangulate(self, makeDelaunay=True):
        """Does the work of triangulating the specified polygon."""
        if self.isTriangulated():
            raise ValueError("triangulate() must only be called once.")
        h = self.bounds['maxY'] - self.bounds['minY']
        w = self.bounds['maxX'] - self.bounds['minX']
        global notify
        topLeft = Point3(self.bounds['minX'],         # far left x
                         self.bounds['maxY'] + abs(h/2),   # creates a triangle twice as tall as the square
                         self._universalZ)
        bottomLeft = Point3(self.bounds['minX'],
                            self.bounds['minY'] - abs(h/2),
                            self._universalZ)
        farRight = getIntersectionBetweenPoints(topLeft,  # line 0 point 0
                                                Point3(self.bounds['maxX'], self.bounds['maxY'], self._universalZ),
                                                bottomLeft,  # line 1 point 0
                                                Point3(self.bounds['maxX'], self.bounds['minY'], self._universalZ))
        self.lastStaticVertexIndex = self.getNumVertices() - 1  # any vertices added after create CDT as opposed to DT
        v0 = self.addVertex(topLeft)
        v1 = self.addVertex(bottomLeft)
        v2 = self.addVertex(farRight)

        bounds = ConstrainedDelaunayAdjacencyTriangle(v0, v1, v2, self._vertexData, self._geomTriangles, self._vertexRewriter)
        # for i in range(0, self._vertexData.getNumRows()):
        #     self._vertexRewriter.setRow(i)
        #     notify.warning("point {0}: {1}".format(i, self._vertexRewriter.getData3f()))
        # notify.warning("bounds:\n{0}".format(bounds))

        triangulated = [bounds]
        while True:
            try:
                pt = self.__polygon.pop()
            except IndexError:
                break
            self._vertexRewriter.setRow(pt)
            point = self._vertexRewriter.getData3f()
            # find the triangle the point lays within
            found = self.findContainingTriangle(point, bounds, triangulated)
            if found is not None:
                # triangulate the point into the triangle, collecting any new triangles
                oldEnd = len(triangulated) - 1
                newTriangles = found.triangulatePoint(pt, triangulated)
                # BLOG heapq.merge() is useless as it returns an iterable which can't be index, but heaps require lists
                # BLOG Hence, it's probably faster to heap.push than to iterate (in C) a merg then iterate to recreate a list
                # BLOG if I use a heap
                triangulated.extend(newTriangles)
                if makeDelaunay:
                    for tri in newTriangles:
                        tri.legalizeEdge0(triangulated)
                        tri.legalizeEdge1(triangulated)
                        tri.legalizeEdge2(triangulated)
                global notify
                notify.warning("\n")
                for i in triangulated:
                    notify.warning("i: {0} indices: {1} neighbors: {2}".format(i.index, i.getIndices(), i.getNeighbors()))
            else:
                raise ValueError("Point given that's outside of original space.")
        self.__polygon = triangulated

