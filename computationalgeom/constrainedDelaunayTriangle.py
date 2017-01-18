#!/usr/bin/python
from direct.directnotify.DirectNotify import DirectNotify

from collections import namedtuple
from computationalgeom.triangle import Triangle
from computationalgeom.utils import isPointInWedge

notify = DirectNotify().newCategory("constrainedDelaunayAdjacencyTriangle")


class ConstrainedDelaunayAdjacencyTriangle(Triangle):
    __slots__ = ('_neighbor0', '_neighbor1', '_neighbor2', )

    @classmethod
    def setAllNeighbors(cls, neighborLst, _triangleList):
        neighborLst = list(set(neighborLst))
        for k in neighborLst:
            tri = _triangleList[k]
            if tri._neighbor0 in neighborLst:
                tri._neighbor0 = None
            if tri._neighbor1 in neighborLst:
                tri._neighbor1 = None
            if tri._neighbor2 in neighborLst:
                tri._neighbor2 = None
        for n in range(0, len(neighborLst)):
            for m in range(n, len(neighborLst)):
                if m == n:
                    continue
                tri1 = _triangleList[n]
                tri2 = _triangleList[m]
                notify.warning("setAllNeighbors")
                notify.warning("{0}".format(tri1))
                notify.warning("{0}".format(tri2))
                shared1 = tri1.getSharedFeatures(tri2)
                shared2 = tri2.getSharedFeatures(tri1)
                numNeighborsSet = 0
                if shared1.edge0 is True:
                    tri1._neighbor0 = tri2.index
                    numNeighborsSet += 1
                if shared1.edge1 is True:
                    tri1._neighbor1 = tri2.index
                    numNeighborsSet += 1
                if shared1.edge2 is True:
                    tri1._neighbor2 = tri2.index
                    numNeighborsSet += 1
                assert numNeighborsSet < 3  # a triangle cannot neighbor more than one side
                numNeighborsSet = 0
                if shared2.edge0 is True:
                    tri2._neighbor0 = tri1.index
                    numNeighborsSet += 1
                if shared2.edge1 is True:
                    tri2._neighbor1 = tri1.index
                    numNeighborsSet += 1
                if shared2.edge2 is True:
                    tri2._neighbor2 = tri1.index
                    numNeighborsSet += 1
                assert numNeighborsSet < 2  # a triangle cannot neighbor more than one side


    class AdjacencyTuple(namedtuple("AdjacencyTuple", 'index neighbor0 neighbor1 neighbor2')):
        __slots__ = ()

    def __init__(self, vindex0, vindex1, vindex2, vertexData, geomTriangles, rewriter, adjTuple=None):
        super(ConstrainedDelaunayAdjacencyTriangle, self).__init__(vindex0, vindex1, vindex2,
                                                                   vertexData, geomTriangles, rewriter)
        if adjTuple is not None:
            self._neighbor0 = adjTuple.neighbor0
            self._neighbor1 = adjTuple.neighbor1
            self._neighbor2 = adjTuple.neighbor2
        else:
            self._neighbor0 = None
            self._neighbor1 = None
            self._neighbor2 = None

    # ##################### NEW For Make Delaunay ##########################
    def legalize(self, _triangleList):
        swappedList = self.legalizeEdge0(_triangleList)
        if len(swappedList) > 0:
            # edge 1 is the new edge. Edge 0 needs checked again as it's a different edge.
            ConstrainedDelaunayAdjacencyTriangle.setAllNeighbors(swappedList, _triangleList)
            swappedList = self.legalizeEdge0(_triangleList)
            ConstrainedDelaunayAdjacencyTriangle.setAllNeighbors(swappedList, _triangleList)
            swappedList = self.legalizeEdge2(_triangleList)
            ConstrainedDelaunayAdjacencyTriangle.setAllNeighbors(swappedList, _triangleList)
            return


        swappedList = self.legalizeEdge1(_triangleList)
        if len(swappedList) > 0:
            # edge 2 is the new edge. Edge 1 needs checked again as it's a different edge.
            ConstrainedDelaunayAdjacencyTriangle.setAllNeighbors(swappedList, _triangleList)
            swappedList = self.legalizeEdge1(_triangleList)
            ConstrainedDelaunayAdjacencyTriangle.setAllNeighbors(swappedList, _triangleList)
            swappedList = self.legalizeEdge0(_triangleList)
            ConstrainedDelaunayAdjacencyTriangle.setAllNeighbors(swappedList, _triangleList)
            return

        swappedList = self.legalizeEdge2(_triangleList)
        if len(swappedList) > 0:
            # edge 0 is the new edge. Edge 2 needs checked again as it's a different edge.
            ConstrainedDelaunayAdjacencyTriangle.setAllNeighbors(swappedList, _triangleList)
            swappedList = self.legalizeEdge2(_triangleList)
            ConstrainedDelaunayAdjacencyTriangle.setAllNeighbors(swappedList, _triangleList)
            swappedList = self.legalizeEdge1(_triangleList)
            ConstrainedDelaunayAdjacencyTriangle.setAllNeighbors(swappedList, _triangleList)
            return

    def legalizeEdge0(self, _triangleList):
        """
        Maximizes the minimum angle by swapping the edge, if need be.
        Returns the other triangle if a swap occurred, None if not.
        """
        trianglesSwapped = []
        # BLOG optimal throw (most of the time there will be a neighbor, hence try rather than if)
        try:
            other = _triangleList[self._neighbor0]
        except TypeError:
            if self._neighbor0 is None:
                return trianglesSwapped
            else:
                # http://nedbatchelder.com/blog/200711/rethrowing_exceptions_in_python.html (confirmed to raise original)
                raise
        # get the shared edge. So, we can make 'ghost' triangles, and save on redundant calculations
        sharedFeatures = self.getSharedFeatures(other)
        global notify
        # notify.warning("\nlegalizeEdge0\nshared {0}\n".format(sharedFeatures))
        if not self.isLegal(other, sharedFeatures):
            trianglesSwapped.extend(self.getNeighbors(includeEmpties=False))
            trianglesSwapped.extend(other.getNeighbors(includeEmpties=False))
            trianglesSwapped.extend((self.index, other.index))
            notify.warning("legalizeEdge0 self {0}:{1} n:{2}".format(self.index,
                                                                     self.getPointIndices(),
                                                                     self.getNeighbors()))
            notify.warning("legalizeEdge0 other {0}:{1} n:{2}".format(other.index,
                                                                      other.getPointIndices(),
                                                                      other.getNeighbors()))
            self.swap(other, _triangleList)
            return trianglesSwapped
        else:
            notify.warning("legalizeEdge0 isLegal => no swap")
            return trianglesSwapped


    def legalizeEdge1(self, _triangleList):
        """
        Maximizes the minimum angle by swapping the edge, if need be.
        Returns the other triangle if a swap occurred, None if not.
        """
        trianglesSwapped = []
        # see legalizeEdge0 for comments
        try:
            other = _triangleList[self._neighbor1]
        except TypeError:
            if self._neighbor1 is None:
                return trianglesSwapped
            else:
                raise
        sharedFeatures = self.getSharedFeatures(other)
        global notify
        # notify.warning("\nlegalizeEdge1\nshared {0}\n".format(sharedFeatures))
        if not self.isLegal(other, sharedFeatures):
            trianglesSwapped.extend(self.getNeighbors(includeEmpties=False))
            trianglesSwapped.extend(other.getNeighbors(includeEmpties=False))
            trianglesSwapped.extend((self.index, other.index))
            notify.warning("legalizeEdge1\nself {0}:{1} n:{2}".format(self.index,
                                                                     self.getPointIndices(),
                                                                     self.getNeighbors()))
            notify.warning("legalizeEdge1\nother {0}:{1} n:{2}".format(other.index,
                                                                      other.getPointIndices(),
                                                                      other.getNeighbors()))
            self.swap(other, _triangleList)
            return trianglesSwapped
        else:
            notify.warning("legalizeEdge1 isLegal => no swap")
            return trianglesSwapped

    def legalizeEdge2(self, _triangleList):
        """
        Maximizes the minimum angle by swapping the edge, if need be.
        Returns the other triangle if a swap occurred, None if not.
        """
        trianglesSwapped = []
        # see legalizeEdge0 for comments
        try:
            other = _triangleList[self._neighbor2]
        except TypeError:
            if self._neighbor2 is None:
                return trianglesSwapped
            else:
                raise
        sharedFeatures = self.getSharedFeatures(other)
        global notify
        if not self.isLegal(other, sharedFeatures):
            trianglesSwapped.extend(self.getNeighbors(includeEmpties=False))
            trianglesSwapped.extend(other.getNeighbors(includeEmpties=False))
            trianglesSwapped.extend((self.index, other.index))
            notify.warning("legalizeEdge2\nself {0}:{1} n:{2}".format(self.index,
                                                                     self.getPointIndices(),
                                                                     self.getNeighbors()))
            notify.warning("legalizeEdge2\nother {0}:{1} n:{2}".format(other.index,
                                                                      other.getPointIndices(),
                                                                      other.getNeighbors()))
            self.swap(other, _triangleList)
            return trianglesSwapped
        else:
            notify.warning("legalizeEdge2 isLegal => no swap")
            return trianglesSwapped

    def swap(self, other, _triangleList):
        # we only want to deal with references to these two triangles
        theseTwo = (self.index, other.index)
        sharedFeatures = self.getSharedFeatures(other)
        otherShared = other.getSharedFeatures(self)
        # swap self. Set other to its new edge. Record the edges that we'll need to legalize afterwards.
        if sharedFeatures.edge0:
            self.swap0(sharedFeatures.otherIndicesNotShared[0], other.index)
        elif sharedFeatures.edge1:
            self.swap1(sharedFeatures.otherIndicesNotShared[0], other.index)
        elif sharedFeatures.edge2:
            self.swap2(sharedFeatures.otherIndicesNotShared[0], other.index)
        else:
            raise ValueError("No shared edge between {0} and {1}".format(self.index, other.index))
        # do the same wrt the other triangle
        if otherShared.edge0:
            other.swap0(sharedFeatures.indicesNotShared[0], self.index)
        elif otherShared.edge1:
            other.swap1(sharedFeatures.indicesNotShared[0], self.index)
        elif otherShared.edge2:
            other.swap2(sharedFeatures.indicesNotShared[0], self.index)
        else:
            raise ValueError("No shared edge between {0} and {1}".format(self.index, other.index))

    def swap0(self, pointInd, otherInd):
        self.pointIndex1 = pointInd

    def swap1(self, pointInd, otherInd):
        self.pointIndex2 = pointInd

    def swap2(self, pointInd, otherInd):
        self.pointIndex0 = pointInd

    def _getDummiesAndAngles(self, sharedFeatures):
        if sharedFeatures.numSharedPoints != 2:
            raise ValueError("Cannot create dummies shared\n{0}\n{1}\nand\n\t{2}".format(sharedFeatures,
                                                                                         self,
                                                                                         sharedFeatures.other))
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
        self._rewriter.setRow(sharedFeatures.otherIndicesNotShared[0])
        point = self._rewriter.getData3f()
        circle = self.getCircumcircleSquared()
        # we can't swap the edges unless the new edge cuts across the shared edge
        if sharedFeatures.edge0 and not self.isPointVisibleOverEdge0(point, inclusive=False):
            return True  # True here means the alternative is illegal
        elif sharedFeatures.edge1 and not self.isPointVisibleOverEdge1(point, inclusive=False):
            return True  # True here means the alternative is illegal
        elif sharedFeatures.edge2 and not self.isPointVisibleOverEdge2(point, inclusive=False):
            return True  # True here means the alternative is illegal

        currentMinAng = min(self.getMinAngleDeg(), other.getMinAngleDeg())
        ghostInds1, ghostInds2, ghostTriMin1, ghostTriMin2 = self._getDummiesAndAngles(sharedFeatures)
        dummyMinAng = min(ghostTriMin1, ghostTriMin2)
        # First 2 conditions for when the fake triangles are degenerate (collinear). Keep the current arrangement
        return ghostTriMin1 <= 0 or ghostTriMin1 <= 0 or dummyMinAng <= currentMinAng

    def isPointVisibleOverEdge0(self, point, inclusive=False):
        return isPointInWedge(point, self.edge1[::-1], self.edge2, inclusive=inclusive)

    def isPointVisibleOverEdge1(self, point, inclusive=False):
        return isPointInWedge(point, self.edge0, self.edge2[::-1], inclusive=inclusive)

    def isPointVisibleOverEdge2(self, point, inclusive=False):
        return isPointInWedge(point, self.edge0[::-1], self.edge1, inclusive=inclusive)

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
        self._rewriter.setRow(shared.otherIndicesNotShared[0])
        opposingPoint = self._rewriter.getData3f()

        if shared and shared.edge0 and self.isPointVisibleOverEdge0(opposingPoint):
            self._neighbor0 = newNeighbor.index
            assert self._neighbor0 or 1 == self.getNeighbors(includeEmpties=False).count(self._neighbor0)
            numSet += 1
        if shared and shared.edge1 and self.isPointVisibleOverEdge1(opposingPoint):
            self._neighbor1 = newNeighbor.index
            assert self._neighbor1 or 1 == self.getNeighbors(includeEmpties=False).count(self._neighbor1)
            numSet += 1
        if shared and shared.edge2 and self.isPointVisibleOverEdge2(opposingPoint):
            self._neighbor2 = newNeighbor.index
            assert self._neighbor2 or 1 == self.getNeighbors(includeEmpties=False).count(self._neighbor2)
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
        if self.containsPoint(point, includeEdges=False):
            newTriangles.extend(self._triangulateSelf(pointIndex, _triangleList))
        else:
            # if the point is on the edge
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
        return newTriangle, onEdge

    def _triangulateOtherEdge(self, pointIndex, point,
                              originatorsEdge, originator, originatorsNewTriangle,
                              _triangleList):
        """Triangulate self when another triangle is initiating the triangulation"""
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
        if originatorsNewTriangle < thisNewTriangle:
            return originatorsNewTriangle, thisNewTriangle
        else:
            return thisNewTriangle, originatorsNewTriangle

    def __str__(self):
        st = super(ConstrainedDelaunayAdjacencyTriangle, self).__str__()
        return st + '\n\tn: {0}, {1}, {2}'.format(self._neighbor0, self._neighbor1, self._neighbor2)



class ConstrainedDelaunayAdjacencyHoleTriangle(ConstrainedDelaunayAdjacencyTriangle):
    __slots__ = ()

    def __init__(self, vindex0, vindex1, vindex2, vertexData, geomTriangles, rewriter):
        super(ConstrainedDelaunayAdjacencyHoleTriangle, self).__init__(vindex0, vindex1, vindex2,
                                                                       vertexData, geomTriangles, rewriter)