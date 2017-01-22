#!/usr/bin/python
from direct.directnotify.DirectNotify import DirectNotify

from collections import namedtuple
from computationalgeom.triangle import Triangle
from computationalgeom.utils import isPointInWedge

notify = DirectNotify().newCategory("constrainedDelaunayAdjacencyTriangle")


class ConstrainedDelaunayAdjacencyTriangle(Triangle):
    __slots__ = ('_neighbor0', '_neighbor1', '_neighbor2', )

    @classmethod
    def setAllNeighbors(cls, neighborTriangles):
        neighborTriangles = list(set(neighborTriangles))
        for tri in neighborTriangles:
            if tri._neighbor0 in neighborTriangles:
                tri._neighbor0 = None
            if tri._neighbor1 in neighborTriangles:
                tri._neighbor1 = None
            if tri._neighbor2 in neighborTriangles:
                tri._neighbor2 = None
        ni = -1
        for tri1 in neighborTriangles:
            ni += 1
            for tri2 in neighborTriangles[ni:]:
                if tri2 == tri1.index:
                    continue
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
                assert numNeighborsSet < 2  # a triangle cannot neighbor more than one side
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
    def legalize(self, newPoint, _triangleList):
        # BLOG optimal throw (most of the time there will be a neighbor, hence try rather than if)
        nei = None
        if newPoint not in self.edge0:
            nei = self._neighbor0
        elif newPoint not in self.edge1:
            nei = self._neighbor1
        elif newPoint not in self.edge2:
            nei = self._neighbor2

        try:
            other = _triangleList[nei]
        except TypeError:
            if nei is None:
                other = None
            else:
                # http://nedbatchelder.com/blog/200711/rethrowing_exceptions_in_python.html (confirmed to raise original)
                raise

        if other is not None:
            # get edges not being swapped in other triangle as they will need legalized afterwards
            sharedParts = self.getSharedFeatures(other)
            notify.warning("self.edgeIndices0 {} sharedParts.indicesNotShared {}".format(self.edgeIndices0, sharedParts.indicesNotShared))
            if sharedParts.edge0:
                followUpEdges = [(self.edgeIndices0[0], sharedParts.indicesNotShared[0]),
                                 (self.edgeIndices0[1], sharedParts.indicesNotShared[0])]
            elif sharedParts.edge1:
                followUpEdges = [(self.edgeIndices1[0], sharedParts.indicesNotShared[0]),
                                 (self.edgeIndices1[1], sharedParts.indicesNotShared[0])]
            elif sharedParts.edge2:
                followUpEdges = [(self.edgeIndices2[0], sharedParts.indicesNotShared[0]),
                                 (self.edgeIndices2[1], sharedParts.indicesNotShared[0])]

            self.legalizeEdge(newPoint, other, _triangleList)

    def legalizeEdge(self, newPoint, other, _triangleList):
        """
        Maximizes the minimum angle by swapping the edge, if need be.
        Returns the other triangle if a swap occurred, None if not.
        """
        trianglesSwapped = []
        # get the shared edge. So, we can make 'ghost' triangles, and save on redundant calculations
        sharedFeatures = self.getSharedFeatures(other)
        otherShared = other.getSharedFeatures(self)
        global notify
        # Record the edges that we'll need to legalize afterwards.
        if not self.isLegal(other, sharedFeatures):
            if otherShared.edge0:  # will throw an undefined var err further down if this block doesn't resolve
                unsharedEdge1 = other.edgeIndices1
                unsharedEdge2 = other.edgeIndices2
            elif otherShared.edge1:
                unsharedEdge1 = other.edgeIndices0
                unsharedEdge2 = other.edgeIndices2
            elif otherShared.edge2:
                unsharedEdge1 = other.edgeIndices0
                unsharedEdge2 = other.edgeIndices1
            # save neighbors so we can reset their relationships after edge swaps
            trianglesSwapped.extend(self.getNeighbors(includeEmpties=False))
            trianglesSwapped.extend(other.getNeighbors(includeEmpties=False))
            trianglesSwapped.extend((self.index, other.index))
            notify.warning("legalizeEdge self {0}:{1} n:{2}".format(self.index,
                                                                     self.getPointIndices(),
                                                                     self.getNeighbors()))
            notify.warning("legalizeEdge other {0}:{1} n:{2}".format(other.index,
                                                                      other.getPointIndices(),
                                                                      other.getNeighbors()))
            # swap self. Set other to its new edge.
            if sharedFeatures.edge0:
                self.pointIndex1 = sharedFeatures.otherIndicesNotShared[0]
            elif sharedFeatures.edge1:
                self.pointIndex2 = sharedFeatures.otherIndicesNotShared[0]
            elif sharedFeatures.edge2:
                self.pointIndex0 = sharedFeatures.otherIndicesNotShared[0]
            else:
                raise ValueError("No shared edge between {0} and {1}".format(self.index, other.index))
            if otherShared.edge0:
                other.pointIndex1 = sharedFeatures.indicesNotShared[0]
            elif otherShared.edge1:
                other.pointIndex2 = sharedFeatures.indicesNotShared[0]
            elif otherShared.edge2:
                other.pointIndex0 = sharedFeatures.indicesNotShared[0]
            else:
                raise ValueError("No shared edge between {0} and {1}".format(self.index, other.index))

            trianglesSwapped = [_triangleList[tri] if isinstance(tri, int) else tri for tri in trianglesSwapped]
            ConstrainedDelaunayAdjacencyTriangle.setAllNeighbors(trianglesSwapped)
            foundEdges = 0
            # find the new edge in self that used to be an unshared edge
            nextOther = None
            if (unsharedEdge1[0] in self.edgeIndices0 and unsharedEdge1[1] in self.edgeIndices0 or
                            unsharedEdge2[0] in self.edgeIndices0 and unsharedEdge2[1] in self.edgeIndices0):
                foundEdges += 1
                if self._neighbor0 is not None:
                    nextOther = _triangleList[self._neighbor0]
            elif (unsharedEdge1[0] in self.edgeIndices1 and unsharedEdge1[1] in self.edgeIndices1 or
                              unsharedEdge2[0] in self.edgeIndices1 and unsharedEdge2[1] in self.edgeIndices1):
                foundEdges += 1
                if self._neighbor1 is not None:
                    nextOther = _triangleList[self._neighbor1]
            elif (unsharedEdge1[0] in self.edgeIndices2 and unsharedEdge1[1] in self.edgeIndices2 or
                              unsharedEdge2[0] in self.edgeIndices2 and unsharedEdge2[1] in self.edgeIndices2):
                foundEdges += 1
                if self._neighbor2 is not None:
                    nextOther = _triangleList[self._neighbor2]

            if foundEdges != 1:
                notify.warning("foundEdges not equalling 1. unsharedE1: {0} unsharedE2 {1} self edge0 {2} edge1 {3} edge1 {4}".format(
                    unsharedEdge1, unsharedEdge2, self.edge0, self.edge1, self.edge2,
                ))
            if nextOther is not None:
                self.legalizeEdge(newPoint, nextOther, _triangleList)

            nextOther = None
            if (unsharedEdge1[0] in other.edgeIndices0 and unsharedEdge1[1] in other.edgeIndices0 or
                            unsharedEdge2[0] in other.edgeIndices0 and unsharedEdge2[1] in other.edgeIndices0):
                foundEdges += 1
                if other._neighbor0 is not None:
                    nextOther = _triangleList[other._neighbor0]
            elif (unsharedEdge1[0] in other.edgeIndices1 and unsharedEdge1[1] in other.edgeIndices1 or
                              unsharedEdge2[0] in other.edgeIndices1 and unsharedEdge2[1] in other.edgeIndices1):
                foundEdges += 1
                if other._neighbor1 is not None:
                    nextOther = _triangleList[other._neighbor1]
            elif (unsharedEdge1[0] in other.edgeIndices2 and unsharedEdge1[1] in other.edgeIndices2 or
                              unsharedEdge2[0] in other.edgeIndices2 and unsharedEdge2[1] in other.edgeIndices2):
                foundEdges += 1
                if other._neighbor2 is not None:
                    nextOther = _triangleList[other._neighbor2]

            # assert foundEdges == 2
            if nextOther is not None:
                other.legalizeEdge(newPoint, nextOther, _triangleList)

        else:
            notify.warning("legalizeEdge isLegal => no swap")


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

    def triangulatePoint(self, pointIndex, _triangleList):
        """
        Triangulates a point that lays within this triangle, either on its interior or its edge.
        Then it returns the new triangles.
        """
        self._rewriter.setRow(pointIndex)
        point = self._rewriter.getData3f()
        slf = self.asPointsEnum()
        newTriangles = []
        global notify
        notify.warning("Triangulate Point\n\tpoint: {}\n\tself: {}".format(point, self))
        oldTriangles = [self, ]
        if self.containsPoint(point, includeEdges=False):
            notify.warning("containsPoint self:\n\t{0}".format(self))
            newTriangle, _ = self._triangulateSelf(pointIndex, _triangleList)
            if newTriangle is None:
                return newTriangles
            notify.warning("containsPoint newTriangle:\n\t{0}".format(newTriangle))
            newTriangles.append(newTriangle)
        else:
            # if the point is on the edge
            newTriangle, onEdge = self._triangulateOnEdge(pointIndex, point, slf)
            if newTriangle is None:
                return []
            notify.warning("_triangulateOnEdge() neighbors: {0}".format(self.getNeighbors()))
            if onEdge == '0' and self._neighbor0 is not None:  # triangulate the neighbor on the edge incident to the point
                other = _triangleList[self._neighbor0]
                oldTriangles.append(other)
                newTriangles.append(newTriangle)
                newTriangle2, _ = other._triangulateOnEdge(pointIndex, point, self)
                if newTriangle2 is not None:
                    newTriangles.append(newTriangle2)
                notify.warning("'0' _triangulateOtherEdge()")
            elif onEdge == '1' and self._neighbor1 is not None:
                other = _triangleList[self._neighbor1]
                oldTriangles.append(other)
                newTriangles.append(newTriangle)
                newTriangle2, _ = other._triangulateOnEdge(pointIndex, point, self)
                if newTriangle2 is not None:
                    newTriangles.append(newTriangle2)
                notify.warning("'1' _triangulateOtherEdge()")
            elif onEdge == '2' and self._neighbor2 is not None:
                other = _triangleList[self._neighbor2]
                oldTriangles.append(other)
                newTriangles.append(newTriangle)
                newTriangle2, _ = other._triangulateOnEdge(pointIndex, point, self)
                if newTriangle2 is not None:
                    newTriangles.append(newTriangle2)
                notify.warning("'2' _triangulateOtherEdge()")
            else:
                notify.warning("No change. The edge was None. newTriangle {}".format(newTriangle))

                newTriangles.append(newTriangle)  # the edge was none
            # get the old triangle neighbors
            oldies = oldTriangles[:]
            for tri in oldTriangles:
                naybs = list(tri.getNeighbors(includeEmpties=False))
                for i in range(0, len(naybs)):
                    try:
                        naybs[i] = _triangleList[naybs[i]]
                    except IndexError:
                        assert naybs[i] in newTriangles
                        naybs[i] = None

                naybs = filter(lambda n: n is not None, naybs)
                oldies.extend(naybs)

            ConstrainedDelaunayAdjacencyTriangle.setAllNeighbors(newTriangles + oldies)
        return newTriangles

    def _triangulateSelf(self, pointIndex, _triangleList):
        """Triangulate the triangle when the dividing point occurs in the interior of this triangle."""
        if pointIndex in (self.pointIndex0, self.pointIndex1, self.pointIndex2):
            return []
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
        listToFix = [newTriangle1, newTriangle2]
        listToFix.append(self)
        naybs = list(self.getNeighbors(includeEmpties=False))
        for n in range(0, len(naybs)):
            naybs[n] = _triangleList[naybs[n]]
        listToFix.extend(naybs)
        ConstrainedDelaunayAdjacencyTriangle.setAllNeighbors(listToFix)
        return [newTriangle1, newTriangle2]

    def _triangulateOnEdge(self, pointIndex, point, slf):
        """Triangulate the triangle when the dividing point lies in the boundary."""
        if isinstance(slf, ConstrainedDelaunayAdjacencyTriangle):
            slf = self.asPointsEnum()
        onEdge = self.getOccupiedEdge(point, slf)
        originalInds = self.getPointIndices()
        if pointIndex in originalInds:
            return None, onEdge
        notify.warning("in _triangulateOnEdge()\n\tonEdge {}".format(onEdge))
        if onEdge == '0':
            reformedTrianglePointsI = (originalInds[0], pointIndex, originalInds[2])
            newTrianglePointsI = (originalInds[1], originalInds[2], pointIndex)
        elif onEdge == '1':
            reformedTrianglePointsI = (originalInds[0], originalInds[1], pointIndex)
            newTrianglePointsI = (originalInds[2], originalInds[0], pointIndex)
        elif onEdge == '2':
            reformedTrianglePointsI = (originalInds[0], originalInds[1], pointIndex)
            newTrianglePointsI = (originalInds[1], originalInds[2], pointIndex)
        elif len(onEdge) == 0:
            raise ValueError("Triangulation of point that is not on this triangle's edge: " +
                             str(point) + " triangle: " + self.__str__())
        elif len(onEdge) > 1:
            return None, onEdge
            # raise ValueError("Triangulation of point that's already a triangulated point: " +
            #                  str(point) + " triangle: " + self.__str__())
        else:
            raise ValueError("Unknown Error with point on edge point:" + str(point) + " edge: " + onEdge + self.__str__())

        self.setPointIndices(*reformedTrianglePointsI)
        newTriangle = ConstrainedDelaunayAdjacencyTriangle(newTrianglePointsI[0], newTrianglePointsI[1], newTrianglePointsI[2],
                                                           self._primitiveInterface.vdata, self._primitiveInterface.primitives,
                                                           self._rewriter)
        return newTriangle, onEdge

    def __str__(self):
        st = super(ConstrainedDelaunayAdjacencyTriangle, self).__str__()
        return st + '\n\tn: {0}, {1}, {2}'.format(self._neighbor0, self._neighbor1, self._neighbor2)

    def __eq__(self, other):
        if isinstance(other, ConstrainedDelaunayAdjacencyTriangle):
            other = other.index
        return self.index == other

    def __lt__(self, other):
        if isinstance(other, ConstrainedDelaunayAdjacencyTriangle):
            other = other.index
        return self.index < other

    def __gt__(self, other):
        return not self.__eq__(other) and not self.__lt__(other)

    def __le__(self, other):
        return self.__lt__(other) or self.__eq__(other)

    def __ge__(self, other):
        return self.__gt__(other) or self.__eq__(other)

    def __cmp__(self, other):
        if self.__eq__(other):
            return 0
        elif self.__lt__(other):
            return -1

class ConstrainedDelaunayAdjacencyHoleTriangle(ConstrainedDelaunayAdjacencyTriangle):
    __slots__ = ()

    def __init__(self, vindex0, vindex1, vindex2, vertexData, geomTriangles, rewriter):
        super(ConstrainedDelaunayAdjacencyHoleTriangle, self).__init__(vindex0, vindex1, vindex2,
                                                                       vertexData, geomTriangles, rewriter)