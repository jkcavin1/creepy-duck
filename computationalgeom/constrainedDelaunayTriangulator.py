#!/usr/bin/python
import heapq

from panda3d.core import Geom, GeomNode
from panda3d.core import GeomVertexData, GeomVertexFormat, GeomTriangles, GeomVertexReader, GeomVertexRewriter
from panda3d.core import Thread
from panda3d.core import Point3

from triangle import Triangle
from utils import getIntersectionBetweenPoints


class ConstrainedDelaunayAdjacencyTriangle(Triangle):
    __slots__ = ('_neighbor0', '_neighbor1', '_neighbor2', )

    def __init__(self, vindex0, vindex1, vindex2, vertexData, geomTriangles, rewriter):
        super(ConstrainedDelaunayAdjacencyTriangle, self).__init__(vindex0, vindex1, vindex2,
                                                                   vertexData, geomTriangles, rewriter)
        self._neighbor0 = None
        self._neighbor1 = None
        self._neighbor2 = None

    def getNeighbors(self):
        return self._neighbor0, self._neighbor1, self._neighbor2

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

    def _setNeighbors0(self, newTriangle):
        newTriangle._neighbor1 = self._neighbor1
        self._neighbor1 = newTriangle.index
        newTriangle._neighbor2 = self.index

    def _setNeighbors1(self, newTriangle):
        newTriangle._neighbor2 = self._neighbor2
        self._neighbor2 = newTriangle.index
        newTriangle._neighbor0 = self.index

    def _setNeighbors2(self, newTriangle):
        newTriangle._neighbor1 = self._neighbor1
        self._neighbor1 = newTriangle.index
        newTriangle._neighbor0 = self.index

    def triangulatePoint(self, pointIndex, triangleList):
        self._rewriter.setRow(pointIndex)
        point = self._rewriter.getData3f()
        slf = self.asPointsEnum()

        if self.containsPoint(point, includeEdges=False):
            # if the point is on the interior
            return self._triangulateSelf(pointIndex)
        else:
            # if the point is on the edge
            newTriangle, onEdge = self._triangulateEdge(pointIndex, point, slf)
            if onEdge == '2' and self._neighbor2 is not None:  # triangulate the neighbor on the edge incident to the point
                return triangleList[self._neighbor2]._triangulateOtherEdge(pointIndex,
                                                                           point, onEdge,
                                                                           self, newTriangle)
            elif onEdge == '1' and self._neighbor1 is not None:
                return triangleList[self._neighbor1]._triangulateOtherEdge(pointIndex,
                                                                           point, onEdge,
                                                                           self, newTriangle)
            elif onEdge == '0' and self._neighbor0 is not None:
                return triangleList[self._neighbor0]._triangulateOtherEdge(pointIndex,
                                                                           point, onEdge,
                                                                           self, newTriangle)
            else:
                return newTriangle,

    def _triangulateSelf(self, pointIndex):
        """Triangulate the triangle when the dividing point occurs on the edge, resulting in this and the neighboring
        triangle getting split."""
        pInd2 = self.pointIndex2
        pInd1 = self.pointIndex1
        pInd0 = self.pointIndex0

        self.pointIndex1 = pointIndex
        newTriangle1 = ConstrainedDelaunayAdjacencyTriangle(pInd0, pInd1, pointIndex,
                                                            self._primitiveInterface.vdata,
                                                            self._primitiveInterface.primitives,
                                                            self._rewriter)
        newTriangle2 = ConstrainedDelaunayAdjacencyTriangle(pointIndex, pInd1, pInd2,
                                                            self._primitiveInterface.vdata,
                                                            self._primitiveInterface.primitives,
                                                            self._rewriter)
        newTriangle1._neighbor2 = self.index
        newTriangle1._neighbor1 = newTriangle2.index
        newTriangle1._neighbor0 = self._neighbor0
        self._neighbor0 = newTriangle1.index

        newTriangle2._neighbor0 = newTriangle1.index
        newTriangle2._neighbor1 = self._neighbor1
        self._neighbor1 = newTriangle2.index
        newTriangle2._neighbor2 = self.index

        if newTriangle1 < newTriangle2:
            return newTriangle1, newTriangle2
        else:
            return newTriangle2, newTriangle1

    def _triangulateEdge(self, pointIndex, point, slf):
        """Triangulate the triangle when the dividing point lies in the boundary."""
        onEdge = self.getOccupiedEdge(point, slf)
        if onEdge == '2':
            newTriangle = self._split(pointIndex, 2, 0)
            self._setNeighbors2(newTriangle)
        elif onEdge == '1':
            newTriangle = self._split(pointIndex, 2, 1)
            self._setNeighbors1(newTriangle)
        elif onEdge == '0':
            newTriangle = self._split(pointIndex, 1, 0)
            self._setNeighbors0(newTriangle)
        elif len(onEdge) == 0:
            raise ValueError("Triangulation of point that is not on this triangle's edge: " +
                             str(point) + " triangle: " + self.__str__())
        elif len(onEdge) > 1:
            raise ValueError("Triangulation of point that's already a triangulated point: " +
                             str(point) + " triangle: " + self.__str__())
        else:
            raise ValueError("Unkown Error with point on edge point:" + str(point) + " edge: " + onEdge + self.__str__())
        return newTriangle, onEdge

    def _triangulateOtherEdge(self, pointIndex, point, originatorsEdge, originator, originatorsNewTriangle):
        """Triangulate self when another triangle is initiating the triangulation"""
        thisNewTriangle, onEdge = self._triangulateEdge(pointIndex, point, self.asPointsEnum())
        edgeWithOriginator = self.sharedFeatures(originator)
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
        return st + ' neighbors: {0}, {1}, {2}'.format(self._neighbor0, self._neighbor1, self._neighbor2)

class ConstrainedDelaunayAdjacencyHoleTriangle(ConstrainedDelaunayAdjacencyTriangle):
    def __init__(self, vindex0, vindex1, vindex2, vertexData, geomTriangles, rewriter):
        super(ConstrainedDelaunayAdjacencyHoleTriangle, self).__init__(vindex0, vindex1, vindex2,
                                                                       vertexData, geomTriangles, rewriter)


class ConstrainedDelaunayTriangulator(object):
    """Creates a Constrained Delaunay Triangulation"""
    def __init__(self, name='ConstrainedDelaunayTriangles', geomVertexDataObj=None,
                 format=GeomVertexFormat.getV3(), usage=Geom.UHDynamic,
                 onVertexCreationCallback=None, universalZ=0.0):

        if geomVertexDataObj is None:
            geomVertexDataObj = GeomVertexData(name, format, usage)
        self._vertexData = geomVertexDataObj
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

    def findContainingTriangle(self, point, triangles, fullList):
        for triangle in filter(lambda x: x is not None, triangles):
            print triangle
            if not hasattr(triangle, 'containsPoint'):
                triangle = fullList[triangle]
            if triangle.containsPoint(point):
                return triangle
            else:
                return self.findContainingTriangle(point, triangle.getNeighbors(), fullList)
        return None

    def getGeomNode(self, name='ConstrainedDelaunayTriangles'):
        """returns a GeomNode sufficient put in the scene and draw."""
        # BLOG My TODO legend
        # DOC 1) (adding to scene) create a Geom and add primitives of like base-type i.e. triangles and triangle strips
        geom = Geom(self._vertexData)
        geom.addPrimitive(self._geomTriangles)
        # DOC 2) create a GeomNode to hold the Geom(s) and add the Geom(s)
        gn = GeomNode(name)
        gn.addGeom(geom)
        # DOC 3) attach the node to the scene
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
    
    def triangulate(self):
        """Does the work of triangulating the specified polygon."""
        h = self.bounds['maxY'] - self.bounds['minY']
        w = self.bounds['maxX'] - self.bounds['minX']
        topLeft = Point3(self.bounds['minX'],         # far left x
                         self.bounds['maxY'] + h/2,   # creates a triangle twice as tall as the square
                         self._universalZ)
        bottomLeft = Point3(self.bounds['minX'],
                            self.bounds['minY'] - h/2,
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
        triangulated = [bounds]
        while True:
            try:
                pt = self.__polygon.pop()
            except IndexError:
                break
            self._vertexRewriter.setRow(pt)
            point = self._vertexRewriter.getData3f()
            found = self.findContainingTriangle(point, (bounds,), triangulated)
            if found is not None:
                newTriangles = found.triangulatePoint(pt, triangulated)
            else:
                raise ValueError("Point given that's outside of original space.")
            for triangle in newTriangles:
                triangulated.append(triangle)

