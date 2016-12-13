#!/usr/bin/python
import collections

from direct.directnotify.DirectNotify import DirectNotify

from panda3d.core import Geom, GeomNode
from panda3d.core import GeomVertexData, GeomVertexFormat, GeomTriangles, GeomVertexReader, GeomVertexRewriter
from panda3d.core import Point3
from computationalgeom.constrainedDelaunayTriangle import ConstrainedDelaunayAdjacencyTriangle, ConstrainedDelaunayAdjacencyHoleTriangle
from utils import getIntersectionBetweenPoints, getCenterOfPoints3D
from utilities.maxHeap import MaxHeap


notify = DirectNotify().newCategory("DelaunayTriangulator")

class _DummyPoint:
    def __init__(self, ind, vertRewriter):
        self.ind = ind
        vertRewriter.setRow(ind)
        self.pt = vertRewriter.getData3f()
        self.rewritter = vertRewriter

    def __lt__(self, other):
        pass


class ConstrainedDelaunayTriangulator(object):
    """Creates a Constrained Delaunay Triangulation"""
    @staticmethod
    def drawTriangleIndices(adjLst):
        from direct.gui.OnscreenText import OnscreenText
        indNP = render.attachNewNode('indsgroup')
        for i in range(0, len(adjLst)):
            center = getCenterOfPoints3D(adjLst[i].tri)
            dummy = indNP.attachNewNode(str(i))
            txt = OnscreenText(text=str(i), pos=center, scale=1)
            txt.reparentTo(dummy)
            dummy.setP(dummy.getP() - 90)

        return indNP

    @staticmethod
    def findContainingTriangle(point, startTriangle, fullList):
        triangles = collections.deque([])
        triangles.append(startTriangle)
        while triangles:
            tri = triangles.popleft()
            if tri.containsPoint(point):
                return tri
            triangles.extend([fullList[n] for n in tri.getNeighbors(includeEmpties=False)])
        raise ValueError("Point added that's outside of the bounded space {0}".format(point))

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
        inf = float('inf')
        negInf = float('-inf')
        self.bounds = {
            'minX': inf,
            'maxX': negInf,
            'minY': inf,
            'maxY': negInf,
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

    def getAdjacencyList(self):
        """Returns a list of triangles, each referencing its own neighbors by their list index."""
        return self.__polygon

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

    def getTriangles(self):
        assert self.isTriangulated()
        return self._geomTriangles
    
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
        global notify
        if self.isTriangulated():
            raise ValueError("triangulate() must only be called once.")
        h = self.bounds['maxY'] - self.bounds['minY']
        w = self.bounds['maxX'] - self.bounds['minX']
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
        # Other than the tree below, any vertices added after this are CDT as opposed to DT
        self.lastStaticVertexIndex = self.getNumVertices() - 1
        # these three will be removed later, thus will not be artificial restraints
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
            # find the triangle the point lays within
            found = ConstrainedDelaunayTriangulator.findContainingTriangle(point, bounds, triangulated)
            if found is not None:
                # triangulate the point into the triangle, collecting any new triangles
                newTriangles = found.triangulatePoint(pt, triangulated)
                # BLOG heapq.merge() is useless as it returns an iterable which can't be index, but heaps require lists
                # BLOG Hence, it's probably faster to heap.push than to iterate (in C) a merge then iterate to recreate a list
                # BLOG if I use a heap
                triangulated.extend(newTriangles)
                if makeDelaunay:
                    for tri in newTriangles:
                        tri.legalize(triangulated)
            else:
                raise ValueError("Point given that's outside of original space.")

        self.__polygon = triangulated

