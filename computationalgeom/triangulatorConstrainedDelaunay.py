#!/usr/bin/python

from panda3d.core import Geom, GeomVertexData, GeomVertexFormat, GeomVertexRewriter

from simpleCircle import SimpleCircle  # for the circumcircle


class Triangle(object):
    __slots__ = ('_selfIndex', '_vertexData', '_geomTriangles', '_rewriter')
    # TODO may implement descriptor for attribute access:
    # https://docs.python.org/2/reference/datamodel.html#implementing-descriptors
    def __init__(self, selfIndex, vertexData, geomTriangles, rewriter):
        super(Triangle, self).__init__()
        self._selfIndex = selfIndex
        self._vertexData = vertexData
        self._geomTriangles = geomTriangles
        self._rewriter = rewriter

    @property
    def point0(self):
        pass


class TriangulatorConstrainedDelaunay(object):
    """Creates a Constrained Delaunay Triangulation"""
    def __init__(self, name='ConstrainedDelaunayTriangles', geomVertexDataObj=None,
                 format=GeomVertexData.getV3(), usage=Geom.UHDynamic,
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
