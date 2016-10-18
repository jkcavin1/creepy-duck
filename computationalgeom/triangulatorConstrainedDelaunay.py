#!/usr/bin/python

from panda3d.core import Geom, GeomVertexData, GeomVertexFormat, GeomVertexRewriter


class TriangulatorCD(object):
    """Creates a Constrained Delaunay Triangulation"""
    def __init__(self, name='ConstrainedDelaunayTriangles', geomVertexDataObj=None,
                 format=GeomVertexData.getV3(), usage=Geom.UHDynamic,
                 onVertexCreationCallback=None, universalZ=0):

        if geomVertexDataObj is None:
            geomVertexDataObj = GeomVertexData(name, format, usage)
        self.vertexData = geomVertexDataObj
        self.vertexRewriter = GeomVertexRewriter(self.vertexData, 'vertex')

        if onVertexCreationCallback is None:
            onVertexCreationCallback = lambda vdata: None
        self._onVertexCreationCallback = onVertexCreationCallback

        self.universalZ = universalZ

    def addPolygonVertex(self):
