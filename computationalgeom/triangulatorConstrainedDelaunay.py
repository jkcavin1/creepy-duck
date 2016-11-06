#!/usr/bin/python

from panda3d.core import Geom, GeomVertexData, GeomVertexFormat, GeomVertexReader, GeomVertexRewriter
from panda3d.core import Thread
from panda3d.core import Point3

from triangle import Triangle


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
