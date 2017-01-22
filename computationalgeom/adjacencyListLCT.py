#!/usr/bin/python
from constrainedDelaunayTriangulator import ConstrainedDelaunayTriangulator, Triangle, SimpleCircle


class LocalClearanceAdjacencyTriangle(Triangle):
    __slots__ = ('_neighbor0', '_neighbor1', '_neighbor2')

    def __init__(self, vindex0, vindex1, vindex2, vertexData, geomTriangles, rewriter):
        super(LocalClearanceAdjacencyTriangle, self).__init__(vindex0, vindex1, vindex2, vertexData, geomTriangles, rewriter)


class LocalClearanceAdjacencyList(object):
    def __init__(self, vertexData, geomTriangles, rewriter):

# class AdjacencyListCDT(ConstrainedDelaunayTriangulator):
#     """An Adjacency List made of a Constrained Delaunay Triangulation using Panda3D's vertex system"""
