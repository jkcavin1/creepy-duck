#!/usr/bin/python
from triangulatorConstrainedDelaunay import TriangulatorConstrainedDelaunay, Triangle, SimpleCircle

class DelaunayAdjacencyTriangle(Triangle):
    __slots__ = ('_neighbor0', '_neighbor1', '_neighbor2')

    def __init__(self, selfIndex, vertexData, geomTriangles, rewriter, neighbor0=None, neighbor1=None, neighbor2=None):
        super(DelaunayAdjacencyTriangle, self).__init__(selfIndex, vertexData, geomTriangles, rewriter)
        self._neighbor0 = neighbor0
        self._neighbor1 = neighbor1
        self._neighbor2 = neighbor2


class AdjacencyListCDT(TriangulatorConstrainedDelaunay):
    """An Adjacency List made of a Constrained Delaunay Triangulation using Panda3D's vertex system"""
