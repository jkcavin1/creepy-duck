#!/usr/bin/python
from panda3d.core import (GeomTriangles, Geom,
                          InternalName)  # https://www.panda3d.org/reference/1.9.1/python/panda3d.core.GeomTriangles.php


class ComputationalTriangle(GeomTriangles):
    def __init__(self, usageHint=Geom.UHDynamic):
        try:
            super(ComputationalTriangle, self).__init__(usageHint)
        except TypeError:
            GeomTriangles.__init__(self, usageHint)
