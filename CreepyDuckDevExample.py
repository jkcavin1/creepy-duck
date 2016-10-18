#!/usr/bin/python


from direct.showbase.ShowBase import ShowBase

# ### TEST IMPORTS BELOW ####
from panda3d.core import Geom, GeomNode, GeomVertexData, GeomTriangles, GeomPoints, GeomVertexFormat, GeomVertexRewriter
from panda3d.core import Vec3, Vec4, Point3, Point4
# ### TEST IMPORTS ABOVE ####

from utilities.pandaHelperFuncs import PanditorEnableMouseFunc, PanditorDisableMouseFunc

class DevTest(ShowBase):
    def __init__(self):
        ShowBase.__init__(self)

        PanditorDisableMouseFunc()
        camera.setPos(0.0, 0.0, 50.0)
        camera.lookAt(0.0)
        PanditorEnableMouseFunc()

        # 1) create GeomVertexData
        frmt = GeomVertexFormat.getV3n3cp()
        vdata = GeomVertexData('triangle', frmt, Geom.UHDynamic)

        # 2) create Writers/Rewriters (must all be created before any readers and readers are one-pass-temporary)
        vertex = GeomVertexRewriter(vdata, 'vertex')
        normal = GeomVertexRewriter(vdata, 'normal')
        color = GeomVertexRewriter(vdata, 'color')

        zUp = Vec3(0, 0, 1)
        wt = Vec4(1.0, 1.0, 1.0, 1.0)
        gr = Vec4(0.5, 0.5, 0.5, 1.0)

        # 3) write each column on the vertex data object (for speed, have a different writer for each column)
        def addPoint(x, y, z):
            vertex.addData3f(x, y, z)
            normal.addData3f(zUp)
            color.addData4f(wt)

        addPoint(0.0, 0.0, 0.0)
        addPoint(5.0, 0.0, 0.0)
        addPoint(0.0, 5.0, 0.0)
        addPoint(5.0, 5.0, 0.0)

        # 4) create a primitive and add the vertices via index (not truely associated with the actual vertex table, yet)
        tris = GeomTriangles(Geom.UHDynamic)
        tris.addVertices(0, 1, 2)
        tris.closePrimitive()
        tris.addVertices(2, 1, 3)
        print "vdataPoints", vdata.getArrays()[0]
        # 5.1) (adding to scene) create a Geom and add the primitive
        geom = Geom(vdata)
        geom.addPrimitive(tris)

        gn = GeomNode('gnode')
        gn.addGeom(geom)
        gnNodePath = render.attachNewNode(gn)

        geomPts = Geom(vdata)
        pts = GeomPoints(Geom.UHStatic)
        pts.addVertices(0, 1, 2)
        pts.closePrimitive()
        geomPts.addPrimitive(pts)
        pointsNode = GeomNode('points_node')
        pointsNode.addGeom(geomPts)
        pointsNP = render.attachNewNode(pointsNode)
        pointsNP.setZ(0.5)

        render.ls()


if __name__ == '__main__':
    print "Yep main"
    app = DevTest()
    app.run()
