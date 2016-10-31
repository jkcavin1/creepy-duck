#!/usr/bin/python


from direct.showbase.ShowBase import ShowBase

from panda3d.core import RenderModeAttrib
from panda3d.core import Vec3, Vec4, Point3, Point4
from panda3d.core import Geom, GeomNode, GeomVertexData, GeomTriangles, GeomPoints, GeomVertexFormat, GeomVertexRewriter

from utilities.pandaHelperFuncs import PanditorEnableMouseFunc, PanditorDisableMouseFunc
from computationalgeom import utilities

from computationalgeom.triangulatorConstrainedDelaunay import Triangle, PrimitiveInterface  # Unit test Triangle

class Developer(ShowBase):
    def __init__(self):
        ShowBase.__init__(self)

        PanditorDisableMouseFunc()
        camera.setPos(0.0, 0.0, 50.0)
        camera.lookAt(0.0)
        PanditorEnableMouseFunc()


        # 1) create GeomVertexData
        frmt = GeomVertexFormat.getV3n3cp()
        vdata = GeomVertexData('triangle_developer', frmt, Geom.UHDynamic)

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
        addPoint(15.0, 15.0, 0.0)

        # 4) create a primitive and add the vertices via index (not truely associated with the actual vertex table, yet)
        tris = GeomTriangles(Geom.UHDynamic)
        t1 = Triangle(0, 1, 2, vdata, tris, vertex)
        t2 = Triangle(2, 1, 3, vdata, tris, vertex)
        print "t1", t1
        print "t2", t2
        c1 = t1.getCircumcircle()
        t1AsEnum = t1.asPointsEnum()
        # check that each line is on the circle's edge

        r0 = (t1AsEnum.point0 - c1.center).length()
        r1 = (t1AsEnum.point1 - c1.center).length()
        r2 = (t1AsEnum.point2 - c1.center).length()
        assert abs(r0 - r2) < utilities.EPSILON and abs(r0 - r1) < utilities.EPSILON
        t2AsEnum = t2.asPointsEnum()
        c2 = t2.getCircumcircle()
        r0 = (t2AsEnum.point0 - c2.center).length()
        r1 = (t2AsEnum.point1 - c2.center).length()
        r2 = (t2AsEnum.point2 - c2.center).length()
        assert abs(r0 - r2) < utilities.EPSILON and abs(r0 - r1) < utilities.EPSILON

        assert t1.getAngleDeg0() == 90.0
        assert t1.getAngleDeg1() == t1.getAngleDeg2()

        oldInd0 = t1.pointIndex0
        oldInd1 = t1.pointIndex1
        oldInd2 = t1.pointIndex2
        t1.pointIndex0 = t1.pointIndex1
        t1.pointIndex1 = oldInd0
        assert t1.pointIndex0 == oldInd1
        assert t1.pointIndex1 == oldInd0
        assert t1.pointIndex0 != t1.pointIndex1
        t1.reverse()
        assert t1.pointIndex1 == oldInd2
        print "t1", t1

        # 5.1) (adding to scene) create a Geom and add primitives of like base-type i.e. triangles and triangle strips
        geom = Geom(vdata)
        geom.addPrimitive(tris)
        # 5.2) create a GeomNode to hold the Geom(s) and add the Geom(s)
        gn = GeomNode('gnode')
        gn.addGeom(geom)
        # 5.3) attache the node to the scene
        gnNodePath = render.attachNewNode(gn)

        # setup a wire frame
        wireNP = render.attachNewNode('wire')
        wireNP.setPos(0.0, 0.0, .1)
        wireNP.setColor(0.1, 0.1, 0.1, 1)
        wireNP.setRenderMode(RenderModeAttrib.MWireframe, .5, 0)
        gnNodePath.instanceTo(wireNP)

        # TODO port the triangle-indices node func drawInds(...)
        #render.ls()


if __name__ == '__main__':
    print "Yep main"
    app = Developer()
    app.run()
