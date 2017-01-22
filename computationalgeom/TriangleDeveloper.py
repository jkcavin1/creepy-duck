#!/usr/bin/python


from direct.showbase.ShowBase import ShowBase
from pandac.PandaModules import WindowProperties

from panda3d.core import RenderModeAttrib, LineSegs
from panda3d.core import Vec3, Vec4, Point3
from panda3d.core import Geom, GeomNode, GeomVertexData, GeomTriangles, GeomVertexFormat, GeomVertexRewriter
from direct.directnotify.DirectNotify import DirectNotify

from utilities import pandaHelperFuncs as PHF
from utils import EPSILON
from constrainedDelaunayTriangulator import ConstrainedDelaunayTriangulator  # Unit test Triangle


notify = DirectNotify().newCategory("DelaunayTriangulatorUnitTest")
# selfEdgeCallCount = otherEdgeCallCount = 0


class Developer(ShowBase):
    def __init__(self):
        ShowBase.__init__(self)
        winProps = WindowProperties()
        winProps.setTitle("Triangle and SimpleCircle Unittest")
        # base.win.requestProperties(winProps) (same as below e.g. self == base)
        self.win.requestProperties(winProps)


        zUp = Vec3(0.0, 0.0, 1.0)
        wt = Vec4(1.0, 1.0, 1.0, 1.0)

        def addToVertex(x, y, z):
            normal.addData3f(zUp)
            color.addData4f(wt)
        # BLOG post about these bullet points then delete them
        # 1) create GeomVertexData (inside the triangulator's constructor)
        frmt = GeomVertexFormat.getV3n3cp()
        triangulator = ConstrainedDelaunayTriangulator(vertexFormat = frmt, onVertexCreationCallback = addToVertex)
        vdata = triangulator.getGeomVertexData()

        # 2) create Writers/Rewriters (must all be created before any readers and readers are one-pass-temporary)
        normal = GeomVertexRewriter(vdata, 'normal')  # DOC 'vertex' is the only prohibited column for the user to use
        color = GeomVertexRewriter(vdata, 'color')

        # 3) write each column on the vertex data object (for speed, have a different writer for each column)

        # DOC 1.DT) create triangulator
        # DOC 2.DT) add vertices (before calling triangulate)
        # ############# NOT ANGLE OPTIMAL BELOW #####################
        triangulator.addVertexToPolygon(5.0, 0.0, 0.0)
        triangulator.addVertexToPolygon(0.0, 0.0, 0.0)
        triangulator.addVertexToPolygon(1.5, 2.5, 0.0)
        triangulator.addVertexToPolygon(0.0, 5.0, 0.0)
        triangulator.addVertexToPolygon(5.0, 5.0, 0.0)
        # ############# NOT ANGLE OPTIMAL ABOVE #####################
        # triangulator.addVertexToPolygon(5.0, 0.0, 0.0)
        # # triangulator.addVertexToPolygon(6.5, 6.5, 0.0)
        # triangulator.addVertexToPolygon(1.5, 2.5, 0.0)
        # triangulator.addVertexToPolygon(0.0, 0.0, 0.0)
        # triangulator.addVertexToPolygon(0.0, 5.0, 0.0)
        # triangulator.addVertexToPolygon(5.0, 5.0, 0.0)

        # DOC 3.DT) add hole vertices (before calling triangulate)

        # DOC 4.DT) call triangulate
        triangulator.triangulate(makeDelaunay=True)
        assert triangulator.isTriangulated()
        adjLst = triangulator.getAdjacencyList()
        foundInvalidReference = foundMissingReference = False  # so I can explicitly state all index references are correct

        for t in adjLst:
            # check that references with no neighbor haven't missed an edge
            noneEdges = []
            if t._neighbor0 is None:
                noneEdges.append(t.edgeIndices0)
            elif t._neighbor1 is None:
                noneEdges.append(t.edgeIndices1)
            elif t._neighbor2 is None:
                noneEdges.append(t.edgeIndices2)

            for tri_ in adjLst:
                for edge_ in noneEdges:
                    # the edge that should hold the reference is on t. The one that should get referenced is on tri_
                    missedCount = 0
                    if edge_[0] in tri_.edgeIndices0 and edge_[1] in tri_.edgeIndices0 and tri_ != t:
                        missedCount += 1
                    if edge_[0] in tri_.edgeIndices1 and edge_[1] in tri_.edgeIndices1 and tri_ != t:
                        missedCount += 1
                    if edge_[0] in tri_.edgeIndices2 and edge_[1] in tri_.edgeIndices2 and tri_ != t:
                        missedCount += 1
                    if missedCount == 1:
                        foundMissingReference = True
                        notify.warning(
                            "!MISSED REFERENCE TO NEIGHBOR\nreferrer: {} ptIndices: {} neighbors: {}\n".format(
                                t.index, t.getPointIndices(), t.getNeighbors(),
                            ) + "missed: {} ptIndices: {} neighbors: {}".format(
                                tri_.index, tri_.getPointIndices(), tri_.getNeighbors(),
                        ))
                    elif missedCount > 1:
                        foundMissingReference = True
                        notify.warning(
                            "!EXTRANEOUS & MISSED SHARED EDGES\nreferrer: {} ptIndices: {} neighbors: {}\n".format(
                                t.index, t.getPointIndices(), t.getNeighbors(),
                            ) + "missed: {} ptIndices: {} neighbors: {}".format(
                                tri_.index, tri_.getPointIndices(), tri_.getNeighbors(),
                        ))

            # check that neighbor relations point to correct triangles
            for n in t.getNeighbors(includeEmpties=False):
                neighbor = adjLst[n]
                otherInds = neighbor.getPointIndices()
                if neighbor.index == t._neighbor0:
                    edge = t.edgeIndices0
                elif neighbor.index == t._neighbor1:
                    edge = t.edgeIndices1
                elif neighbor.index == t._neighbor2:
                    edge = t.edgeIndices2
                if edge[0] not in otherInds and edge[1] not in otherInds:
                    foundInvalidReference = True
                    notify.warning(
                        "!INVALID REFERENCE TO NEIGHBOR\nreferrer: {} indices: {} neighbors: {}".format(
                            t.index, t.getPointIndices(), t.getNeighbors(),
                        ) + "\nreferee: {} indices: {} neighbors: {}".format(
                            neighbor.index, neighbor.getPointIndices(), neighbor.getNeighbors()
                    ))

        if not foundMissingReference:
            notify.warning("No error missing reference in neighbors.")
        else:
            notify.warning("!!!ERROR missing reference in neighbor references.")

        if not foundInvalidReference:
            notify.warning("No error found in neighbors that were referenced.")
        else:
            notify.warning("!!!ERROR found in neighbors that were referenced.")

        foundPointInsideCircle = False
        for t in adjLst:
            circle_ = t.getCircumcircle()
            # cycle through triangles checking each point against each circle.center
            for e in adjLst:
                p0, p1, p2 = e.getPoints()

                if circle_.radius - (p0 - circle_.center).length() > EPSILON:
                    foundPointInsideCircle = True
                    notify.warning(
                        "!point in circumcircle point {0} circle {1}\ntriangle1: {2}\ntriangle2: {3}".format(
                            p0, circle_, t, e
                    ))

                if circle_.radius - (p1 - circle_.center).length() > EPSILON:
                    foundPointInsideCircle = True
                    notify.warning(
                        "!point in circumcircle point {0} circle {1}\ntriangle1: {2}\ntriangle2: {3}".format(
                            p1, circle_, t, e
                    ))

                if circle_.radius - (p2 - circle_.center).length() > EPSILON:
                    foundPointInsideCircle = True
                    notify.warning(
                        "!point in circumcircle point {0} circle {1}\ntriangle1: {2}\ntriangle2: {3}".format(
                            p2, circle_, t, e
                    ))

        if not foundPointInsideCircle:
            notify.warning("No point found inside circumcircle.")
        else:
            notify.warning("!!!ERROR found in neighbors that were referenced.")
        # TODO test edges that reference no neighbor
        # triangles = triangulator.getGeomTriangles()
        # print "Triangulated:"
        # for tri in triangleList:
        #     print "\t{0}".format(tri)

        # 4) create a primitive and add the vertices via index (not truly associated with the actual vertex table, yet)
        # tris = GeomTriangles(Geom.UHDynamic)
        # t1 = Triangle(0, 1, 2, vdata, tris, vertex)
        # t2 = Triangle(2, 1, 3, vdata, tris, vertex)
        # c1 = t1.getCircumcircle()
        # t1AsEnum = t1.asPointsEnum()
        # r0 = (t1AsEnum.point0 - c1.center).length()
        # r1 = (t1AsEnum.point1 - c1.center).length()
        # r2 = (t1AsEnum.point2 - c1.center).length()
        # assert abs(r0 - r2) < utilities.EPSILON and abs(r0 - r1) < utilities.EPSILON
        # t2AsEnum = t2.asPointsEnum()
        # c2 = t2.getCircumcircle()
        # r0 = (t2AsEnum.point0 - c2.center).length()
        # r1 = (t2AsEnum.point1 - c2.center).length()
        # r2 = (t2AsEnum.point2 - c2.center).length()
        # assert abs(r0 - r2) < utilities.EPSILON and abs(r0 - r1) < utilities.EPSILON

        # assert t1.getAngleDeg0() == 90.0
        # assert t1.getAngleDeg1() == t1.getAngleDeg2()
        #
        # oldInd0 = t1.pointIndex0
        # oldInd1 = t1.pointIndex1
        # oldInd2 = t1.pointIndex2
        # t1.pointIndex0 = t1.pointIndex1
        # t1.pointIndex1 = oldInd0
        # assert t1.pointIndex0 == oldInd1
        # assert t1.pointIndex1 == oldInd0
        # assert t1.pointIndex0 != t1.pointIndex1
        # t1.reverse()
        # assert t1.pointIndex1 == oldInd2
        #
        gn = triangulator.getGeomNode('triangles')
        gnNodePath = render.attachNewNode(gn)

        # setup a wire frame
        wireNP = render.attachNewNode('wire')
        wireNP.setPos(0.0, 0.0, .1)
        wireNP.setColor(0.1, 0.1, 0.1, 1)
        wireNP.setRenderMode(RenderModeAttrib.MWireframe, .5, 0)
        gnNodePath.instanceTo(wireNP)
        #
        # # test and draw intersections and circles
        # pt1 = Point3(0.0, 5.0, 0.0)
        # pt2 = Point3(1.0, 5.0, 0.0)
        # intersection = t2.getIntersectionsWithCircumcircle(pt1, pt2)
        # circle = t2.getCircumcircle()
        # cuts = 128
        # border = circle.getBorder(cuts, closed=True)
        # assert len(border) == cuts or (len(border) == cuts + 1 and border[0] == border[len(border) - 1])
        # n = len(border)
        # xMid = yMid = 0
        # for p in border:
        #     xMid += p.x
        #     yMid += p.y
        # mid = Point3(xMid / n, yMid / n, border[0].z)
        # assert mid.almostEqual(circle.center, 0.06)
        # assert t2.isLeftWinding()
        # assert t1.containsPoint(c1.center) != t1.containsPoint(c1.center, includeEdges=False)
        #
        # circleSegs = LineSegs("circleLines")
        # circleSegs.setColor(1.0, 0.0, 0.0, 1.0)
        # for p in border:
        #     circleSegs.drawTo(*p)
        # circleNode = circleSegs.create(False)
        # circleNP = render.attachNewNode(circleNode)
        # circleNP.setZ(-5)
        #
        # originSpot = LineSegs("intersection")
        # originSpot.setColor(1.0, 0.0, 0.0, 1.0)
        # originSpot.setThickness(10)
        # for p in intersection:
        #     originSpot.drawTo(p)
        # spotNode = originSpot.create(False)
        # spotNP = render.attachNewNode(spotNode)
        # circleNP.setZ(-0.75)

        # fix the camera rot/pos
        PHF.PanditorDisableMouseFunc()
        camera.setPos(0.0, 0.0, 50.0)
        camera.lookAt(Point3(0.0))  # 2.5, 2.5, 0.0))
        PHF.PanditorEnableMouseFunc()

        # print "isLeftWinding()", triangulator.isLeftWinding()
        # TODO port the triangle-indices node func drawTriangleIndices(...)
        indsNp = ConstrainedDelaunayTriangulator.drawTriangleIndices(triangulator.getTriangleList())
        indsNp.setPos(0.0, 0.0, 0.3)
        # render.ls()


if __name__ == '__main__':
    app = Developer()
    app.run()
