__author__ = 'Lab Hatter'

# from panda3d.core import ConfigVariablString
# from panda3d.core import GeomVertexFormat, GeomVertexData, GeomLines
# from panda3d.core import Geom, GeomNode, GeomTriangles, GeomVertexWriter, ModelNode, NodePath
from direct.showbase.ShowBase import ShowBase

from panda3d.core import Point3
from panda3d.core import RenderModeAttrib, LineSegs
from utilities.pandaHelperFuncs import PanditorEnableMouseFunc, PanditorDisableMouseFunc
from PolygonUtils.AdjacencyList import AdjacencyList, makeTriMesh
from PolygonUtils.PolygonUtils import getCenterOfPoints3D
from TriangulationAStarR import TriangulationAStarR
from CcwShapes import CrossWithHole



def drawInds(adjLst):
    from direct.gui.OnscreenText import OnscreenText
    indNP = render.attachNewNode('indsgroup')
    for i in range(0, len(adjLst)):
        center = getCenterOfPoints3D(adjLst[i].tri)
        dummy = indNP.attachNewNode(str(i))
        txt = OnscreenText(text=str(i), pos=center, scale=1)
        txt.reparentTo(dummy)
        dummy.setP(dummy.getP() - 90)

    return indNP

class Pathfinding(ShowBase): #BareBonesEditor):
    def __init__(self):
        ShowBase.__init__(self)
        #BareBonesEditor.__init__(self)
        PanditorDisableMouseFunc()
        camera.setPos( 0.0, 0.0, 50.0)
        camera.lookAt(0.0)
        PanditorEnableMouseFunc()

        # mapThrs = TheirMap()
        mapThrs = CrossWithHole()
        print mapThrs[1]
        mesh_trilator = makeTriMesh(mapThrs[0], mapThrs[1])  # , holes) ###########

        aLst = AdjacencyList(mesh_trilator[1])

        indsNP = drawInds(aLst.adjLst)  # put text on each triangle
        indsNP.setPos(0.0, 0.0, .2)
        indsNP.setColor(0.0, 1.0, 1.0, 1.0)
        mapNP = render.attachNewNode(mesh_trilator[0])
        wireNP = render.attachNewNode('wire')
        wireNP.setPos(0.0, 0.0, .1)
        wireNP.setColor(1.0, 0.0, 0.0, 1)
        wireNP.setRenderMode(RenderModeAttrib.MWireframe, .5, 0)
        mapNP.instanceTo(wireNP)

        aStar = TriangulationAStarR(aLst.adjLst, Point3(0.0, -5.0, 0.0), Point3(0.0, 5.5, 0.0), radius=0.0)
        # aStar = TriangulationAStarR(aLst.adjLst, Point3(aLst.adjLst[17].getCenter() + Point3(5, 0, 0)), Point3(0, 11, 0), radius=.55)
        # aStar = TriangulationAStarR(aLst.adjLst, Point3(-5, 4, 0), Point3(aLst.adjLst[17].getCenter() + Point3(5, 0, 0)), radius=.55)
        path = aStar.AStar()
        print "\n\nEND PATH\n", path
        # https://www.panda3d.org/manual/index.php?title=Putting_your_new_geometry_in_the_scene_graph&diff=prev&oldid=6303
        linesegs = LineSegs("lines")
        linesegs.setColor(0, 0, 1, 1)
        linesegs.setThickness(5)
        for p in path:
            linesegs.drawTo(p)
        node = linesegs.create(False)
        nodePath = render.attachNewNode(node)
        nodePath.setZ(.15)


        # tests
        # pt = Point3(6.01, -12.0, 0)
        # p1 = Point3(3, -12, 0)
        # p2 = Point3(3, -6, 0)
        # p3 = Point3(6, -12, 0)
        # print isPointInWedge(pt, [p1, p2], [p1, p3])
        # nearest point on line test
        # line = [p1, p2]
        # nearest = getNearestPointOnLine(pt, line, True)
        # linesegs2 = LineSegs("lines2")
        # linesegs2.setColor(0, 1, 1, 1)
        # linesegs2.setThickness(5)
        # linesegs2.drawTo(p1)
        # linesegs2.drawTo(nearest)
        # linesegs2.setThickness(2)
        # linesegs2.drawTo(p2)
        # node2 = linesegs2.create(False)
        # nodePath = render.attachNewNode(node2)
        # nodePath.setZ(.25)
        # print nearest




if __name__ == '__main__':
    app = Pathfinding()
    app.run()
