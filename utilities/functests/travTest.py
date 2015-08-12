__author__ = 'Lab Hatter'

from direct.showbase.ShowBase import ShowBase


class TravTest(ShowBase):
    def __init__(self):
        ShowBase.__init__(self)
        self.camera.setPos(25, 25, 25)
        self.camera.lookAt(0, 0, 0)
        #z = 0
        #self.plane = Plane(Vec3(0, 0, 1), Point3(0, 0, z))

        dummy = render.attachNewNode('dummy')
        for i in range(0, 4):
            model = loader.loadModel("jack")
            model.setTag('pickable', str(i))
            model.setPos( i, i + 0.5, 0.0)
            #model.setScale(0.5)
            model.reparentTo(dummy)
        self.traverse()

        print "\nrender.ls()", render.ls()

    def traverse(self):
        import barebones.utilities.pandaHelperFuncs as helpers
        helpers.TraverserLevelFirst(render,self.myPrint)

    def myPrint(self, NP):
        print "currNP: ", NP


ap = TravTest()
ap.run()