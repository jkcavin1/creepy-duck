__author__ = 'Lab Hatter'

from panda3d.core import Point3


def getSharedEdgeStr(triA, triB):
    """Returns the edge that B shares w/ A i.e. If B lies on A's 12 edge, returns '12', otherwise returns '' or '1'"""
    if triA is None or triB is None:
        return ''

    pointsA = triA.getPoints()
    pointsB = triB.getPoints()
    # two matching vertices means their neighbours
    found = ''
    for i in range(0, len(pointsA)):
        if pointsA[i] is not None and pointsA[i] == pointsB[0] or pointsA[i] == pointsB[1] or pointsA[i] == pointsB[2]:
            found = str(i + 1)
            break
    # none match
    if found == '':
        return found

    # now that we found one lets see if there's a second match
    if i + 1 <= len(pointsA):
        for j in range(i + 1, len(pointsA)):
            if pointsA[j] is not None:  # and j != i:
                if pointsA[j] == pointsB[0] or pointsA[j] == pointsB[1] or pointsA[j] == pointsB[2]:
                    found += str(j + 1)
                    return found

    return found


class Triangle(object):
    def __init__(self, pt1, pt2, pt3):
        self.setTri(pt1, pt2, pt3)


    def setTri(self, pt1, pt2, pt3):

        tri = [pt1, pt2, pt3]
        rightVec = tri[1] - tri[0]
        leftVec = tri[2] - tri[0]
        if rightVec.cross(leftVec).z < 0:
            tmp = tri[1]
            tri[1] = tri[2]
            tri[2] = tmp
        self.tri = tri
        # print "triangle = ", self.tri
        self.pt1 = pt1
        self.pt2 = pt2
        self.pt3 = pt3

    def getTri(self):
        return tuple((self.pt1, self.pt2, self.pt3))

    def getPoint1(self):
        return self.tri[0]

    def getPoint2(self):
        return self.tri[1]

    def getPoint3(self):
        return self.tri[2]

    def getEdge12(self):
        return [self.pt1, self.pt2]

    def getEdge23(self):
        return [self.pt2, self.pt3]

    def getEdge13(self):
        return [self.pt1, self.pt2]

    def getPoints(self):
        return [self.pt1, self.pt2, self.pt3]

    def getCenter(self):
        return Point3((self.pt1.x + self.pt2.x + self.pt3.x)/3,
                      (self.pt1.y + self.pt2.y + self.pt3.y)/3,
                      (self.pt1.z + self.pt2.z + self.pt3.z)/3)

    def getSharedPoints(self, other):
        pts = []
        found = False
        for pt in self.tri:
            # print "pt = ", pt
            if pt in other.tri:
                found = True
                pts.append(pt)
        if not found:
            sr = "getSharedPoints requires at least one point be in both triangles\nself"\
                 + str(self) + "\nother" + str(other)
            raise Exception(sr)
        return pts

    def getNonSharedPoint(self, other):
        shrd = self.getSharedPoints(other)
        for pt in self.tri:
            if pt not in other.tri:
                return pt

    # def __str__(self):
    #     return "dmflksdj;fklj"
    #
    # def __repr__(self):
    #     sr = "Tri: < " + self.pt1 + ", " + self.pt2 + ", " + self.pt3 + " >"
    #     return "dmflksdj;fklj"



if __name__ == '__main__':
    app = Triangle()