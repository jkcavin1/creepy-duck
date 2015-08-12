__author__ = 'Lab Hatter'

from panda3d.core import Point3
# TODO: change the CcwShapes points to points instead of vectors as they are now DUMMY!!!
def HorseShoeCentered():
    """Returns a horseshoe shape, thus behaving like a constructor."""
    hole1 = []
    hole1.append(Point3(-2, -2, 0))
    hole1.append(Point3(-1, -2, 0))
    hole1.append(Point3(-1, .5, 0))
    hole1.append(Point3(1, .5, 0))
    hole1.append(Point3(1, -2, 0))
    hole1.append(Point3(2, -2, 0))
    hole1.append(Point3(2, 2, 0))
    hole1.append(Point3(-2, 2, 0))

    return hole1


def SquareOffCenter():
    """Behaves like a constructor by returning a square"""
    hole2 = []
    hole2.append(Point3(3, 3, 0))
    hole2.append(Point3(4, 3, 0))
    hole2.append(Point3(4, 4, 0))
    hole2.append(Point3(3, 4, 0))
    return hole2


def SquareMap10x10():
    map10 = []
    map10.append(Point3(-10, -10, 0))
    map10.append(Point3(10, -10, 0))
    map10.append(Point3(10, 10, 0))
    map10.append(Point3(-10, 10, 0))
    return map10

def TheirMap():
    thrMap = []
    thrMap.append(Point3(-12.0, -12.0, 0.0))
    thrMap.append(Point3(-3.0, -12.0, 0.0))
    thrMap.append(Point3(-3.0, -7.0, 0.0))
    thrMap.append(Point3(3.0, -6.0, 0.0))

    thrMap.append(Point3(3.0, -12.0, 0.0))
    thrMap.append(Point3(6.0, -12.0, 0.0))
    thrMap.append(Point3(6.0, -6.0, 0.0))
    thrMap.append(Point3(8.0, -7.0, 0.0))

    thrMap.append(Point3(8.0, -12.0, 0.0))
    thrMap.append(Point3(12.0, -12.0, 0.0))
    thrMap.append(Point3(12.0, -4.0, 0.0))
    thrMap.append(Point3(-3.0, -4.0, 0.0))

    thrMap.append(Point3(-3.0, -1.0, 0.0))
    thrMap.append(Point3(12.0, -1.0, 0.0))
    thrMap.append(Point3(12.0, 13.0, 0.0))  # Point3(12.0, 12.0, 0.0))
    thrMap.append(Point3(-12.0, 13.0, 0.0))  # Point3(-12.0, 12.0, 0.0))
    tri1 = []
    tri1.append(Point3(-5.0, 5.0, 0.0))  # Point3(-3.0, -0.25, 0.0))
    tri1.append(Point3(1.0, 3.0, 0.0))
    tri1.append(Point3(-3.0, 10.99, 0.0))  # Point3(-3.0, 9.0, 0.0)
    tri2 = []
    tri2.append(Point3(4.0, 2.0, 0.0))
    tri2.append(Point3(8.0, 3.0, 0.0))
    tri2.append(Point3(0.0, 12.25, 0.0))  # Point3(8.0, 9.0, 0.0))
    for i in range(0, 3):
        tri2[i] = tri2[i] + Point3(1.0, 0.0, 0.0)
    tris = []
    tris.append(tri1)
    tris.append(tri2)

    mapWHoles = []
    mapWHoles.append(thrMap)
    mapWHoles.append(tris)
    #print "mWholes ", mapWHoles
    # for i in mapWHoles:
    #     print "mWholes i", i

    return mapWHoles


def CrossWithHole():
    cross = []
    cross.append(Point3(-1.0, -6.0, 0.0))
    cross.append(Point3(1.0, -6.0, 0.0))
    cross.append(Point3(1.0, 0.01, 0.0))
    cross.append(Point3(6.0, 0.0, 0.0))
    cross.append(Point3(6.0, 6.0, 0.0))
    cross.append(Point3(-6.0, 6.0, 0.0))
    cross.append(Point3(-6.0, 0.0, 0.0))
    cross.append(Point3(-0.0, -0.5, 0.0))  # Point3(-1.0, 0.01, 0.0)

    hole = []
    hole.append(Point3(-1.0, 2.0, 0.0))
    hole.append(Point3(1.0, 2.0, 0.0))
    hole.append(Point3(0.0, 4.5, 0.0))

    return [cross, [hole]]


if __name__ == '__main__':
    app = TheirMap()