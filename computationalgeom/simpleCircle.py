#!/usr/bin/python


class SimpleCircle(object):
    __slots__ = ('center', 'radius', )

    def __init__(self, center, radius):
        super(SimpleCircle, self).__init__()
        self.center = center
        self.radius = radius

    def __str__(self):
        return "radius {0}, center: {1}".format(self.radius, self.center)
