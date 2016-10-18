__author__ = 'Lab Hatter'


import math
import heapq
from panda3d.core import Vec3, Point3, LineSegs
from PolygonUtils.PolygonUtils import getDistance, getCenterOfPoints3D, getNearestPointOnLine,\
    getLeftPt, makeTriangleCcw, triangleContainsPoint, getDistToLine, isPointInWedge
from PolygonUtils.AdjacencyList import AdjLstElement, copyAdjLstElement, getSharedEdgeStr



class FunVecs(object):
    def __init__(self, start, left, lPt, right, rPt):
        self.startPt = start
        self.leftVec = left
        self.lPt = lPt
        self.rightVec = right
        self.rPt = rPt

    def getCross(self):
        return self.leftVec.cross(self.rightVec)

    def updateLeft(self, newPt):
        self.lPt = newPt
        self.leftVec = Vec3(newPt - self.startPt)

    def updateRight(self, newPt):
        self.rPt = newPt
        self.rightVec = Vec3(newPt - self.startPt)

    def reset(self, start, proposedLeft, proposedRight):
        left = Vec3(proposedLeft - start)
        lpt = proposedLeft
        right = Vec3(proposedRight - start)
        rpt = proposedRight
        # if these are on the wrong side switch them
        # print "startPt", startPt  #, " mid - startPt ", getCenterOfPoints3D(shared)
        mid = getCenterOfPoints3D([proposedLeft, proposedRight]) - start
        # print "mid", mid, midP, " leftVec - mid ", lpt-mid, shared, leftVec
        if left.cross(mid).z > 0.0:
            # print "swap"
            tmp = left
            left = right
            right = tmp

            tmp = lpt
            lpt = rpt
            rpt = tmp
        self.startPt = start
        self.leftVec = left
        self.rightVec = right
        self.rPt = rpt
        self.lPt = lpt
        # print "reset", self


    def area(self, vec1, vec2):
        ax = vec1.x
        ay = self.startPt.y
        bx = vec1.x
        by = vec1.y
        cx = vec2.x
        cy = vec2.y
        return ax * (by - cy) + bx * (cy - ay) + cx * (ay - by)

    def __repr__(self):
        return "< startPt: " + str(self.startPt) +\
               ", leftPt: " + str(self.lPt) +\
               ", rightPt: " + str(self.rPt) #+\
               #", leftVec: " + str(self.leftVec) +\
               #", rightVec: " + str(self.rightVec) +\
               #">"


class TriangulationAStarR(object):
    def __init__(self, adjLst, startPt, goalPt, radius=0):
        self.adjLst = adjLst

        # use vectors to determine triangles contain the startPt and the goalPt
        for t in adjLst:
            if triangleContainsPoint(startPt, t.tri):
                startTri = t.selfInd
            if triangleContainsPoint(goalPt, t.tri):
                goalTri = t.selfInd


        self.startPt = startPt
        self.start = adjLst[startTri]
        self.start.g = 0
        self.start.f = 0
        self.goalPt = goalPt
        self.goal = adjLst[goalTri]
        self.open = []
        heapq.heappush(self.open, (0, self.start))
        self.closed = dict()
        self.closed[str(self.start.selfInd)] = self.start
        self.curr = self.start
        self.bestPath = None
        self.bestPathDist = 10000
        self.radius = radius

    def AStar(self):
        print "start AStar start: ", self.start.selfInd, " startPt ", self.startPt,\
            " goal: ", self.goal.selfInd, " goalPt ", self.goalPt
        bestPath = path = []
        bestPathCost = 100000
        if self.start == self.goal:
            return [self.startPt, self.goalPt]

        pathsVisited = []
        while self.open != []:
            n = heapq.heappop(self.open)[1]  # open = [(]
            print "tri ind " + str(n.selfInd), " f: ", n.f
            isFirst = True
            bestF = 100000
            bestInd = -1
            # resolve ties in favor of best path
            for chldInd in n.getNaybs():
                if str(chldInd) in self.closed and n.selfInd != self.closed[str(chldInd)].par:
                    print "####    find parent in closed    ####"
                    # making the print work below
                    w = self.getWidthThrough(n, self.closed[str(chldInd)])
                    print "ind " + str(chldInd) + " is in closed." + " Width: " + str(w)
                    print "bestF = ", bestF, "   f to check ", self.closed[str(chldInd)].f
                    if isFirst and self.getWidthThrough(n, self.closed[str(chldInd)]) > 2*self.radius:
                        # do not parent the goal to a path that has already been through he funnel
                        if chldInd not in pathsVisited:
                            print "first and width good"
                            bestF = self.closed[str(chldInd)].f
                            bestInd = self.closed[str(chldInd)].selfInd
                            isFirst = False
                    elif self.closed[str(chldInd)].f < bestF\
                            and self.getWidthThrough(n, self.closed[str(chldInd)]) > 2*self.radius:
                        # do not parent the goal to a path that has already been through he funnel
                        if chldInd not in pathsVisited:
                            print "better f and width good"
                            bestF = self.closed[str(chldInd)].f
                            bestInd = self.closed[str(chldInd)].selfInd

            if bestInd != -1:  # we found a legal parent
                print "parented to ", bestInd
                n.par = bestInd

            # once the nodes we're getting from open are costlier than our path, we've found the best path
            if n.f > bestPathCost:
                print "break ind " + str(n.selfInd), " n.f ", n.f, " bestPathCost ", bestPathCost
                break

            if n == self.goal:
                print "################       FOUND GOAL       ####################"
                path = self.makeChannel(self.goal, self.closed[str(self.goal.par)])

                cost = 0
                for c in range(0, len(path) - 1):
                    cost += getDistance(path[c], path[c + 1])

                # keep track of what paths we've traversed, so they don't get re-traversed.
                pathsVisited.append(self.goal.par)

                # # Reset the goal so we can recalculate f, g, and h for other potential paths
                self.goal.par = None
                self.goal.g = 100000
                self.goal.f = 100000


                if bestPathCost == -1:  # this is the first path
                    bestPath = path
                    bestPathCost = cost
                elif cost < bestPathCost:
                    bestPath = path
                    bestPathCost = cost

            if n != self.goal:
                # put n in closed as long as it's not the goal
                self.closed[str(n.selfInd)] = n

            for chldInd in n.getNaybs():
                print "####   EXPAND N   ####"
                # Never expand the goal. We cannot have one of the goal's children parented to the goal.
                # That'd be backwards.
                if n == self.goal:
                    break
                sChl = str(chldInd)
                # print "child ", sChl, "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"
                # get the width of the path through each side
                if self.adjLst[chldInd].n12 is None:
                    w12 = getDistToLine(self.adjLst[chldInd].tri[2], self.adjLst[chldInd].tri[0], self.adjLst[chldInd].tri[1])
                else:
                    w12 = self.getWidthAcrossEdges(self.adjLst[chldInd],
                                                        [self.adjLst[chldInd].tri[1], self.adjLst[chldInd].tri[2]],
                                                          [self.adjLst[chldInd].tri[0], self.adjLst[chldInd].tri[2]])
                # print " w2313: ", w2313, "<<<<<<<<<<<<<<<<<<<<<<<<<<"
                if self.adjLst[chldInd].n23 is None:
                    w23 = getDistToLine(self.adjLst[chldInd].tri[0], self.adjLst[chldInd].tri[1], self.adjLst[chldInd].tri[2])
                else:
                    w23 = self.getWidthAcrossEdges(self.adjLst[chldInd],
                                                        [self.adjLst[chldInd].tri[0], self.adjLst[chldInd].tri[1]],
                                                          [self.adjLst[chldInd].tri[0], self.adjLst[chldInd].tri[2]])
                # print "w2313: ", w2313, " w1213: ", w1213, "<<<<<<<<<<<<<<<<<<<<<<<<<<"
                if self.adjLst[chldInd].n13 is None:
                    w13 = getDistToLine(self.adjLst[chldInd].tri[1], self.adjLst[chldInd].tri[0], self.adjLst[chldInd].tri[2])
                else:
                    w13 = self.getWidthAcrossEdges(self.adjLst[chldInd],
                                                        [self.adjLst[chldInd].tri[0], self.adjLst[chldInd].tri[1]],
                                                          [self.adjLst[chldInd].tri[1], self.adjLst[chldInd].tri[2]])

                print "n ind", n.selfInd, "chl ind " + sChl + " nw12: ", w12, " w1213: ", w23, " w1223: ", w13, "<<<<<<<<<<<<<<<<<<<<<<<<<<"

                nrToG = self.getNearestTrianglePtToStartOrGoal(self.adjLst[chldInd])
                h = getDistance(self.goalPt, nrToG)
                # TODO: handle their MAX( g1, g2, g3,...) OR don't and leave it to my shortened version
                g = self.calculateG(chldInd, h, n)
                f = h + g
                print "g: ", g, " h: ", h
                if sChl not in self.closed:# or f < self.closed[sChl].f:
                    if self.getWidthThrough(self.adjLst[chldInd], n) <= 2*self.radius:
                        print "ignore child width <= r width ", self.getWidthThrough(self.adjLst[chldInd], n),\
                                " from ", self.adjLst[chldInd].selfInd, " to ", n.selfInd
                        continue

                    self.adjLst[chldInd].f = f
                    self.adjLst[chldInd].g = g
                    self.adjLst[chldInd].w2313 = w12
                    self.adjLst[chldInd].w1213 = w23
                    self.adjLst[chldInd].w1223 = w13
                    w = str(self.getWidthThrough(self.adjLst[chldInd], n))  # make the print below work (print bug)
                    print "ind " + sChl + " child width = ", w
                    print "put in open f = ", f
                    heapq.heappush(self.open, (f, self.adjLst[chldInd]))
                elif sChl in self.closed and f < self.closed[sChl].f:# or chldInd == self.goal.selfInd:    ## and self.getWidthThrough(self.closed[sChl], n) > 2*self.radius:
                    print "ind " + sChl + " in closed w/ better f. bestPathCost ", bestPathCost, " chldInd.f ", f
                    self.closed[sChl] = self.adjLst[chldInd]
                    self.closed[sChl].f = f
                    self.closed[sChl].g = g
                    # self.closed[sChl].w2313 = w12
                    # self.closed[sChl].w1213 = w23
                    # self.closed[sChl].w1223 = w13
                    heapq.heappush(self.open, (f, self.adjLst[chldInd]))

                # print "end child ", self.adjLst[chldInd]

            print "end AStar loop #####################################################################\n\n"

        print "best path? ", bestPathCost, " f ", n.f
        for i in range(0, len(self.open)):
            opn = self.open[i][1]
            print "open ind", opn.selfInd, "f", opn.f
        # if the start and goal are not in the same triangle
        # if self.goal.par is not None:
        return bestPath
        # return self.makeChannel(self.goal, self.closed[str(self.goal.par)])
        # else:
        #     return [self.startPt, self.goalPt]
    def getNearestTrianglePtToStartOrGoal(self, tri, target='goal'):
        """Gets the nearest point on the triangle to the target triangle."""
        if target == 'goal':
            # print "get nearest to goal"
            target = self.goalPt
        elif target == 'start':
            # print "get nearest to start"
            target = self.startPt

        clst12 = getNearestPointOnLine(target, [tri.tri[0], tri.tri[1]], asLineSeg=True)
        clst23 = getNearestPointOnLine(target, [tri.tri[1], tri.tri[2]], asLineSeg=True)
        clst13 = getNearestPointOnLine(target, [tri.tri[0], tri.tri[2]], asLineSeg=True)
        minDist = getDistance(target, clst12)
        clstPt = clst12
        # print "getNearestPt\n12 ", clst12, " 23 ", clst23, " 13 ", clst13
        if minDist > getDistance(target, clst23):
            minDist = getDistance(target, clst23)
            clstPt = clst23

        if minDist > getDistance(target, clst13):
            clstPt = clst13
        # print "clstPt ", clstPt
        return clstPt

    def calculateG(self, chldInd, h, n):
        child = self.adjLst[chldInd]
        # region Calculate g using makeChannel PROBLEMATIC!!!
        # child = copyAdjLstElement(child)
        # # child.par = n.selfInd
        # g = 0
        #
        # # swap out the  goal so that the funnel algorithm will calculate based on this temporary goal
        # print " child goal ", child
        # tmpG = self.goal
        # self.goal = child
        # tmpGPt = self.goalPt
        # self.goalPt = self.getNearestTrianglePtToStartOrGoal(child, target='start')
        #
        # path = self.makeChannel(child, n)
        # # just for printing
        # pt = self.getNearestTrianglePtToStartOrGoal(child, target='start')
        # print "nearest point to start ", pt, " goalPt ", self.goalPt, " tmpGPt ", tmpGPt, "\npath", path
        # for p in range(0, len(path) - 1):
        #     # The goal pt is the last point but we need to use a point in this triangle
        #     g += getDistance(path[p], path[p + 1])
        #     print "g counter ", g, " point p", path[p], " point p + 1", path[p + 1]
        #
        # self.goal = tmpG
        # self.goalPt = tmpGPt
        # return g
        # endregion


        # 1
        shPts = n.getSharedPoints(child)
        closeToSDist = getDistance(shPts[0], self.startPt)
        if chldInd == self.start.selfInd:
            closeToSDist = 0
        else:
            for p in shPts:
                if closeToSDist > getDistance(p, self.startPt):
                    closeToSDist = getDistance(p, self.startPt)
        # 2 ??? Why isn't this used
        closeToGDist = getDistance(child.tri[0], self.goalPt)
        for p in child.tri:
            if closeToGDist > getDistance(p, self.goalPt):
                closeToGDist = getDistance(p, self.goalPt)

        startGoalH = getDistance(self.startPt, self.goalPt) - h

        # 3
        if child.par is not None:
            parGdiffHH = self.adjLst[child.par].g + (self.adjLst[child.par].g - closeToGDist)
        else:
            parGdiffHH = 0


        return max(closeToSDist, startGoalH, parGdiffHH)


    def makeChannel(self, end, nextN, start=None):
        """Takes the end of a channel of triangles and creates lists of Right and Left points that lead through the channel"""
        if start is None:
            start = self.start
        # Not good: if the two are naybs they may have a corner between them
        # for nayb in end.getNaybs():
        #     if nayb in nextN.getNaybs():
        #         # p = end.getSharedPoints(nextN)
        #         # return p
        #         return [self.startPt, self.goalPt]
        # print "make channel"
        end = copyAdjLstElement(end)
        for ii in range(0, 3):
            if end.tri[ii] not in end.getSharedPoints(nextN):
                end.tri[ii] = self.goalPt
        # end.par = nextN.selfInd
        channel = [end]

        curr = nextN
        # make a channel out of the list of adjacency indexes
        while curr != start:
            # copy the adj triangles out so we can strip references to non-channel triangle without messing of the map

            cpy = copyAdjLstElement(self.closed[str(curr.selfInd)])
            channel.append(cpy)
            currKey = str(curr.selfInd)
            print "currKey", currKey
            parKey = str(self.closed[currKey].par)
            curr = self.closed[parKey]

        # cpy is a copy of the startPt
        # make the special case triangle for the startPt
        cpyInd = str(self.start.selfInd)
        cpy = copyAdjLstElement(self.closed[cpyInd])
        channel.append(cpy)
        channel = list(reversed(channel))

        # pick the starting point and the starting leftVec and rightVec
        # make the starting triangle such that it's points are the starting point and the points shared with the next tri
        shrdPts = channel[0].getSharedPoints(channel[1])
        start = copyAdjLstElement(channel[0])  # channel[0]
        # print "after copy\n", start
        for pp in range(0, 3):
            if start.tri[pp] not in shrdPts:
                # print "pp not in", start.tri[pp]
                start.tri[pp] = self.startPt
        # print "shrdPts",  shrdPts
        # print "after point change\n", start
        # print channel[1]
        channel[0] = start
        # TODO: handle an arbitrary point as the goal
        goalPoint = self.goalPt  # end.getCenter()
        path = self.funnel(channel, goalPoint)
        # return channel
        return path

    def funnel(self, channel, goalPt):
        """creates a true path out of the given funnel and returns the points and length"""
        def isDistSmall(a, b):
            tol = 0.0001*0.0001
            cX = a.x - b.x
            cY = a.y - b.y
            return math.sqrt(cX*cX + cY*cY) < tol
        # print "\n\n\n\n"
        start = channel[0]
        second = channel[1]
        funVecs = self.makeFunVecs(start, second)
        pathPts = [funVecs.startPt]
        # run funnel algorithm
        nxtL = nxtR = Point3(0)
        leftInd = rightInd = i = 0  # TOCHECK: may need to restart using leftVec and rightVec verts
        while i < len(channel) - 1:
            # have to look at the vertices on the entry edge. Check both of them against the funnel
            sharedPts = channel[i].getSharedPoints(channel[i + 1])
            # if mid cross the fist point is poss then the first pt is on the leftVec
            # if midVec.cross(sharedPts[0] - funVecs.startPt).z >= 0:
            if getLeftPt(channel[i].getCenter(), sharedPts) == sharedPts[0]:
                nxtL = sharedPts[0]
                vecToNxtL = nxtL - funVecs.startPt
                nxtR = sharedPts[1]
                vecToNxtR = nxtR - funVecs.startPt
            else:  # its the other way around
                nxtL = sharedPts[1]
                vecToNxtL = nxtL - funVecs.startPt
                nxtR = sharedPts[0]
                vecToNxtR = nxtR - funVecs.startPt

            print "NEXT TRI", channel[i], "\n", "next L", nxtL, "next R", nxtR

            # if the point is outside on the leftVec hold, else the next point is to the rightVec of the leftVec side
            if funVecs.leftVec.cross(vecToNxtL).z <= 0:  # 1 don't update if the next vert is outside the funnel
                # print "leftVec good"
                # if the next point is to the leftVec of the rightVec side
                # it's still inside the funnel, update the leftVec vector
                if isDistSmall(funVecs.startPt, nxtL) or funVecs.rightVec.cross(vecToNxtL).z >= 0:  # 2
                    # print "leftVec still good"
                    funVecs.updateLeft(nxtL)
                    leftInd = i
                else:  # if we've crossed the rightVec we need a new point for startPt (apex)
                    # print "fail cross R", funVecs.rightVec.cross(vecToNxtL), " L ", funVecs.leftVec.cross(vecToNxtL).z
                    # set the rightVec point as the new startPt
                    funVecs.startPt = funVecs.rPt
                    # print "append***** rPt", funVecs.rPt
                    pathPts.append(funVecs.rPt)
                    i = rightInd
                    # if mid cross the fist point is poss then the first pt is on the leftVec
                    # if midVec.cross(sharedPts[0] - funVecs.startPt).z >= 0:
                    if getLeftPt(channel[i].getCenter(), sharedPts) == sharedPts[0]:
                        funVecs.updateLeft(sharedPts[0])
                        funVecs.updateRight(sharedPts[1])
                    else:  # its the other way around
                        funVecs.updateLeft(sharedPts[1])
                        funVecs.updateRight(sharedPts[0])

            # print "funVecs1", funVecs, "   i", i

            # 1 don't update if the next vert is outside the funnel on the rightVec
            if funVecs.rightVec.cross(vecToNxtR).z >= 0:
                # print "rightVec good"
                # make sure it didn't cross the leftVec side
                if isDistSmall(funVecs.startPt, nxtR) or funVecs.leftVec.cross(vecToNxtR).z <= 0:  # 2
                    funVecs.updateRight(nxtR)
                    rightInd = i
                    # print "still good"#, funVecs, " LL  cross ", funVecs.getCross(), "    i ", i
                else:  # if we've crossed the leftVec we need a new point for startPt (apex)
                    # print "fail cross R", funVecs.rightVec.cross(vecToNxtR), " L ", funVecs.leftVec.cross(vecToNxtR)
                    # do the same as above but for the leftVec
                    funVecs.startPt = funVecs.lPt
                    # print "append***** lPt", funVecs.rPt
                    pathPts.append(funVecs.lPt)
                    i = leftInd
                    # if mid cross the fist point is poss then the first pt is on the leftVec
                    # if midVec.cross(sharedPts[0] - funVecs.startPt).z >= 0:
                    if getLeftPt(channel[i].getCenter(), sharedPts) == sharedPts[0]:
                        funVecs.updateLeft(sharedPts[0])
                        funVecs.updateRight(sharedPts[1])
                    else:  # its the other way around
                        funVecs.updateLeft(sharedPts[1])
                        funVecs.updateRight(sharedPts[0])
            # print "############## funnel loop ###################\n###########################################"
            # keep the indices moving so we don't start back all the way to where we first encountered the points
            if funVecs.rPt in channel[i].tri:
                rightInd = i

            if funVecs.lPt in channel[i].tri:
                leftInd = i

            # print "funVecs2", funVecs, "   i", i
            # print "path", pathPts, "\n"
            i += 1

        vecToGoal = goalPt - funVecs.startPt
        # REMOVE below


        # REMOVE above

        # print "\n\n############## funneler ###################\n######################################################"
        # print "last rightVec", funVecs.rPt, "last left", funVecs.lPt
        if funVecs.leftVec.cross(vecToGoal).z >= 0:
            # print "if goal outside left"
            # if the goal is to the left of the left side, it needs added as a corner.
            # Check the next point on the left.
            # if the next point is to the right of the vector pointing from the left to the goal,
            # it needs added as another corner.
            pathPts.append(funVecs.lPt)
            funVecs.startPt = funVecs.lPt
            if leftInd + 2 < len(channel):
                # print "if ind check OK"
                # I should check the remaining left side points in a loop, but the map is not jagged so I'll forgo that
                i = leftInd + 1
                sharedPts = channel[i].getSharedPoints(channel[i + 1])
                # if mid cross the fist point is poss then the first pt is on the leftVec
                # if midVec.cross(sharedPts[0] - funVecs.startPt).z >= 0:
                if getLeftPt(channel[i].getCenter(), sharedPts) == sharedPts[0]:
                    funVecs.updateLeft(sharedPts[0])
                else:  # its the other way around
                    funVecs.updateLeft(sharedPts[1])
                vecToGoal = goalPt - funVecs.startPt
                if funVecs.leftVec.cross(vecToGoal).z >= 0:
                    # print "last check append lPt", funVecs.lPt
                    pathPts.append(funVecs.lPt)


        elif funVecs.rightVec.cross(vecToGoal).z <= 0:
            pathPts.append(funVecs.rPt)  # #############  check if the goal is rightVec of the rightVec side
            funVecs.startPt = funVecs.rPt
            # print "goal outside right"
            if rightInd + 2 < len(channel):
                # print "if ind check ok rightInd = ", rightInd
                # I should check the remaining left side points in a loop, but the map is not jagged so I'll forgo that
                i = rightInd + 1
                sharedPts = channel[i].getSharedPoints(channel[i + 1])
                # if mid cross the fist point is positive then the first pt is on the left
                if getLeftPt(channel[i].getCenter(), sharedPts) == sharedPts[0]:
                    funVecs.updateRight(sharedPts[1])
                    # print "next rPt", sharedPts[1], " new rPt", funVecs.rPt
                else:  # its the other way around
                    # print "next rPt", sharedPts[0], " new rPt", funVecs.rPt
                    funVecs.updateRight(sharedPts[0])
                vecToGoal = goalPt - funVecs.startPt
                if funVecs.rightVec.cross(vecToGoal).z <= 0:
                    # print "last check append rPt", funVecs.rPt
                    pathPts.append(funVecs.rPt)

        pathPts.append(goalPt)
        return pathPts

    def getNextVec(self, i, channel):
        # the edge is the edge on channel[i + 1] NOT i
        edge = getSharedEdgeStr(channel[i + 1], channel[i])
        # find which point in the next triangle isn't also in this triangle
        # then get it's vector and a vector to the edge's mid point so we can figure out what side the pt is on
        # print "getNextVec\n", channel[i + 1], "\n", channel[i]
        if edge == "12":  # the new point is either the 1st or second point in the triangle
            # check the other edges to get the one that leads out of i + 1
            # get the vec to it's midpoint
            if channel[i + 1].n23 is not None and self.adjLst[channel[i + 1].n23] in channel:
                vecToMid = getCenterOfPoints3D([channel[i + 1].tri[1], channel[i + 1].tri[2]])\
                            - channel[i].getCenter()
            else:  # it's the other edge
                vecToMid = getCenterOfPoints3D([channel[i + 1].tri[0], channel[i + 1].tri[2]])\
                            - channel[i].getCenter()
            vecToNxt = channel[i + 1].tri[2] - channel[i].getCenter()

        elif edge == "23":  # check the second edge in like fashion
            if channel[i + 1].n12 is not None and self.adjLst[channel[i + 1].n12] in channel:
                vecToMid = getCenterOfPoints3D([channel[i + 1].tri[0], channel[i + 1].tri[1]])\
                            - channel[i].getCenter()
            else:  # it's the other edge
                vecToMid = getCenterOfPoints3D([channel[i + 1].tri[0], channel[i + 1].tri[2]])\
                            - channel[i].getCenter()
            vecToNxt = channel[i + 1].tri[0] - channel[i].getCenter()
        else:  # edge == "13" the new point must be in this edge
            if channel[i + 1].n12 is not None and self.adjLst[channel[i + 1].n12] in channel:
                vecToMid = getCenterOfPoints3D([channel[i + 1].tri[0], channel[i + 1].tri[1]])\
                            - channel[i].getCenter()
            else:  # it's the other edge
                vecToMid = getCenterOfPoints3D([channel[i + 1].tri[1], channel[i + 1].tri[2]])\
                            - channel[i].getCenter()

            vecToNxt = channel[i + 1].tri[1] - channel[i].getCenter()

        if vecToNxt.cross(vecToMid).z >= 0:
            # print "rightVec", channel[i].selfInd, vecToMid, vecToNxt, "next",\
            #     channel[i].getCenter().x + vecToNxt.x, channel[i].getCenter().y + vecToNxt.y
            return ["rightVec", vecToNxt, Point3( channel[i].getCenter().x + vecToNxt.x,
                                              channel[i].getCenter().y + vecToNxt.y,
                                              channel[i].getCenter().z + vecToNxt.z)]
        else:
            # print "leftVec", channel[i].selfInd, vecToMid, vecToNxt, "next",\
            #     channel[i].getCenter().x + vecToNxt.x, channel[i].getCenter().y + vecToNxt.y
            return ["leftVec", vecToNxt, Point3( channel[i].getCenter().x + vecToNxt.x,
                                              channel[i].getCenter().y + vecToNxt.y,
                                              channel[i].getCenter().z + vecToNxt.z)]

    def makeFunVecs(self, start, second):
        shared = start.getSharedPoints(second)
        for i in start.tri:
            if i not in shared:
                startPt = i
        left = Vec3(shared[0] - startPt)
        lpt = shared[0]
        right = Vec3(shared[1] - startPt)
        rpt = shared[1]
        # if these are on the wrong side switch them
        # print "startPt", startPt  #, " mid - startPt ", getCenterOfPoints3D(shared)
        mid = getCenterOfPoints3D(shared) - startPt
        # print "mid", mid, midP, " leftVec - mid ", lpt-mid, shared, leftVec
        if left.cross(mid).z > 0.0:  #(leftVec - mid).y >= 0.0:
            # print "swap"
            tmp = left
            left = right
            right = tmp

            tmp = lpt
            lpt = rpt
            rpt = tmp
        funVec = FunVecs(startPt, left, lpt, right, rpt)
        # print "make funner ", funVec
        return funVec


    def getWidthThrough(self, tri1, tri2):
        """Returns the width of tri2 when crossed from tr1 to tri2 on to tri2's parent (if it has a parent)."""
        # the following gives the edge on the 1st triangle that the 2nd passed triangle lies on
        edgeIn = getSharedEdgeStr(tri2, tri1)
        if tri2.par is not None:
            edgeOut = getSharedEdgeStr(tri2, self.adjLst[tri2.par])
        else:
            # TODO: take the least width between the two possible exit edges
            # print "parent == None"
            if edgeIn == '12':
                return getDistance(tri2.getPoint1(), tri2.getPoint2())
            elif edgeIn == '23':
                return getDistance(tri2.getPoint2(), tri2.getPoint3())
            else:
                return getDistance(tri2.getPoint1(), tri2.getPoint3())
        # print "parent does NOT equal None"
        # 12 always comes first. 13 always comes last. Possibilities are 12, 23, 13 mutually exclusive (not duplicates)
        crossedEdges = ''
        if edgeIn == '12':
            crossedEdges = edgeIn + edgeOut
        elif edgeOut == '12':
            crossedEdges = edgeOut + edgeIn
        elif edgeIn == '13':
            crossedEdges = edgeOut + edgeIn
        elif edgeOut == '13':
            crossedEdges = edgeIn + edgeOut
        else:
            msg = "getWidthThrough defaulted edge match-up edgeIn: " + edgeIn + " edgeOut: " + edgeOut
            raise StandardError(msg)
        # crossedEdges = 'w' + crossedEdges

        #print crossedEdges + " =====================  crossedEdges"
        if crossedEdges == '1223':
            return tri2.w1223
        elif crossedEdges == '2313':
            return tri2.w2313
        elif crossedEdges == '1213':
            return tri2.w1213
        else:
            msg = "getWidthThrough defaulted return value crossedEdges: " + crossedEdges
            raise StandardError(msg)

    def getWidthAcrossEdges(self, searchTri, edge1, edge2):
        """Calculates the path width through this triangle. Edge1 and edge2 are the edges being crossed."""
        # this calculates the distance from the point shared by edge1 and edge2 to the nearest obstacle
        # 1st it sets the width of the triangle to the shortest edge being crossed
        # then it searches across the third edge to see if there is an obstacle closer than its own vertices
        # yes that can happen!!!
        for p in edge1:
            if p in edge2:
                pt = p  # get the point that both edges share. This is the point we are measuring the distance to.

        if edge2 == searchTri.getEdge12() and searchTri.n12 is None\
            or edge2 == searchTri.getEdge23() and searchTri.n23 is None\
            or edge2 == searchTri.getEdge13() and searchTri.n13 is None:
            # if edge2 is on a constrained side swap it for edge1
            # doint this makes it so we only have to check edge1. It cuts our code for the next step in half.
            tmp = edge2
            edge2 = edge1
            edge1 = tmp

        # TODO make this work with edge 1, 2, & 3 and local vars nayb 1, 2, & 3 so it's not sooo much code
        if edge1 == searchTri.getEdge12() and searchTri.n12 is None:
            if edge2 == searchTri.getEdge23() and searchTri.n23 is None:
                # Both search edges are constrained, so the width of the triangle is the width of the third edge.
                return getDistance(searchTri.getPoint1(), searchTri.getPoint3())

            elif edge2 == searchTri.getEdge13() and searchTri.n13 is None:
                # ditto
                return getDistance(searchTri.getPoint2(), searchTri.getPoint3())
            else:
                # the other edge is not constrained, so the initial width
                # should be the shortest of either the length of this unconstrained edge
                # or the distance from its non-shared point to the constrained side
                if edge2 == searchTri.getEdge23():
                    minWidth = getDistance(searchTri.getPoint2(), searchTri.getPoint3())
                    # we also need to find the end point for the next step
                    if pt != edge2[0]:
                        otherPt = edge2[0]
                    else:
                        otherPt = edge2[1]
                else:  # the other (non-constrained) edge is 13
                    minWidth = getDistance(searchTri.getPoint1(), searchTri.getPoint3())
                    # we also need to find the end point for the next step
                    if pt != edge2[0]:
                        otherPt = edge2[0]
                    else:
                        otherPt = edge2[1]
                # so now get the distance from the end to the other (constrained) edge
                debugEdgeConstrained = edge1  ############################# DEBUG
                distAcrossTri = getDistance(getNearestPointOnLine(otherPt, edge1),
                                            otherPt)
                if distAcrossTri < minWidth:
                    minWidth = distAcrossTri
                # ########################### this next bit seems off
                else:
                    minWidth = getDistance(searchTri.getPoint1(), searchTri.getPoint3())
        elif edge1 == searchTri.getEdge13() and searchTri.n13 is None:

            if edge2 == searchTri.getEdge23() and searchTri.n23 is None:
                # both are constrained, so the width of the triangle is the length of the unconstrained edge
                return getDistance(searchTri.getPoint1(), searchTri.getPoint2())

            elif edge2 == searchTri.getEdge12() and searchTri.n12 is None:
                return getDistance(searchTri.getPoint2(), searchTri.getPoint3())
            else:
                # the other edge is not constrained, so the initial width
                # should be either the length of this unconstrained edge
                # or the distance from its non-shared point to the constrained side, whichever is shortest
                if edge2 == searchTri.getEdge23():
                    minWidth = getDistance(searchTri.getPoint2(), searchTri.getPoint3())
                    # we also need to find the end point for the next step
                    if pt != edge2[0]:
                        otherPt = edge2[0]
                    else:
                        otherPt = edge2[1]
                else:  # the other (non-constrained) edge is 12
                    minWidth = getDistance(searchTri.getPoint1(), searchTri.getPoint2())
                    # we also need to find the end point for the next step
                    if pt != edge2[0]:
                        otherPt = edge2[0]
                    else:
                        otherPt = edge2[1]
                # so now get the distance from the end to the other (constrained) edge
                debugEdgeConstrained = edge1  ############################# DEBUG
                distAcrossTri = getDistance(getNearestPointOnLine(otherPt, edge1),
                                            otherPt)
                if distAcrossTri < minWidth:
                    minWidth = distAcrossTri
                # ########################### this next bit seems off
                else:
                    minWidth = getDistance(searchTri.getPoint1(), searchTri.getPoint3())

        elif edge1 == searchTri.getEdge23() and searchTri.n23 is None:

            if edge2 == searchTri.getEdge13() and searchTri.n13 is None:
                # both are constrained, so the width of the triangle is the length of the unconstrained edge
                return getDistance(searchTri.getPoint1(), searchTri.getPoint2())

            elif edge2 == searchTri.getEdge12() and searchTri.n12 is None:
                return getDistance(searchTri.getPoint1(), searchTri.getPoint2())
            else:
                # the other edge is not constrained, so the initial width
                # should be either the length of this unconstrained edge
                # or the distance from its non-shared point to the constrained side, whichever is shortest
                if edge2 == searchTri.getEdge12():
                    minWidth = getDistance(searchTri.getPoint1(), searchTri.getPoint2())
                    # we also need to find the end point for the next step
                    if pt != edge2[0]:
                        otherPt = edge2[0]
                    else:
                        otherPt = edge2[1]
                else:  # the other (non-constrained) edge is 13
                    minWidth = getDistance(searchTri.getPoint1(), searchTri.getPoint3())
                    # we also need to find the end point for the next step
                    if pt != edge2[0]:
                        otherPt = edge2[0]
                    else:
                        otherPt = edge2[1]
                # so now get the distance from the end to the other (constrained) edge
                debugEdgeConstrained = edge1  ############################# DEBUG
                distAcrossTri = getDistance(getNearestPointOnLine(otherPt, edge1),
                                            otherPt)
                if distAcrossTri < minWidth:
                    minWidth = distAcrossTri
                # ########################### this next bit seems off
                else:
                    minWidth = getDistance(searchTri.getPoint1(), searchTri.getPoint3())
        else:  # edge1 and edge2 are not constrained
            # Get the width of the shortest of the these two edges
            minWidth = min((edge1[0] - edge1[1]).length(), (edge2[0] - edge2[1]).length())

        # if minWidth < 1:
        #     print "minWidth < 1 pt = ", pt, " || otherPt = ", otherPt, "  || debugConstrained = ", debugEdgeConstrained

        # save these so we don't consider them as nearest points later, else every triangle's width will be 0
        edgePts = [edge1[0], edge1[1]]
        edgePts.extend([edge2[0], edge2[1]])

        # FINALLY search across the third edge for a constrained edge
        # that's closer (to the shared point) than this triangle's vertices
        # print self.adjLst
        # ###################################################
        counter = 0
        # ###################################################
        for t in range(0, len(self.adjLst)):
            # if the edge is constrained, check to see if it narrows the width of this path
            tri = self.adjLst[t]
            # print tri
            if tri.selfInd != searchTri.selfInd:
                if tri.n12 is None:
                    # if the constrained edge, is on the opposite side
                    # from the point shared between the shared edges i.e. for point C check across edge c
                    nearest = getNearestPointOnLine(pt, [tri.tri[0], tri.tri[1]], True)
                    # print tri.selfInd, " 12 is none nearest", nearest
                    if isPointInWedge(nearest, edge1, edge2) and nearest not in edgePts:
                        # and it's in the wedge, check the distance against the current minimum width
                        newW = getDistance(pt, nearest)
                        # print "in wedge newW", newW
                        if newW < minWidth:
                            # ####################################################
                            from panda3d.core import LineSegs
                            # print "pt = ", pt, " || nearest = ", nearest
                            counter += 1
                            linesegs2 = LineSegs("lines" + str(counter))
                            linesegs2.setColor(0, 1, 1, 1)
                            linesegs2.setThickness(5)
                            linesegs2.drawTo(pt)
                            linesegs2.drawTo(nearest)
                            node2 = linesegs2.create(False)
                            nodePath = render.attachNewNode(node2)
                            nodePath.setZ(.25)
                            # ####################################################
                            minWidth = newW
                # do likewise for the other edges
                if tri.n23 is None:
                    nearest = getNearestPointOnLine(pt, [tri.tri[1], tri.tri[2]], True)
                    # print tri.selfInd, " 23  is none nearest", nearest
                    if isPointInWedge(nearest, edge1, edge2) and nearest not in edgePts:
                        newW = getDistance(pt, nearest)
                        # print "in wedge newW", newW
                        if newW < minWidth:
                            # ####################################################
                            from panda3d.core import LineSegs
                            # print "pt = ", pt, " || nearest = ", nearest
                            counter += 1
                            linesegs2 = LineSegs("lines" + str(counter))
                            linesegs2.setColor(0, 1, 1, 1)
                            linesegs2.setThickness(5)
                            linesegs2.drawTo(pt)
                            linesegs2.drawTo(nearest)
                            node2 = linesegs2.create(False)
                            nodePath = render.attachNewNode(node2)
                            nodePath.setZ(.25)
                            # ####################################################
                            minWidth = newW

                if tri.n13 is None:
                    nearest = getNearestPointOnLine(pt, [tri.tri[0], tri.tri[2]], True)
                    # print tri.selfInd, " 13 is none nearest", nearest
                    if isPointInWedge(nearest, edge1, edge2) and nearest not in edgePts:
                        newW = getDistance(pt, nearest)
                        # print "in wedge newW", newW
                        if newW < minWidth:
                            # ####################################################
                            from panda3d.core import LineSegs
                            # print "pt = ", pt, " || nearest = ", nearest
                            counter += 1
                            linesegs2 = LineSegs("lines" + str(counter))
                            linesegs2.setColor(0, 1, 1, 1)
                            linesegs2.setThickness(5)
                            linesegs2.drawTo(pt)
                            linesegs2.drawTo(nearest)
                            node2 = linesegs2.create(False)
                            nodePath = render.attachNewNode(node2)
                            nodePath.setZ(.25)
                            # ####################################################
                            minWidth = newW




        return minWidth


    def __str__(self):
        sr = "TAStar:\nstartPt: " + str(self.start.selfInd) +\
            "\ngoal: " + str(self.goal.selfInd) +\
            "\ncurr: " + str(self.curr) +\
            "\nopen: " + str(self.open) +\
            "\nclosed: " + str(self.closed)
        return sr


if __name__ == '__main__':
    app = TriangulationAStar()