#!/usr/bin/python

from direct.directnotify.DirectNotify import DirectNotify
notify = DirectNotify().newCategory("MaxHeap")


class MaxHeap(list):

    # TODO remove heap list inheritance
    @staticmethod
    def parentIndexOf(i):
        if i == 0:
            raise IndexError("Can not retrieve parent of the heap's top.")
        return i/2 - 1 if i % 2 == 0 else i/2

    @staticmethod
    def lChildOf(i):
        return 2*i + 2

    @staticmethod
    def rChildOf(i):
        return 2*i + 1

    @staticmethod
    def isMaxHeap(lst):
        for i in range(0, len(lst)/2):
            try:
                if lst[i] <= lst[MaxHeap.rChildOf(i)]:
                    return False
            except IndexError:
                pass
            try:
                if lst[i] <= lst[MaxHeap.lChildOf(i)]:
                    return False
            except IndexError:
                pass
        return True
    # @staticmethod
    # def test(heap):
    #     for i in range(len(heap), -)

    def __init__(self, args=[], gtFunc=None, ltFunc=None):
        super(MaxHeap, self).__init__(args)
        self.heapSize = len(args)
        self.gtFunc = gtFunc
        self.ltFunc = ltFunc
        if args:
            notify.warning("buildHeap __init__")
            self.buildHeap()

    def buildHeap(self):
        self.heapSize = len(self)
        for i in range(len(self)/2, -1, -1):
            self._heapify(i)

    def _heapify(self, i):
        while True:
            l = MaxHeap.lChildOf(i)
            r = MaxHeap.rChildOf(i)
            # if there is a custom comparison func us it
            if self.gtFunc is None:
                _gt_ = lambda a, b: self[a] > self[b]
            else:
                _gt_ = lambda a, b: self.gtFunc(self[a], self[b])

            if l <= self.heapSize - 1 and _gt_(l, i):
                largest = l
            else:
                largest = i
            if r <= self.heapSize - 1 and _gt_(r, largest):
                largest = r
            if largest != i:
                tmp = self[i]
                self[i] = self[largest]
                self[largest] = tmp
                i = largest
                # self._heapify(largest)  No Recursion
            else:
                return None

    def heapSort(self):
        if self.heapSize < len(self):
            self.buildHeap()
        # notify.warning("\nHeapSort")
        for j in range(len(self) - 1, 0, -1):
            tmp = self[0]
            # notify.warning("j {0} self[j] {1} self[0] {2}".format(j, self[j], self[0]))
            self[0] = self[j]
            self[j] = tmp
            self.heapSize -= 1
            self._heapify(0)
            # notify.warning("part heap {0}".format(self))

    def _increaseKey(self, i, key):
        isLegal = key < self[i] if self.ltFunc is None else self.ltFunc(key, self[i])
        if isLegal:
            raise ValueError("The new key is smaller than the current key.")
        self[i] = key
        parentIsLess = (lambda i: self[MaxHeap.parentIndexOf(i)] < self[i] if self.ltFunc is None
                        else lambda i: self.ltFunc(self[MaxHeap.parentIndexOf(i)], self[i]))
        while i > 0 and parentIsLess(i):
            tmp = self[i]
            self[i] = self[MaxHeap.parentIndexOf(i)]
            self[MaxHeap.parentIndexOf(i)] = tmp
            i = MaxHeap.parentIndexOf(i)

    def peek(self):
        if self.heapSize > 0:
            return self[0]

    def pop(self):
        if self.heapSize < 1:
            raise IndexError("Heap underflow.")
        top = self[0]
        self[0] = self[self.heapSize - 1]
        super(MaxHeap, self).pop()
        self.heapSize -= 1
        self._heapify(0)
        return top

    def push(self, val):
        if self.heapSize == len(self):
            super(MaxHeap, self).append(float('-inf'))
            self.heapSize += 1
        else:
            self[self.heapSize] = float('-inf')
            self.heapSize += 1
        self._increaseKey(self.heapSize - 1, val)

    def __str__(self):
        return super(MaxHeap, self).__str__()

if __name__ == '__main__':
    l = list(reversed([10, 9, 8, 7, 6, 5, 4, 3, 2, 1]))
    heap = MaxHeap(l)
    notify.warning("l {0}".format(l))
    notify.warning("heap {0}".format(heap))
    heap.heapSort()
    notify.warning("sorted {0}".format(heap))
    notify.warning("size {0}".format(heap.heapSize))
    heap.buildHeap()
    notify.warning("rebuilt {0}".format(heap))
    notify.warning("l {0}".format(l))
    heap2 = MaxHeap()
    for n in l:
        notify.warning("n {0}".format(n))
        heap2.push(n)
    notify.warning("heap2 {0}".format(heap2))
    notify.warning("isMaxHeap {0}".format(MaxHeap.isMaxHeap(heap2)))
    heap2.heapSort()
    notify.warning("after sort isMaxHeap {0}".format(MaxHeap.isMaxHeap(heap2)))
    heap2.buildHeap()
    last = float('inf')
    while heap2:
        v = heap2.pop()
        assert last > v
        notify.warning("pop {0} heap2 {1}".format(v, heap2))
        last = v
