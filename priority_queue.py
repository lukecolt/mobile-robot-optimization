class Priority:
    """
    działanie z kluczami
    """

    def __init__(self, k1, k2):
        """
        :parametr k1, k2: klucz
        """
        self.k1 = k1
        self.k2 = k2

    def __lt__(self, other):
        """
        porównanie: mniej niż
        :parametr other: porównywane klucze
        :zwraca: kolejność
        """
        return self.k1 < other.k1 or (self.k1 == other.k1 and self.k2 < other.k2)

    def __le__(self, other):
        """
        porównanie: mniej lub równe niż
        :parametr other: porównywane klucze
        :zwraca: kolejność
        """
        return self.k1 < other.k1 or (self.k1 == other.k1 and self.k2 <= other.k2)


class PriorityNode:
    """
    działanie z kolejnością wierzchołków
    """

    def __init__(self, priority, vertex):
        """
        :parametr priority: nadanie priorytetu
        :parametr vertex:
        """
        self.priority = priority
        self.vertex = vertex

    def __le__(self, other):
        """
        :parametr other: porównywany węzeł
        :zwraca: kolejność
        """
        return self.priority <= other.priority

    def __lt__(self, other):
        """
        :parametr other: porównywany węzeł
        :zwraca: kolejność
        """
        return self.priority < other.priority


class PriorityQueue:
    def __init__(self):
        self.heap = []
        self.vertices_in_heap = []

    def top(self):
        return self.heap[0].vertex

    def top_key(self):
        if len(self.heap) == 0: return Priority(float('inf'), float('inf'))
        return self.heap[0].priority

    """!!!PONIŻSZY KOD ZOSTAŁ SKOPIOWANY I ZMODYFIKOWANY!!! Źródło: Lib/heapq.py"""
    def pop(self):

        lastelt = self.heap.pop()  # błąd gdy sterta jest pusta
        if self.heap:
            returnitem = self.heap[0]
            self.heap[0] = lastelt
            self._siftup(0)
        else:
            returnitem = lastelt
        self.vertices_in_heap.remove(returnitem.vertex)
        return returnitem

    def insert(self, vertex, priority):
        item = PriorityNode(priority, vertex)
        self.vertices_in_heap.append(vertex)
        self.heap.append(item)
        self._siftdown(0, len(self.heap) - 1)

    def remove(self, vertex):
        #przesunięty vertex do ifa
        for index, priority_node in enumerate(self.heap):
            if priority_node.vertex == vertex:
                self.heap[index] = self.heap[len(self.heap) - 1]
                self.heap.remove(self.heap[len(self.heap) - 1])
                self.vertices_in_heap.remove(vertex)
                break
        self.build_heap()

    def update(self, vertex, priority):
        for index, priority_node in enumerate(self.heap):
            if priority_node.vertex == vertex:
                self.heap[index].priority = priority
                break
        self.build_heap()


    def build_heap(self):
        """Przekształć listę na stertę"""
        n = len(self.heap)
        # Przekształcenie typu bottom-up.
        for i in reversed(range(n // 2)):
            self._siftup(i)

    def _siftdown(self, startpos, pos):
        newitem = self.heap[pos]
        while pos > startpos:
            parentpos = (pos - 1) >> 1
            parent = self.heap[parentpos]
            if newitem < parent:
                self.heap[pos] = parent
                pos = parentpos
                continue
            break
        self.heap[pos] = newitem

    def _siftup(self, pos):
        endpos = len(self.heap)
        startpos = pos
        newitem = self.heap[pos]
        childpos = 2 * pos + 1  
        while childpos < endpos:
            rightpos = childpos + 1
            if rightpos < endpos and not self.heap[childpos] < self.heap[rightpos]:
                childpos = rightpos
            self.heap[pos] = self.heap[childpos]
            pos = childpos
            childpos = 2 * pos + 1
        self.heap[pos] = newitem
        self._siftdown(startpos, pos)
