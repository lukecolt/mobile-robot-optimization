import math
from typing import List


class Vertex:
    def __init__(self, pos: (int, int)):
        self.pos = pos
        self.edges_and_costs = {}

    def add_edge_with_cost(self, succ: (int, int), cost: float):
        if succ != self.pos:
            self.edges_and_costs[succ] = cost

    @property
    def edges_and_c_old(self):
        return self.edges_and_costs


class Vertices:
    def __init__(self):
        self.list = []

    def add_vertex(self, v: Vertex):
        self.list.append(v)

    @property
    def vertices(self):
        return self.list


def heuristic(p: (int, int), q: (int, int)) -> float:
    """
    Funkcja pomocnicza do obliczania odległości między dwoma punktami.
    :parametr p: (x,y)
    :paramert q: (x,y)
    :zwraca: dystans metryka manhattan 
    """
    return math.sqrt((p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2)


def get_movements_4n(x: int, y: int) -> List:
    """
    uzyskanie kroków w 4 dozwolonych kierunkach (4 kierunki świata)
    :zwraca: lista ruchów wraz z kosztem [(dx, dy, koszt)]
    """
    return [(x + 1, y + 0),
            (x + 0, y + 1),
            (x - 1, y + 0),
            (x + 0, y - 1)]


def get_movements_8n(x: int, y: int) -> List:
    """
    uzyskanie kroków w 8 dozwolonych kierunkach (oprócz 4 kierunków świata, także po skosie)
    :zwraca: lista ruchów wraz z kosztem [(dx, dy, koszt]
    """
    return [(x + 1, y + 0),
            (x + 0, y + 1),
            (x - 1, y + 0),
            (x + 0, y - 1),
            (x + 1, y + 1),
            (x - 1, y + 1),
            (x - 1, y - 1),
            (x + 1, y - 1)]
