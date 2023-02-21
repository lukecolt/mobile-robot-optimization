import numpy as np
from utils import get_movements_4n, get_movements_8n, heuristic, Vertices, Vertex
from typing import Dict, List

OBSTACLE = 255
UNOCCUPIED = 0
OBSTACLEI = 16

class OccupancyGridMap:
    def __init__(self, x_dim, y_dim, exploration_setting='8N'):
        """
        incjalizacja wartości początkowych dla mapy przeszkód
        :parametr x_dim: wymiar w kierunku x
        :parametr y_dim: wymiar w kierunku y
        :parametr exploration_setting: wybór sposobu eksploracji mapy
        """
        self.x_dim = x_dim
        self.y_dim = y_dim
        self.map_extents = (x_dim, y_dim)
        # inicjalizacja mapy przeszkód
        self.occupancy_grid_map = np.zeros(self.map_extents, dtype=np.uint8)
        self.exploration_setting = exploration_setting

    def get_map(self):
        """
        :zwraca: aktualną mapę przeszkód
        """
        return self.occupancy_grid_map

    def set_map(self, new_ogrid):
        """
        :parametr new_ogrid:
        :zwraca: None
        """
        self.occupancy_grid_map = new_ogrid

    def is_unoccupied(self, pos: (int, int)) -> bool:
        """
        :parametr pos: dane komórki, którą chcemy sprawdzić
        :zwraca: True gdy komórka jest przeszkodą, w przeciwnym wypadku False 
        """
        (x, y) = (round(pos[0]), round(pos[1])) 
        (row, col) = (x, y)

        return self.occupancy_grid_map[row][col] == UNOCCUPIED

    def in_bounds(self, cell: (int, int)) -> bool:
        """
        Sprawdza czy komórka znajduje się w graniach mapy
        :parametr cell: pozycja komórki (x,y)
        :zwraca: True jeśli jest w granicach, False w przeciwnym wypadku
        """
        (x, y) = cell
        return 0 <= x < self.x_dim and 0 <= y < self.y_dim

    def filter(self, neighbors: List, avoid_obstacles: bool):
        """
        :parametr neighbors: lista potencjalnych sąsiadów przed filtrowaniem
        :parametr avoid_obstacles: jeśli True, odfiltrowuje komórki z przeszkodami na liście
        :zwraca:
        """
        if avoid_obstacles:
            return [node for node in neighbors if self.in_bounds(node) and self.is_unoccupied(node)]
        return [node for node in neighbors if self.in_bounds(node)]

    def succ(self, vertex: (int, int), avoid_obstacles: bool = False) -> list:
        """
        :parametr avoid_obstacles:
        :parametr vertex: wierzchołek, dla którego szuka się kolejnw wierzchołki
        :zwraca:
        """
        (x, y) = vertex

        if self.exploration_setting == '4N':  
            movements = get_movements_4n(x=x, y=y)
        else:
            movements = get_movements_8n(x=x, y=y)
        if (x + y) % 2 == 0: movements.reverse()

        filtered_movements = self.filter(neighbors=movements, avoid_obstacles=avoid_obstacles)
        return list(filtered_movements)

    def set_obstacle(self, pos: (int, int)):
        """
        :parametr pos: pozycja komórki, dla której ustawiamy przeszkodę niewidoczną
        :zwraca: 
        """
        (x, y) = (round(pos[0]), round(pos[1])) 
        (row, col) = (x, y)
        self.occupancy_grid_map[row, col] = OBSTACLE
    
    def set_obstaclei(self, pos: (int, int)):
        """
        :parametr pos: pozycja komórki, dla której ustawiamy przeszkodę widoczną
        :zwraca:
        """
        (x, y) = (round(pos[0]), round(pos[1]))  
        (row, col) = (x, y)
        self.occupancy_grid_map[row, col] = OBSTACLEI


    def local_observation(self, global_position: (int, int), view_range: int = 2) -> Dict:
        """
        :parametr global_position: pozycja robota na globalnej mapie
        :parametr view_range: okno zasięgu działania obserwacji
        :zwraca: typ dictionary - new observations
        """
        (px, py) = global_position
        nodes = [(x, y) for x in range(px - view_range, px + view_range + 1)
                 for y in range(py - view_range, py + view_range + 1)
                 if self.in_bounds((x, y))]
        return {node: UNOCCUPIED if self.is_unoccupied(pos=node) else OBSTACLE for node in nodes}


class SLAM:
    def __init__(self, map: OccupancyGridMap, view_range: int):
        self.ground_truth_map = map
        self.slam_map = map
        self.view_range = view_range

    def set_ground_truth_map(self, gt_map: OccupancyGridMap):
        self.ground_truth_map = gt_map

    def c(self, u: (int, int), v: (int, int)) -> float:
        """
        obliczanie kosztu między węzłami
        :parametr u: from vertex
        :parametr v: to vertex
        :zwraca: odległość euklidesowa, inf jeśli przeszkoda na drodze
        """
        if not self.slam_map.is_unoccupied(u) or not self.slam_map.is_unoccupied(v):
            return float('inf')
        else:
            return heuristic(u, v)

    def rescan(self, global_position: (int, int)):

        local_observation = self.ground_truth_map.local_observation(global_position=global_position,
                                                                    view_range=self.view_range)

        vertices = self.update_changed_edge_costs(local_grid=local_observation)
        return vertices, self.slam_map

    def update_changed_edge_costs(self, local_grid: Dict) -> Vertices:
        vertices = Vertices()
        for node, value in local_grid.items():
            if value == OBSTACLE:
                if self.slam_map.is_unoccupied(node):
                    v = Vertex(pos=node)
                    succ = self.slam_map.succ(node)
                    for u in succ:
                        v.add_edge_with_cost(succ=u, cost=self.c(u, v.pos))
                    vertices.add_vertex(v)
                    self.slam_map.set_obstacle(node)
        return vertices
