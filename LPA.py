from multiprocessing import set_start_method
import matplotlib.pyplot as plt
from priority_queue import PriorityQueue, Priority
from grid import OccupancyGridMap
import numpy as np
from utils import heuristic, Vertex, Vertices
from typing import Dict, List
import time
OBSTACLE = 255
UNOCCUPIED = 0

class LPA:
    def __init__(self, map: OccupancyGridMap, s_start: (int, int), s_goal: (int, int)):
        """
        :parametr map: mapa przeszkód zapewniana przez gui
        :parametr s_start: punkt początkowy
        :parametr s_goal: punkt końcowy
        """
        self.new_edges_and_old_costs = None

        # incjalizacja algorytmu
        self.s_startprime=s_start
        self.s_start = s_start
        self.s_goal = s_goal
        self.s_last = s_start
        self.U = PriorityQueue()
        self.rhs = np.ones((map.x_dim, map.y_dim)) * np.inf
        self.bckup = self.rhs.copy()
        self.g = self.rhs.copy()
        self.rhs[self.s_start] = 0
        self.sensed_map = map
        self.U.insert(self.s_start, self.calculate_key(self.s_start))

    def resetLPA(self,position):
        self.s_start=position
        self.g=self.bckup.copy()
        self.rhs=self.bckup.copy()
        self.rhs[self.s_start]=0
        self.U = PriorityQueue()
        self.U.insert(self.s_start, self.calculate_key(self.s_start))

    def calculate_key(self, s: (int, int)):
        """
        :parametr s: wierzchołek, dla którego chcemy obliczyć klucz
        :zwraca: klasa priorytetów dwóch kluczy
        """
        k1 = min(self.g[s], self.rhs[s]) + self.c(s, self.s_goal)
        k2 = min(self.g[s], self.rhs[s])
        return Priority(k1, k2)

    def c(self, u: (int, int), v: (int, int)) -> float:
        """
        kalkulacja czasu między węzłami
        :parametr u: od wierzchołka
        :parametr v: do wierzchołka
        :zwraca: odległość euklidesowa inf jeśli przeszkoda na drodze
        """
        if not self.sensed_map.is_unoccupied(u) or not self.sensed_map.is_unoccupied(v):
            return float('inf')
        else:
            return heuristic(u, v)

    def contain(self, u: (int, int)) -> (int, int):
        return u in self.U.vertices_in_heap
        
    def update_vertex(self, u: (int, int)):
        if self.g[u]!=self.rhs[u] and self.contain(u):
            self.U.update(u,self.calculate_key(u))
            
        elif self.g[u]!=self.rhs[u] and not self.contain(u): 
            self.U.insert(u, self.calculate_key(u))
        
        elif self.g[u]==self.rhs[u] and self.contain(u):
            self.U.remove(u)

    def compute_shortest_path(self):
        while self.U.top_key() < self.calculate_key(self.s_goal) or self.rhs[self.s_goal] != self.g[self.s_goal]:
            u = self.U.pop().vertex
            if self.g[u] > self.rhs[u]:
                self.g[u] = self.rhs[u]
                self.U.remove(u)
                succ = self.sensed_map.succ(vertex=u)
                for s in succ:
                    if s!=self.s_start:
                        self.rhs[s]=min(self.rhs[s], self.g[u]+self.c(u,s))
                    self.update_vertex(s)
            else:
                gold=self.g[u]
                self.g[u] = float('inf')
                succ = self.sensed_map.succ(vertex=u)
                succ.append(u)
                for s in succ:
                    if self.rhs[s]==gold+self.c(u,s) or s==u:
                        if s!=self.s_start:
                            pred=self.sensed_map.succ(s) 
                            argmin=[float('inf')]
                            for s_ in pred:
                                argmin.append(self.g[s_]+self.c(s_,s))
                            self.rhs[s]=min(argmin)
                    self.update_vertex(s)
                    
    def rescan(self) -> Vertices:
        new_edges_and_old_costs = self.new_edges_and_old_costs
        self.new_edges_and_old_costs = None
        return new_edges_and_old_costs

    def move_and_replan(self, robot_position: (int, int)):
        t=time.time()
        path = [self.s_goal]
        self.s_start = robot_position
        self.s_last = self.s_start
        s_last = self.s_goal
        s_goal=self.s_goal
        self.compute_shortest_path()
        while self.s_start != s_goal:
            assert (self.g[self.s_goal] != float('inf')), "There is no known path!"
            succ = self.sensed_map.succ(s_goal, avoid_obstacles=False)
            min_s = float('inf')
            arg_min = None
            for s_ in succ:
                if s_ != s_last:
                    temp = self.c(s_goal, s_) + self.g[s_]
                    if temp < min_s:
                        min_s = temp
                        arg_min = s_
                    elif arg_min!=None:
                        if temp==min_s and self.g[s_]<self.g[arg_min]:
                            min_s = temp
                            arg_min = s_
            s_goal = arg_min
            s_last=s_goal
            path.append(s_goal)
            changed_edges_with_old_cost = self.rescan()
            if changed_edges_with_old_cost:
                self.s_last = s_goal
                vertices = changed_edges_with_old_cost.vertices
                for vertex in vertices:
                    v = vertex.pos
                    succ_v = vertex.edges_and_c_old
                    for u, c_old in succ_v.items():
                        c_new = self.c(u, v)
                        if c_old > c_new:
                            if u != self.s_start:
                                self.rhs[u] = min(self.rhs[u], self.c(u, v) + self.g[v])
                        elif self.rhs[u] == c_old + self.g[v]:
                            if u != self.s_start:
                                min_s = float('inf')
                                succ_u = self.sensed_map.succ(vertex=u)
                                for s_ in succ_u:
                                    temp = self.c(u, s_) + self.g[s_]
                                    if min_s > temp:
                                        min_s = temp
                                self.rhs[u] = min_s
                            self.update_vertex(u)          
                self.compute_shortest_path()

        t=time.time()-t
        path.reverse()
        return path, self.g, self.rhs, t




