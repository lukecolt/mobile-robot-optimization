import pygame
import time
from grid import OccupancyGridMap
from typing import List
import os

# Definicja kolorów
BLACK = (0, 0, 0)  # BLACK
UNOCCUPIED = (255, 255, 255)  # WHITE
GOAL = (0, 255, 0)  # GREEN
START = (255, 0, 0)  # RED
OBSTACLE = (77, 77, 51)  # GRAY2
OBSTACLEI = (153,76,0) #BROWN
LOCAL_GRID = (0, 0, 80)  # BLUE

colors = {
    0: UNOCCUPIED,
    1: GOAL,
    16: OBSTACLEI,
    255: OBSTACLE
}


class Animation:
    def __init__(self,
                 title="Path Planning",
                 width=10,
                 height=10,
                 margin=0,
                 x_dim=100,
                 y_dim=50,
                 start=(0, 0),
                 goal=(50, 50),
                 viewing_range=3):
        x = 400
        y = 100
        os.environ['SDL_VIDEO_WINDOW_POS'] = "%d,%d" % (x,y)
        self.active = True
        self.width = width
        self.height = height
        self.margin = margin
        self.x_dim = x_dim
        self.y_dim = y_dim
        self.start = start
        self.current = start
        self.observation = {"pos": None, "type": None}
        self.goal = goal
        self.viewing_range = viewing_range
        pygame.init()
        window_size = [(width + margin) * y_dim + margin,
                       (height + margin) * x_dim + margin]
        self.screen = pygame.display.set_mode((window_size), pygame.SCALED | pygame.RESIZABLE)
        
        self.world = OccupancyGridMap(x_dim=x_dim,
                                      y_dim=y_dim,
                                      exploration_setting='8N')

        pygame.display.set_caption(title)
        pygame.font.SysFont('Comic Sans MS', 36)
        self.done = False
        self.clock = pygame.time.Clock()

    def get_position(self):
        return self.current

    def set_position(self, pos: (int, int)):
        self.current = pos

    def get_map(self):
        return self.world
    def get_goal(self):
        return self.goal

    def set_goal(self, goal: (int, int)):
        self.goal = goal

    def set_start(self, start: (int, int)):
        self.start = start

    def display_path(self, path=None):
        if path is not None:
            for step in path:
                # rysowanie aktualnej ścieżki robota
                step_center = [round(step[1] * (self.width + self.margin) + self.width / 2) + self.margin,
                               round(step[0] * (self.height + self.margin) + self.height / 2) + self.margin]
                #rysowanie pozycji robota jako czerwonego punktu
                pygame.draw.circle(self.screen, START, step_center, round(self.width / 2) - 2)
                
    def run_game(self, path=None):
        if path is None:
            path = []
        grid_cell = None

        for event in pygame.event.get():
            if event.type == pygame.QUIT:  # jeśli użytkownik naciśnie przycisk close
                print("quit")
                self.done = True 

            elif (event.type == pygame.USEREVENT + 0) and self.active:
                # automatyczny ruch
                if len(path)>1:
                    (x, y) = path[1]
                    self.set_position((x, y))
     
        self.screen.fill(BLACK)

        # rysowanie siatki
        for row in range(self.x_dim):
            for column in range(self.y_dim):
                # kolorowanie komórek
                pygame.draw.rect(self.screen, colors[self.world.occupancy_grid_map[row][column]],
                                 [(self.margin + self.width) * column + self.margin,
                                  (self.margin + self.height) * row + self.margin,
                                  self.width,
                                  self.height])

        self.display_path(path=path)
        # wypełnij komórkę końcową kolorem zielonym
        pygame.draw.rect(self.screen, GOAL, [(self.margin + self.width) * self.goal[1] + self.margin,
                                             (self.margin + self.height) * self.goal[0] + self.margin,
                                             self.width,
                                             self.height])

        # rysowanie poruszającego się robota
        robot_center = [round(self.current[1] * (self.width + self.margin) + self.width / 2) + self.margin,
                        round(
                            self.current[0] * (self.height + self.margin) + self.height / 2) + self.margin]

        # rysowanie pozycji robota jako czerwonego okręgu
        pygame.draw.circle(self.screen, START, robot_center, round(self.width / 2) - 2)

        #Rysowanie mapy siatki przeszkód
        pygame.draw.rect(self.screen, LOCAL_GRID,
                         [robot_center[0] - self.viewing_range * (self.height + self.margin),
                          robot_center[1] - self.viewing_range * (self.width + self.margin),
                          2 * self.viewing_range * (self.height + self.margin),
                          2 * self.viewing_range * (self.width + self.margin)], 2)

        # ustawienie cyklu odświeżania
        self.clock.tick(20)
        # aktualizacja danych
        pygame.display.flip()
pygame.quit()