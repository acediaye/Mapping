import pygame
import numpy as np

# colors
BLACK = (0, 0, 0)
RED = (255, 0, 0)
# GREEN = (0, 255, 0)


class Lidar(object):
    def __init__(self, sensor_range, mymap):
        self.sensor_range = sensor_range
        self.mymap = mymap
        self.width, self.height = mymap.get_size()
        self.obstacles = []

    def distance(self, obstacle_pos: tuple, sensor_pos: tuple) -> float:
        dist_x = obstacle_pos[0] - sensor_pos[0]
        dist_y = obstacle_pos[1] - sensor_pos[1]
        return np.sqrt(dist_x**2 + dist_y**2)

    def find_obstacles(self, sensor_pos: tuple) -> list:
        data = []
        x1, y1 = sensor_pos[0], sensor_pos[1]
        for angle in np.linspace(0, 2*np.pi, 100):
            x2, y2 = (x1 + self.sensor_range*np.cos(angle),
                      y1 + self.sensor_range*np.sin(angle))
            for i in range(0, 101):  # 0->100
                u = i / 100  # 0->1
                x = int(x1*(1-u) + x2*(u))  # x1->x2
                y = int(y1*(1-u) + y2*(u))  # y1->y2
                if 0 < x < self.width and 0 < y < self.height:
                    color = self.mymap.get_at((x, y))
                    if (color[0], color[1], color[2]) == (BLACK):
                        distance = self.distance((x, y), (x1, y1))
                        # output = [self.sensor_pos, distance, angle]
                        output = [(x1, y1), distance, angle]
                        data.append(output)
                        print(output)
                        break
        if len(data) > 0:
            return data
        else:
            return None


class World(object):
    def __init__(self, map_size):
        self.point_cloud = []
        self.floor_plan = pygame.image.load('floor_plan.png')
        self.map_height, self.map_width = map_size
        self.mymap = pygame.display.set_mode((self.map_width, self.map_height))
        self.mymap.blit(self.floor_plan, (0, 0))
        self.map_blank = self.mymap.copy()
        self.map_data = self.mymap.copy()

    def obstacle_pos(self, sensor_pos, distance, angle) -> tuple:
        x = int(sensor_pos[0] + distance*np.cos(angle))
        y = int(sensor_pos[1] + distance*np.sin(angle))
        return x, y

    def store_data(self, data: tuple):
        # print(len(self.point_cloud))
        if data is not None:
            for element in data:
                point = self.obstacle_pos(element[0], element[1], element[2])
                if point not in self.point_cloud:
                    self.point_cloud.append(point)

    def show_data(self):
        self.infomap = self.mymap.copy()
        for point in self.point_cloud:
            # self.map_data.set_at((point[0], point[1]), RED)
            self.infomap.set_at((point[0], point[1]), RED)


pygame.init()
pygame.display.set_caption('lidar')
world = World((600, 1200))

world.original = world.mymap.copy()
lidar = Lidar(200, world.original)
world.mymap.fill(BLACK)
# world.map.fill(BLACK)
world.infomap = world.mymap.copy()

run = True
sensor_on = False

while run:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            run = False
        if event.type == pygame.MOUSEBUTTONDOWN:
            sensor_on = True
        elif event.type == pygame.MOUSEBUTTONUP:
            sensor_on = False
    if sensor_on:
        position = pygame.mouse.get_pos()
        sensor_data = lidar.find_obstacles(position)
        world.store_data(sensor_data)
        world.show_data()
    world.mymap.blit(world.infomap, (0, 0))
    # world.mymap.blit(world.map_blank, (0, 0))
    # world.mymap.blit(world.map_data, (0, 0))

    pygame.display.update()
