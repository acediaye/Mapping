import pygame
import numpy as np

# colors
BLACK = (0, 0, 0)
RED = (255, 0, 0)


class Lidar(object):
    def __init__(self, sensor_range: int, mymap):
        """
        sensor_range: int
            range in terms of pixels
        mymap: image
            world image. code detects black pixels as obstacles
        """
        self.sensor_range = sensor_range
        self.mymap = mymap
        self.width, self.height = mymap.get_size()
        self.obstacles = []

    def distance(self, obstacle_pos: tuple, sensor_pos: tuple) -> float:
        """
        obstacle position in terms of x, y
        sensor position in terms of x, y
        returns distance in terms of float
        """
        dist_x = obstacle_pos[0] - sensor_pos[0]
        dist_y = obstacle_pos[1] - sensor_pos[1]
        return np.sqrt(dist_x**2 + dist_y**2)

    def find_obstacles(self, sensor_pos: tuple) -> list:
        """
        sensor_pos: tuple
            x, y
        return: list
            sensor: int
            distance: float
            angle: float
            list of starting sensor_pos, distance and angle to obstacle
        """
        data = []
        x1, y1 = sensor_pos[0], sensor_pos[1]
        for angle in np.linspace(0, 2*np.pi, 100):
            x2, y2 = (x1 + self.sensor_range*np.cos(angle),
                      y1 + self.sensor_range*np.sin(angle))
            # scaling from initial point to sensor range point
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
        self.image = pygame.image.load('pics/floor_plan.png')
        self.map_height, self.map_width = map_size
        self.mymap = pygame.display.set_mode((self.map_width, self.map_height))
        self.mymap.blit(self.image, (0, 0))  # add image on blank

        self.floor_plan = self.mymap.copy()  # copy to use
        self.mymap.fill(BLACK)  # blacked out main image
        self.point_map = self.mymap.copy()  # points drawn on blacked image

    def obstacle_pos(self, sensor_pos, distance, angle) -> tuple:
        """
        with data from lidar, calculate the x, y position of obstacles
        obstacles in terms of int for pixels
        """
        x = int(sensor_pos[0] + distance*np.cos(angle))
        y = int(sensor_pos[1] + distance*np.sin(angle))
        return x, y

    def store_data(self, data: tuple):
        """
        save x, y position of obstacles in terms of int
        """
        # print(len(self.point_cloud))
        if data is not None:
            for element in data:
                point = self.obstacle_pos(element[0], element[1], element[2])
                if point not in self.point_cloud:
                    self.point_cloud.append(point)

    def show_data(self):
        """
        plot all x, y points as pixels
        """
        for point in self.point_cloud:
            self.point_map.set_at((point[0], point[1]), RED)


pygame.init()
pygame.display.set_caption('lidar')
world = World((600, 1200))  # size need to match png image
lidar = Lidar(200, world.floor_plan)  # code detects black as obstacles

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
    world.mymap.blit(world.point_map, (0, 0))
    pygame.display.update()
