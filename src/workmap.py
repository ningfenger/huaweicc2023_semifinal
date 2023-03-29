# coding=utf-8
from typing import Optional, List, Tuple, Set
import copy
'''
地图类，保存整张地图数据
'''


class Workmap:
    GROUND = 0
    BOLCK = 100
    ROAD = 50

    def __init__(self, debug=False) -> None:
        self.roads: List[set] = []  # 窄路集合
        self.map_data: List[str] = []  # 原始地图信息
        # 二值地图 0为空地 100为墙 50 为路径
        self.robots_loc: dict = []  # k 机器人坐标点 v 机器人ID
        self.workbenchs_loc: dict = []  # k 工作台坐标点 v 工作台ID
        self.__map_gray = [[self.GROUND]*100 for _ in range(100)]
        if debug:
            import matplotlib.pyplot as plt
            self.plt = plt
            self.draw_map = self.__draw_map
            self.draw_path = self.__draw_path

    def read_map(self):
        '''
        从标准输入读取地图
        '''
        for i in range(100):
            self.map_data.append(input())
            for j in range(100):
                if self.map_data[i][j] == '#':  # 障碍
                    self.__map_gray[i][j] = self.BOLCK
                elif self.map_data[i][j] == 'A':  # 机器人
                    x = i * 0.5 + 0.25
                    y = j * 0.5 + 0.25
                    self.robots_loc[(i, j)] = len(self.robots_loc)
                    yield 'A', (x, y)
                elif '1' <= self.map_data[i][j] <= '9':  # 工作台
                    x = i * 0.5 + 0.25
                    y = j * 0.5 + 0.25
                    self.workbenchs_loc[(i, j)] = len(self.workbenchs_loc)
                    yield self.map_data[i][j], (x, y)
        input()  # 读入ok
    def get_workbench4robot(self):
        '''
        获取每个机器人可以访问的工作台列表
        '''
        res = [[] for _ in range(4)]
        visited_robot = set() # 如果在遍历过程中找到了其他机器人，说明他两个的地点是可以相互到达的，进而可访问的工作台也是相同的
        # 机器人的最大直径是0.53，因此需要有3*3个格子才能容纳之
    
    
    def get_roadID(self, loc: Tuple[int]) -> int:
        '''
        判断某坐标是否属于窄路，如果是返回窄路坐标否则返回-1
        '''
        for roadID, road in enumerate(self.roads):
            if loc in road:
                return roadID
        return -1

    def add_road(self, road_set: Set[Tuple[int]]):
        '''
        创建某条窄路
        '''
        self.roads.append(road_set)

    def draw_map(self):
        pass

    def __draw_map(self):
        '''
        绘制地图
        '''
        self.plt.imshow(self.__map_gray)
        self.plt.show()

    def draw_path(self, _):
        pass

    def __draw_path(self, path):
        '''
        绘制一个路径
        '''
        path_map = copy.copy(self.__map_gray)
        for x, y in path:
            path_map[x][y] = self.ROAD
        self.plt.imshow(path_map)
        self.plt.show()


if __name__ == '__main__':
    map_gray = [[0]*50 for _ in range(50)]
    import matplotlib.pyplot as plt

    map_gray[1] = [100]*50
    map_gray[2] = [50]*50

    plt.imshow(map_gray)
    plt.show(block=False)
