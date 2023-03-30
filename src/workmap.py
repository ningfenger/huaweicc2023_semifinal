# coding=utf-8
from typing import Optional, List, Tuple, Set
import copy
import itertools
from workbench import Workbench

'''
地图类，保存整张地图数据
概念解释：
窄路: 只用未手持物品的机器人才能通过的路, 判定条件，右上三个点都不是障碍(可以是边界)
宽路: 手持物品的机器人也能通过的路, 判定条件, 周围一圈都必须是空地(不是障碍或边界)
狭路: 只能一个机器人通过的路, 判定条件 腐蚀膨胀
'''


class Workmap:
    GROUND = 0
    BOLCK = 100
    PATH = 30  # 算法规划的路径，绘图用
    ROAD = 50  # 窄路
    BROAD_ROAD = 70  # 宽路

    def __init__(self, debug=False) -> None:
        self.roads: List[set] = []  # 狭路集合
        self.map_data: List[str] = []  # 原始地图信息
        # 二值地图 0为空地 100为墙 50 为路径
        self.robots_loc: dict = {}  # k 机器人坐标点 v 机器人ID
        self.workbenchs_loc: dict = {}  # k 工作台坐标点 v 工作台ID
        self.map_gray = [[self.GROUND]*100 for _ in range(100)]
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
                    self.map_gray[i][j] = self.BOLCK
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

    def init_roads(self):
        '''
        识别出窄路和宽路
        '''
        # 先算宽路
        for i in range(1, 99):
            for j in range(1, 99):
                for x, y in itertools.product([-1, 0, 1], repeat=2):
                    if self.map_gray[i+x][j+y] == self.BOLCK:
                        break
                else:
                    self.map_gray[i][j] = self.BROAD_ROAD
        # 再算窄路
        for i in range(100):
            for j in range(100):
                if self.map_gray[i][j] == self.BROAD_ROAD:
                    continue
                for x, y in itertools.product([0, 1], repeat=2):
                    if i+x > 99 or j+y > 99:
                        continue
                    if self.map_gray[i+x][j+y] == self.BOLCK:
                        break
                else:
                    self.map_gray[i][j] = self.ROAD

    def robot2workbench(self):
        '''
        获取每个机器人可以访问的工作台列表, 买的过程由此构建
        '''
        res = [[] for _ in self.robots_loc]
        visited_robot = []  # 如果在遍历过程中找到了其他机器人，说明他两个的地点是可以相互到达的，进而可访问的工作台也是相同的
        visited_workbench = []  # 记录可达的工作台ID
        visited_loc = [[False]*100 for _ in range(100)]  # 记录访问过的节点
        for robot_loc, robot_ID in self.robots_loc.items():
            if res[robot_ID]:  # 已经有内容了，不必再更新
                continue
            dq = [robot_loc]
            while dq:
                i, j = dq.pop()
                for x, y in [(-1, 0), (1, 0), (0, 1), (0, -1)]:
                    n_x, n_y = i+x, j+y
                    if n_x > 99 or n_y > 99 or n_x < 0 or n_y < 0 or visited_loc[n_x][n_y] or self.map_gray[n_x][n_y] not in [self.BROAD_ROAD, self.ROAD]:
                        continue
                    dq.append((n_x, n_y))
                    visited_loc[n_x][n_y] = True
                    if self.map_data[n_x][n_y] == 'A':
                        visited_robot.append(self.robots_loc[(n_x, n_y)])
                    if '1' <= self.map_data[n_x][n_y] <= '7':  # 只关心1-9，因为空手去89没有意义
                        visited_workbench.append(
                            self.workbenchs_loc[(n_x, n_y)])  # 将这个工作台添加到列表
            tmp = visited_workbench[:]  # 拷贝一下
            for idx in visited_robot:
                res[idx] = tmp  # 这里指向同一个列表，节省内存
            visited_robot.clear()
            visited_workbench.clear()
        return res

    def workbench2workbench(self):
        '''
        获取每个工作台可以访问的收购对应商品的工作台列表, 卖的过程由此构建
        '''
        res = [[] for _ in self.workbenchs_loc]
        visited_workbench = []  # 记录可达的工作台ID及类型, 在这同一个列表中的工作台说明可以相互访问
        visited_loc = [[False]*100 for _ in range(100)]  # 记录访问过的节点
        for workbench_loc, workbench_ID in self.workbenchs_loc.items():
            if res[workbench_ID]:  # 已经有内容了，不必再更新
                continue
            if self.map_data[workbench_loc[0]][workbench_loc[1]] in ['8', '9']: # 8 9没有出售目标
                continue
            dq = [workbench_loc]
            while dq:
                i, j = dq.pop()
                for x, y in [(-1, 0), (1, 0), (0, 1), (0, -1)]:
                    n_x, n_y = i+x, j+y
                    # 因为是卖的过程，必须是宽路
                    if n_x > 99 or n_y > 99 or n_x < 0 or n_y < 0 or visited_loc[n_x][n_y] or self.map_gray[n_x][n_y] != self.BROAD_ROAD:
                        continue
                    dq.append((n_x, n_y))
                    visited_loc[n_x][n_y] = True
                    if '1' <= self.map_data[n_x][n_y] <= '9':
                        visited_workbench.append((self.workbenchs_loc[(n_x, n_y)], int(self.map_data[n_x][n_y])))  # 将这个工作台添加到列表
            for wb_ID, wb_type in visited_workbench:
                # 为每一个在集合中的工作台寻找可以访问的目标
                if wb_type in [8,9]: # 8,9 只收不卖
                    continue
                for aim_ID, aim_type in visited_workbench:
                    if wb_type in Workbench.WORKSTAND_IN[aim_type]:
                        res[wb_ID].append(aim_ID)
            visited_workbench.clear()
        return res

    def get_roadID(self, loc: Tuple[int]) -> int:
        '''
        判断某坐标是否属于狭路，如果是返回狭路坐标否则返回-1
        '''
        for roadID, road in enumerate(self.roads):
            if loc in road:
                return roadID
        return -1

    def add_road(self, road_set: Set[Tuple[int]]):
        '''
        创建某条狭路
        '''
        self.roads.append(road_set)

    def draw_map(self):
        pass

    def __draw_map(self):
        '''
        绘制地图
        '''
        self.plt.imshow(self.map_gray)
        self.plt.show()

    def draw_path(self, _):
        pass

    def __draw_path(self, path):
        '''
        绘制一个路径
        '''
        path_map = copy.copy(self.map_gray)
        for x, y in path:
            path_map[x][y] = self.PATH
        self.plt.imshow(path_map)
        self.plt.show()


if __name__ == '__main__':
    map_gray = [[0]*50 for _ in range(50)]
    import matplotlib.pyplot as plt

    map_gray[1] = [100]*50
    map_gray[2] = [50]*50

    plt.imshow(map_gray)
    plt.show(block=False)
