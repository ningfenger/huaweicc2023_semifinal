# coding=utf-8
from typing import Optional, List, Tuple, Set
import copy
import itertools
from workbench import Workbench
from tools import *
import numpy as np
import time
from functools import lru_cache

'''
地图类，保存整张地图数据
概念解释：
窄路: 只用未手持物品的机器人才能通过的路, 判定条件，右上三个点都不是障碍(可以是边界)
宽路: 手持物品的机器人也能通过的路, 判定条件, 周围一圈都必须是空地(不是障碍或边界)
狭路: 只能一个机器人通过的路, 判定条件 腐蚀膨胀
'''


class Workmap:
    BLOCK = 0  # 障碍
    GROUND = 1  # 空地
    ROAD = 2  # 窄路, 临近的四个空地的左下角，因此如果经过这类点请从右上角走
    BROAD_ROAD = 3  # 宽路 瞄着中间走就行
    PATH = 10  # 算法规划的路径，绘图用

    # TURNS = [(-1, 0), (1, 0), (0, 1), (0, -1)]
    TURNS = list(itertools.product([-1, 0, 1], repeat=2))  # 找路方向，原则: 尽量减少拐弯
    TURNS.remove((0, 0))

    def __init__(self, debug=False) -> None:
        self.roads: List[set] = []  # 狭路集合
        self.map_data: List[str] = []  # 原始地图信息
        # 二值地图 0为空地 100为墙 50 为路径
        self.robots_loc: dict = {}  # k 机器人坐标点 v 机器人ID
        self.workbenchs_loc: dict = {}  # k 工作台坐标点 v 工作台ID
        self.map_gray = [[self.GROUND] * 100 for _ in range(100)]
        if debug:
            import matplotlib.pyplot as plt
            self.plt = plt
            self.draw_map = self.__draw_map
            self.draw_path = self.__draw_path
        self.buy_map = {}  # 空手时到每个工作台的路径
        self.sell_map = {}  # 手持物品时到某个工作台的路径

    @lru_cache
    def loc_int2float(self, i: int, j: int, rode=False):
        '''
        地图离散坐标转实际连续坐标
        rode: 标识是否是窄路，窄路按原先右上角
        '''
        x = 0.5 * j + 0.25
        y = (100 - i) * 0.5 - 0.25
        if rode:
            x += 0.25
            y -= 0.25
        return x, y

    def loc_float2int(self, x, y):
        '''
        地图实际连续坐标转离散坐标
        '''
        i = round(100 - (y + 0.25) * 2)
        j = round((x - 0.25) * 2)
        return i, j

    def read_map(self):
        '''
        从标准输入读取地图
        '''
        for i in range(100):
            self.map_data.append(input())
            for j in range(100):
                if self.map_data[i][j] == '#':  # 障碍
                    self.map_gray[i][j] = self.BLOCK
                elif self.map_data[i][j] == 'A':  # 机器人
                    x, y = self.loc_int2float(i, j)
                    self.robots_loc[(i, j)] = len(self.robots_loc)
                    yield 'A', (x, y)
                elif '1' <= self.map_data[i][j] <= '9':  # 工作台
                    x, y = self.loc_int2float(i, j)
                    self.workbenchs_loc[(i, j)] = len(self.workbenchs_loc)
                    yield self.map_data[i][j], (x, y)
        input()  # 读入ok
        # if OK_str == 'OK':
        #     raise Exception('OK')

    def init_roads(self):
        '''
        识别出窄路和宽路
        '''
        # 先算宽路
        for i in range(1, 99):
            for j in range(1, 99):
                for x, y in itertools.product([-1, 0, 1], repeat=2):
                    if self.map_gray[i + x][j + y] == self.BLOCK:
                        break
                else:
                    self.map_gray[i][j] = self.BROAD_ROAD
        # 再算窄路
        for i in range(100):
            for j in range(100):
                if self.map_gray[i][j] == self.BROAD_ROAD:
                    continue
                if '1' <= self.map_data[i][j] <= '3':  # 工作台也被认为可达
                    self.map_gray[i][j] = self.ROAD
                    continue
                for x, y in itertools.product([0, 1], repeat=2):
                    if i + x > 99 or j + y > 99:
                        continue
                    if self.map_gray[i + x][j + y] == self.BLOCK:
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
        visited_loc = [[False] * 100 for _ in range(100)]  # 记录访问过的节点
        for robot_loc, robot_ID in self.robots_loc.items():
            if res[robot_ID]:  # 已经有内容了，不必再更新
                continue
            dq = [robot_loc]
            while dq:
                i, j = dq.pop()
                for x, y in self.TURNS:
                    n_x, n_y = i + x, j + y
                    if n_x > 99 or n_y > 99 or n_x < 0 or n_y < 0 or visited_loc[n_x][n_y] or self.map_gray[n_x][
                        n_y] < self.ROAD:
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
        visited_loc = [[False] * 100 for _ in range(100)]  # 记录访问过的节点
        for workbench_loc, workbench_ID in self.workbenchs_loc.items():
            if res[workbench_ID]:  # 已经有内容了，不必再更新
                continue
            if self.map_data[workbench_loc[0]][workbench_loc[1]] in ['8', '9']:  # 8 9没有出售目标
                continue
            dq = [workbench_loc]
            while dq:
                i, j = dq.pop()
                for x, y in self.TURNS:
                    n_x, n_y = i + x, j + y
                    # 因为是卖的过程，必须是宽路
                    if n_x > 99 or n_y > 99 or n_x < 0 or n_y < 0 or visited_loc[n_x][n_y] or self.map_gray[n_x][
                        n_y] != self.BROAD_ROAD:
                        continue
                    dq.append((n_x, n_y))
                    visited_loc[n_x][n_y] = True
                    if '1' <= self.map_data[n_x][n_y] <= '9':
                        visited_workbench.append(
                            (self.workbenchs_loc[(n_x, n_y)], int(self.map_data[n_x][n_y])))  # 将这个工作台添加到列表
            for wb_ID, wb_type in visited_workbench:
                # 为每一个在集合中的工作台寻找可以访问的目标
                if wb_type in [8, 9]:  # 8,9 只收不卖
                    continue
                for aim_ID, aim_type in visited_workbench:
                    if wb_type in Workbench.WORKSTAND_IN[aim_type]:
                        res[wb_ID].append(aim_ID)
            visited_workbench.clear()
        return res

    # def gen_a_path_old(self, workbench_ID, workbench_loc, broad_road=False):
    #     '''
    #     生成一个工作台到其他节点的路径
    #     workbench_ID: 工作台ID
    #     workbench_loc: 当前节点坐标
    #     broad_road: 是否只能走宽路
    #     '''
    #     un_reach_node = deque()  # 能达(是路)而未达(没有在最短距离内到达)的点的集合
    #     if broad_road:
    #         target_map = self.sell_map[workbench_ID]
    #         low_value = self.BROAD_ROAD
    #     else:
    #         target_map = self.buy_map[workbench_ID]
    #         low_value = self.ROAD
    #     loc_x, loc_y = workbench_loc
    #     target_map[loc_x][loc_y] = workbench_loc

    #     for k in range(1, max(loc_x-1, loc_y-1, 100-loc_x, 100-loc_y)):
    #         turns = [(k, i) for i in range(-k, k+1)] + [(i, k)
    #                                                     for i in range(-k+1, k)]
    #         turns += [(-i, -j) for i, j in turns]
    #         for x, y in turns:
    #             node_x, node_y = loc_x + x, loc_y + y
    #             if node_x < 0 or node_y < 0 or node_x >= 100 or node_y >= 100 or self.map_gray[node_x][node_y] < low_value:
    #                 continue

    #             # 根据所处位置决定目标方向
    #             if x == k:
    #                 test_turns = [(-1, -1), (-1, 0), (-1, 1)]
    #             elif x == -k:
    #                 test_turns = [(1, -1), (1, 0), (1, 1)]
    #             elif y == k:
    #                 test_turns = [(1, -1), (0, -1), (-1, -1)]
    #             else:
    #                 test_turns = [(1, 1), (0, 1), (-1, 1)]
    #             aim_loc = None
    #             min_angle_diff = 4  # 减少转弯
    #             for i, j in test_turns:
    #                 test_x, test_y = node_x + i, node_y+j
    #                 if test_x < 0 or test_y < 0 or test_x >= 100 or test_y >= 100 or not target_map[test_x][test_y]:
    #                     continue
    #                 last_x, last_y = target_map[test_x][test_y]
    #                 angle_diff = abs(last_x+node_x-2*test_x) + \
    #                     abs(last_y+node_y-2*test_y)
    #                 if angle_diff < min_angle_diff:
    #                     min_angle_diff = angle_diff
    #                     aim_loc = (test_x, test_y)
    #             if aim_loc:
    #                 target_map[node_x][node_y] = aim_loc
    #             else:
    #                 un_reach_node.append((node_x, node_y))
    #     while un_reach_node:  # 最后集中处理未到达点
    #         tmp_length = len(un_reach_node)
    #         for _ in range(tmp_length):
    #             node_x, node_y = un_reach_node.pop()
    #             aim_loc = None
    #             min_angle_diff = 4  # 减少转弯
    #             for i, j in self.TURNS:
    #                 test_x, test_y = node_x + i, node_y+j
    #                 if test_x < 0 or test_y < 0 or test_x >= 100 or test_y >= 100 or not target_map[test_x][test_y]:
    #                     continue
    #                 last_x, last_y = target_map[test_x][test_y]
    #                 angle_diff = abs(last_x+node_x-2*test_x) + \
    #                     abs(last_y+node_y-2*test_y)
    #                 if angle_diff < min_angle_diff:
    #                     min_angle_diff = angle_diff
    #                     aim_loc = (test_x, test_y)
    #             if aim_loc:
    #                 target_map[node_x][node_y] = aim_loc
    #             else:
    #                 un_reach_node.appendleft((node_x, node_y))
    #         if len(un_reach_node) == tmp_length: # 剩下的这些节点已经不可能到了
    #             break

    def gen_a_path(self, workbench_ID, workbench_loc, broad_road=False):
        '''
        生成一个工作台到其他节点的路径，基于迪杰斯特拉优化
        workbench_ID: 工作台ID
        workbench_loc: 当前节点坐标
        broad_road: 是否只能走宽路
        '''
        reach = []  # 上一轮到达的节点集合
        if broad_road:
            target_map = self.sell_map[workbench_ID]
            low_value = self.BROAD_ROAD
        else:
            target_map = self.buy_map[workbench_ID]
            low_value = self.ROAD
        node_x, node_y = workbench_loc
        target_map[node_x][node_y] = workbench_loc
        # node_x/y 当前节点 next_x/y 将要加入的节点 last_x/y 当前节点的父节点
        for x, y in self.TURNS:
            next_x, next_y = node_x + x, node_y + y
            if next_x < 0 or next_y < 0 or next_x >= 100 or next_y >= 100 or self.map_gray[next_x][next_y] < low_value:
                continue
            reach.append((next_x, next_y))  # 将周围节点标记为可到达
            target_map[next_x][next_y] = (node_x, node_y)
        while reach:
            tmp_reach = {}  # 暂存新一轮可到达点, k可到达点坐标, v 角度差
            for node_x, node_y in reach:
                last_x, last_y = target_map[node_x][node_y]
                for i, j in self.TURNS:
                    next_x, next_y = node_x + i, node_y + j
                    if next_x < 0 or next_y < 0 or next_x >= 100 or next_y >= 100 or self.map_gray[next_x][
                        next_y] < low_value:
                        continue
                    if (next_x, next_y) not in tmp_reach:
                        if target_map[next_x][next_y]:  # 已被访问过说明是已经添加到树中的节点
                            continue
                        else:
                            angle_diff = abs(last_x + node_x - 2 * next_x) + \
                                         abs(last_y + node_y - 2 * next_y)
                            tmp_reach[(next_x, next_y)] = angle_diff
                            target_map[next_x][next_y] = (node_x, node_y)
                    else:
                        angle_diff = abs(last_x + node_x - 2 * next_x) + \
                                     abs(last_y + node_y - 2 * next_y)
                        if angle_diff < tmp_reach[(next_x, next_y)]:
                            tmp_reach[(next_x, next_y)] = angle_diff
                            target_map[next_x][next_y] = (node_x, node_y)
            reach = tmp_reach.keys()

    def gen_paths(self):
        '''
        基于查并集思想，优先选择与自己方向一致的节点
        以工作台为中心拓展
        '''
        base_map = [[None for _ in range(100)] for _ in range(100)]
        for loc, idx in self.workbenchs_loc.items():
            x, y = loc
            flag1, flag2 = True, True
            workbench_type = self.map_data[x][y]
            if '8' <= workbench_type <= '9':
                flag1 = False  # 空手到8、9没有意义
            elif '1' <= workbench_type <= '3':
                flag2 = False  # 拿着东西到1、2、3没有意义
            if flag1:
                self.buy_map[idx] = copy.deepcopy(base_map)
                self.gen_a_path(idx, loc, False)
            if flag2:
                self.sell_map[idx] = copy.deepcopy(base_map)
                self.gen_a_path(idx, loc, True)

    def get_float_path(self, float_loc, workbench_ID, broad_road=False):
        '''
        获取浮点型的路径
        参数同get_path
        返回路径(换算好的float点的集合)
        '''
        path = self.get_path(float_loc, workbench_ID, broad_road)
        for i in range(len(path)):
            path[i] = self.loc_int2float(*path[i])
        return path

    def get_path(self, float_loc, workbench_ID, broad_road=False):
        '''
        获取某点到某工作台的路径
        float_loc: 当前位置的浮点坐标
        workbench_ID: 工作台ID
        broad_road: 是否只能走宽路   
        返回路径(int点的集合)
        '''
        if broad_road:  # 决定要查哪个地图
            target_map = self.sell_map[workbench_ID]
        else:
            target_map = self.buy_map[workbench_ID]
        node_x, node_y = self.loc_float2int(*float_loc)
        if not target_map[node_x][node_y]:
            for x, y in self.TURNS:
                test_x, test_y = node_x + x, node_y + y
                if target_map[test_x][test_y]:
                    node_x, node_y = test_x, test_y
                    break
            else:  # 当前点和临近点都无法到达目的地
                return []
        path = []
        x, y = node_x, node_y
        while 1:
            path.append((x, y))
            if (x, y) == target_map[x][y]:
                break
            x, y = target_map[x][y]
        return path

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
        path_map = copy.deepcopy(self.map_gray)
        for x, y in path:
            path_map[x][y] = self.PATH
        self.plt.imshow(path_map)
        self.plt.show()

    def read_map_directly(self, map_path):
        with open(map_path) as map:
            lines = map.readlines()
            for i, line in enumerate(lines):
                self.map_data.append(line)
                for j in range(100):
                    if line[j] == '#':  # 障碍
                        self.map_gray[i][j] = self.BLOCK


if __name__ == '__main__':
    # map_gray = [[0]*50 for _ in range(50)]
    # import matplotlib.pyplot as plt
    #
    # map_gray[1] = [100]*50
    # map_gray[2] = [50]*50
    #
    # plt.imshow(map_gray)
    # plt.show(block=False)
    import matplotlib.pyplot as plt

    work_map = Workmap(debug=True)
    work_map.read_map_directly("../maps/4.txt")
    astar = AStar(work_map)
    start = (25.25, 49.75)
    goal = (18.75, 49.75)
    T1 = time.time()
    path = np.array(astar.get_path_cor(start, goal, False))
    T2 = time.time()
    print(T2 - T1)
    img = work_map.map_gray
    img = np.array(img).astype('uint8')
    plt.imshow(img)
    plt.title("img")
    plt.show()

    fig = plt.figure()
    plt.plot(path[:, 0], path[:, 1])
    plt.show()
    pass
