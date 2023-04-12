# coding=utf-8
from typing import Optional, List, Tuple, Set
import copy
import itertools
from workbench import Workbench
from tools import *
import numpy as np
import time
from functools import lru_cache
from collections import deque
import sys

'''
地图类，保存整张地图数据
概念解释：
窄路: 只用未手持物品的机器人才能通过的路, 判定条件，右上三个点都不是障碍(可以是边界)
宽路: 手持物品的机器人也能通过的路, 判定条件, 周围一圈都必须是空地(不是障碍或边界)
广路: 可以冲刺
'''


class Workmap:
    BLOCK = 0  # 障碍
    GROUND = 1  # 空地
    ROAD = 2  # 窄路, 临近的四个空地的左下角，因此如果经过这类点请从右上角走
    BROAD_ROAD = 3  # 宽路 瞄着中间走就行
    SUPER_BROAD_ROAD = 4  # 广路 可以冲刺
    PATH = 5  # 算法规划的路径，绘图用

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
        self.unreanchble_warkbench = set()  # 记录不能正常使用的工作台
        self.broad_shifting = {}  # 有些特殊宽路的偏移量

    @lru_cache(None)
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
        elif (i, j) in self.broad_shifting:
            # 特殊宽路
            shifting = self.broad_shifting[(i, j)]
            x += shifting[0]
            y += shifting[1]

        return x, y

    def loc_float2int(self, x, y):
        '''
        地图实际连续坐标转离散坐标
        '''
        i = round(100 - (y + 0.25) * 2)
        j = round((x - 0.25) * 2)
        return i, j

    def dis_loc2path(self, loc, path):
        '''
        获取某个点到某路径的最短距离
        '''
        dists = np.sqrt(np.sum((np.array(path) - np.array(loc)) ** 2, axis=1))
        nearest_row = np.argmin(dists)
        return dists[nearest_row]

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

    def init_roads(self):
        '''
        识别出窄路和宽路
        '''
        # 先算宽路
        for i in range(1, 99):
            for j in range(1, 99):
                '''
                ...
                ...
                ...
                '''
                tmp_blocks = []
                for x, y in list(itertools.product([-1, 0, 1], repeat=2)):
                    if self.map_gray[i + x][j + y] == self.BLOCK:
                        tmp_blocks.append((x, y))
                        if len(tmp_blocks) > 1:
                            break
                if len(tmp_blocks) == 0:
                    self.map_gray[i][j] = self.BROAD_ROAD
                    continue
                elif len(tmp_blocks) == 1:
                    '''
                    ...     ...
                    ....   ....
                     ...   ...
                    '''
                    x, y = tmp_blocks[0]
                    if x == 0 or y == 0:  # 必须是四个角上
                        continue
                    flag1 = 0 <= j - 2 * x <= 99 and 0 <= i + y <= 99 and self.map_gray[j - 2 * x][i] != self.BLOCK and \
                        self.map_gray[j - 2 * x][i + y] != self.BLOCK
                    flag2 = 0 <= i - 2 * x <= 99 and 0 <= j + y <= 99 and self.map_gray[i - 2 * x][j] != self.BLOCK and \
                        self.map_gray[i - 2 * x][j + y] != self.BLOCK
                    if flag1 and flag2:
                        self.map_gray[i][j] = self.BROAD_ROAD
                        # 要根据具体情况加偏移量
                        if self.map_gray[i - 2 * x][j - y] == self.BLOCK:
                            self.broad_shifting[(i, j)] = (0, x * 0.25)
                        elif self.map_gray[j - 2 * x][i - y] == self.BLOCK:
                            self.broad_shifting[(i, j)] = (-y * 0.25, 0)
                        else:
                            # 往远离的方向推
                            self.broad_shifting[(i, j)] = (-y * 0.25, x * 0.25)

        # 再算窄路
        for i in range(100):
            for j in range(100):
                if self.map_gray[i][j] == self.BROAD_ROAD:
                    continue
                for x, y in itertools.product([0, 1], repeat=2):
                    if i + x > 99 or j + y > 99:
                        continue
                    if self.map_gray[i + x][j + y] == self.BLOCK:
                        break
                else:
                    self.map_gray[i][j] = self.ROAD
                    # 一个斜着的格子也过不去
                    if (i == 0 or j == 99 or self.map_gray[i - 1][j + 1] == self.BLOCK) and (
                            i == 99 or j == 0 or self.map_gray[i + 1][j - 1] == self.BLOCK):
                        self.map_gray[i][j] = self.GROUND
        for i, j in self.workbenchs_loc:  # 集中处理工作台
            if self.map_gray[i][j] == self.BROAD_ROAD:
                continue
            tmp_blocks = []  # 十字区域障碍
            for x, y in [(1, 0), (0, 1), (0, -1), (-1, 0)]:
                if i + x < 0 or i + x > 99 or j + y < 0 or j + y > 99 or self.map_gray[i + x][j + y] == self.BLOCK:
                    tmp_blocks.append((x, y))
            if len(tmp_blocks) > 2:
                self.unreanchble_warkbench.add((i, j))
                continue
            elif len(tmp_blocks) == 2:  # 两个障碍
                if '4' <= self.map_data[i][j] <= '9':  # 这类是一定要卖给它东西的, 直接标记为不能用
                    self.unreanchble_warkbench.add((i, j))
                    continue
                # 对于1-3 只有两个障碍是对角的形式才行
                if tmp_blocks[0][0] == tmp_blocks[1][0] or tmp_blocks[0][1] == tmp_blocks[1][1]:
                    self.unreanchble_warkbench.add((i, j))
                    continue
            if '4' <= self.map_data[i][j] <= '9':
                if (i == 0 or j == 0 or self.map_gray[i - 1][j - 1] == self.BLOCK) and (
                        i == 99 or j == 99 or self.map_gray[i + 1][j + 1] == self.BLOCK):
                    self.unreanchble_warkbench.add((i, j))
                    continue
                if (i == 0 or j == 99 or self.map_gray[i - 1][j + 1] == self.BLOCK) and (
                        i == 99 or j == 0 or self.map_gray[i + 1][j - 1] == self.BLOCK):
                    self.unreanchble_warkbench.add((i, j))
                    continue

        # 算广路 方便机器人冲冲冲 上下左右都是宽路即可
        for i in range(2, 98):
            for j in range(2, 98):
                if self.map_gray[i][j] < self.BROAD_ROAD or (i, j) in self.broad_shifting:
                    continue
                # 十字区域都是宽路即可
                for x, y in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
                    if self.map_gray[i][j] < self.BROAD_ROAD or (i, j) in self.broad_shifting:
                        break
                else:
                    self.map_gray[i][j] = self.SUPER_BROAD_ROAD

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
                    if n_x > 99 or n_y > 99 or n_x < 0 or n_y < 0 or visited_loc[n_x][n_y]:
                        continue
                    if self.map_data[n_x][n_y] == 'A':
                        visited_robot.append(self.robots_loc[(n_x, n_y)])
                    # 只关心1-9，因为空手去89没有意义
                    if '1' <= self.map_data[n_x][n_y] <= '7' and (n_x, n_y) not in self.unreanchble_warkbench:
                        visited_workbench.append(
                            self.workbenchs_loc[(n_x, n_y)])  # 将这个工作台添加到列表
                    visited_loc[n_x][n_y] = True
                    if self.map_gray[n_x][n_y] >= self.ROAD:
                        dq.append((n_x, n_y))
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
                    if n_x > 99 or n_y > 99 or n_x < 0 or n_y < 0 or visited_loc[n_x][n_y]:
                        continue
                    if '1' <= self.map_data[n_x][n_y] <= '9' and (n_x, n_y) not in self.unreanchble_warkbench:
                        visited_workbench.append(
                            (self.workbenchs_loc[(n_x, n_y)], int(self.map_data[n_x][n_y])))  # 将这个工作台添加到列表
                    visited_loc[n_x][n_y] = True
                    if self.map_gray[n_x][n_y] >= self.BROAD_ROAD:
                        dq.append((n_x, n_y))
            for wb_ID, wb_type in visited_workbench:
                # 为每一个在集合中的工作台寻找可以访问的目标
                if wb_type in [8, 9]:  # 8,9 只收不卖
                    continue
                for aim_ID, aim_type in visited_workbench:
                    if wb_type in Workbench.WORKSTAND_IN[aim_type]:
                        res[wb_ID].append(aim_ID)
            visited_workbench.clear()
        return res

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
            if loc in self.unreanchble_warkbench:
                continue
            x, y = loc
            flag1, flag2 = True, True
            workbench_type = self.map_data[x][y]
            if '8' <= workbench_type <= '9':
                flag1 = False  # 空手到8、9没有意义
            # 宽路先都算
            # elif '1' <= workbench_type <= '3':
            #     flag2 = False  # 拿着东西到1、2、3没有意义
            if flag1:
                self.buy_map[idx] = copy.deepcopy(base_map)
                self.gen_a_path(idx, loc, False)
            if flag2:
                self.sell_map[idx] = copy.deepcopy(base_map)
                self.gen_a_path(idx, loc, True)

    def get_avoid_path(self, wait_flaot_loc, work_path, robots_loc, broad_road=False, safe_dis: float = None):
        '''
        为机器人规划一条避让路径, 注意此函数会临时修改map_gray如果后续有多线程优化, 请修改此函数
        wait_flaot_loc: 要避让的机器人坐标
        work_path: 正常行驶的机器人路径
        robots_loc: 其他机器人坐标
        broad_road: 是否只能走宽路, 根据机器人手中是否 持有物品确定
        return: 返回路径, 为空说明无法避让
        '''
        if not safe_dis:
            if broad_road:
                safe_dis = 1.06
            else:
                safe_dis = 0.98
        if broad_road:
            low_value = self.BROAD_ROAD
        else:
            low_value = self.ROAD
        node_x, node_y = self.loc_float2int(*wait_flaot_loc)
        tmp_blocks = {}  # 将其他机器人及其一圈看做临时障碍物
        path_map = {(node_x, node_y): (node_x, node_y)}  # 记录路径
        for robot_loc in robots_loc:
            robot_x, robot_y = self.loc_float2int(*robot_loc)
            block_turns = self.TURNS + [(0,0)] #+ [(0, 2), (0, -2), (-2, 0), (2, 0)]
            for x, y in block_turns:
                block_x, block_y = robot_x + x, robot_y + y
                if block_x < 0 or block_x > 99 or block_y < 0 or block_y > 99:
                    continue
                # 暂存原来的值，方便改回去
                tmp_blocks[(block_x, block_y)
                           ] = self.map_gray[block_x][block_y]
                self.map_gray[block_x][block_y] = self.BLOCK
        dq = deque([(node_x, node_y)])
        aim_node = None  # 记录目标节点
        # 开始找路 直接bfs找一下先看看效果
        while dq:
            node_x, node_y = dq.pop()
            for x, y in self.TURNS:
                next_x, next_y = node_x + x, node_y + y
                if (next_x, next_y) in path_map or next_x < 0 or next_y < 0 or next_x >= 100 or next_y >= 100 or \
                        self.map_gray[next_x][next_y] < low_value:
                    continue
                # 保存路径
                path_map[(next_x, next_y)] = (node_x, node_y)
                float_loc = self.loc_int2float(
                    next_x, next_y, self.map_gray[next_x][next_y] == self.ROAD)
                if self.dis_loc2path(float_loc, work_path) >= safe_dis:
                    aim_node = (next_x, next_y)
                    dq = None  # 清空队列使外层循环退出
                    break
                dq.appendleft((next_x, next_y))  # 新点放左边
        # 恢复map_gray
        for k, v in tmp_blocks.items():
            block_x, block_y = k
            self.map_gray[block_x][block_y] = v
        if not aim_node:
            return []
        path = []  # 重建路径, 这是个逆序的路径
        x, y = aim_node
        while 1:
            path.append((x, y))
            if (x, y) == path_map[(x, y)]:
                break
            x, y = path_map[(x, y)]
        float_path = []  # 转成浮点
        for i in range(1, len(path) + 1):
            float_path.append(self.loc_int2float(
                *path[-i], self.map_gray[x][y] == self.ROAD))
        return float_path

    def get_float_path(self, float_loc, workbench_ID, broad_road=False, key_point=False):
        '''
        获取浮点型的路径
        float_loc: 当前位置的浮点坐标
        workbench_ID: 工作台ID
        broad_road: 是否只能走宽路  
        key_point: 关键点模式, 若指定为True则只返回路径上需要转弯的关键点
        返回路径(换算好的float点的集合)
        '''
        if broad_road:
            path = self.get_path(float_loc, workbench_ID, broad_road)
        else:  # 如果不指定走宽路，则优先走宽路
            path = self.get_better_path(float_loc, workbench_ID)
        if not path:
            return path
        if key_point and len(path) > 2:
            new_path = [path[0]]
            # 记录之前两个点的差值
            tmp_x, tmp_y = path[1][0] - path[0][0], path[1][1] - path[0][1]
            idx = 2
            while idx < len(path):
                next_x, next_y = path[idx][0] - \
                    path[idx - 1][0], path[idx][1] - path[idx - 1][1]
                if next_x != tmp_x or next_y != tmp_y:  # 变向了说明在前一个点处拐弯
                    new_path.append(path[idx - 1])
                    tmp_x, tmp_y = next_x, next_y
                idx += 1
            new_path.append(path[-1])
            # logging.info(f'Path: {path} new_path{new_path}')
            path = new_path

        for i in range(len(path) - 1):
            x, y = path[i]
            path[i] = self.loc_int2float(
                x, y, self.map_gray[x][y] == self.ROAD)
        path[-1] = self.loc_int2float(*path[-1])
        return path

    def get_better_path(self, float_loc, workbench_ID, threshold=15):
        '''
        同时计算宽路和窄路，如果宽路比窄路多走不了阈值, 就选宽路
        threshold: 阈值，如果宽路窄路都有，宽路比窄路多走不了阈值时走宽路
        '''
        # 窄路
        path1 = self.get_path(float_loc, workbench_ID)
        # 宽路
        path2 = self.get_path(float_loc, workbench_ID, True)
        # 不存在宽路，或者宽路比窄路多走超过阈值则返回窄路
        if not path2 or len(path2) - len(path1) > threshold:
            return path1
        return path2

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
                if test_x < 0 or test_x > 99 or test_y < 0 or test_y > 99:
                    continue
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
