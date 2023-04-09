# coding=utf-8
import heapq
import math
import numpy as np
import time

def np_norm(loc1, loc2):
    return np.sqrt(np.sum((loc2 - loc1) ** 2))

def np_theta(loc1, loc2):
    vec = loc2 - loc1
    return np.arctan2(vec[1], vec[0])

def will_collide(x1, y1, vx1, vy1, x2, y2, vx2, vy2, t_max, r = 1.1):
    # 计算机器人之间的初始距离
    dist = math.sqrt((x1 - x2)**2 + (y1 - y2)**2)
    if dist < r:
        return (True, None, None, 0.1, 0.1)
    # 计算相对速度
    rel_vx = vx1 - vx2
    rel_vy = vy1 - vy2
    # 如果机器人的相对速度为零，则它们永远不会相遇
    if rel_vx == 0 and rel_vy == 0:
        return (False, None, None, None, None)
    # 计算参数方程
    a = rel_vx**2 + rel_vy**2
    b = 2 * ((x1 - x2) * rel_vx + (y1 - y2) * rel_vy)
    c = (x1 - x2)**2 + (y1 - y2)**2 - 4 * r**2
    delta = b**2 - 4 * a * c
    # 如果delta小于零，则机器人之间不会相遇
    if delta < 0:
        return (False, None, None, None, None)
    else:
        t1 = (-b + math.sqrt(delta)) / (2 * a)
        t2 = (-b - math.sqrt(delta)) / (2 * a)
        t = min(t1, t2)
        # 如果时间是负数或者超出了预测的时间范围，则机器人之间不会相遇
        if t < 0 or t > t_max:
            return (False, None, None, None, None)
        # 计算碰撞点的位置
        collision_x = (x1 + vx1 * t + x2 + vx2 * t) / 2
        collision_y = (y1 + vy1 * t + y2 + vy2 * t) / 2
        # 计算碰撞点距离各自的长度
        distance1 = math.sqrt((collision_x - x1)**2 + (collision_y - y1)**2) - r
        distance2 = math.sqrt((collision_x - x2)**2 + (collision_y - y2)**2) - r
        return (True, collision_x, collision_y, distance1, distance2)

def will_collide2(x1, y1, vx1, vy1, x2, y2, vx2, vy2, t_max, r_colli = 1.09):
    # 计算机器人之间的初始距离
    dist = math.sqrt((x1 - x2)**2 + (y1 - y2)**2)
    if dist < r_colli:
        return r_colli
    # 计算相对速度
    rel_vx = vx1 - vx2
    rel_vy = vy1 - vy2
    # 如果机器人的相对速度为零，则它们永远不会相遇
    if rel_vx == 0 and rel_vy == 0:
        return 100
    # 计算参数方程
    a = rel_vx**2 + rel_vy**2
    b = 2 * ((x1 - x2) * rel_vx + (y1 - y2) * rel_vy)
    c = (x1 - x2)**2 + (y1 - y2)**2
    d_0 = np.sqrt(c)
    d_max = np.sqrt(a * t_max **2 + b * t_max + c)
    t_small = - 2 * a / b
    d_small = np.sqrt(a * t_small **2 + b * t_small + c)
    if t_small > 0 and t_small < t_max:
        # 极值点在中间
        t_set = np.array([0, t_small, t_max])
        d_set = np.array([d_0, d_small, d_max])
        idx = np.argmin(d_set)
        t = t_set[idx]
        d = d_set[idx]
    else:
        # 极值点不在中间
        t_set = np.array([0, t_max])
        d_set = np.array([d_0, d_max])
        idx = np.argmin(d_set)
        t = t_set[idx]
        d = d_set[idx]

    return d

def line_ray_intersection(target1, target2, loc, theta):
    dircos_robot = [math.cos(theta), math.sin(theta)]

    vex_1to2 = target2 - target1

    length_1to2 = np.sqrt(np.sum(np.power(vex_1to2, 2)))

    dircos_1to2 = vex_1to2 / length_1to2

    A = np.array([[dircos_robot[0], -dircos_1to2[0]], [dircos_robot[1], -dircos_1to2[1]]])
    B = np.array([[target1[0] - loc[0]], [target1[1] - loc[1]]])

    # 使用克莱姆法则求解线性方程组
    detA = np.linalg.det(A)
    if detA != 0:
        # 如果系数矩阵的行列式不为0，则有唯一解
        t = np.zeros_like(B)
        for i in range(2):
            Ai = A.copy()
            # time.sleep(20)
            Ai[:, i] = B[:, 0]
            detAi = np.linalg.det(Ai)
            t[i] = detAi / detA

        if t[0] > 0 and t[0] < 1 and t[1] < 0:
            target_loc = [loc[0] + dircos_robot[0] * (t[0] + 1), loc[1] + dircos_robot[1] * (t[0] + 1)]
        else:
            target_loc = target1
    else:
        target_loc = target1
    return target_loc


def line_ray_intersection2(target1, target2, targetb1, targetb2):
    m_len = 1.5
    vex_b2tob1 = targetb1 - targetb2

    length_b2tob1 = np.sqrt(np.sum(np.power(vex_b2tob1, 2)))

    dircos_b2tob1 = vex_b2tob1 / length_b2tob1

    vex_1to2 = target2 - target1

    length_1to2 = np.sqrt(np.sum(np.power(vex_1to2, 2)))

    dircos_1to2 = vex_1to2 / length_1to2

    A = np.array([[dircos_b2tob1[0], -dircos_1to2[0]], [dircos_b2tob1[1], -dircos_1to2[1]]])
    B = np.array([[target1[0] - targetb1[0]], [target1[1] - targetb1[1]]])

    # 使用克莱姆法则求解线性方程组
    detA = np.linalg.det(A)
    if detA != 0:
        # 如果系数矩阵的行列式不为0，则有唯一解
        t = np.zeros_like(B)
        for i in range(2):
            Ai = A.copy()
            # time.sleep(20)
            Ai[:, i] = B[:, 0]
            detAi = np.linalg.det(Ai)
            t[i] = detAi / detA

        if t[0] >= 0 and t[1] < 0:
            target_loc = [targetb1[0] + dircos_b2tob1[0] * (t[0] + 1), targetb1[1] + dircos_b2tob1[1] * (t[0] + 1)]
        else:
            target_loc = target1
    else:
        target_loc = target1
    return target_loc


class PqItem:
    def __init__(self, cost, point):
        self.__cost = cost
        self.__point = point

    @property
    def cost(self):
        return self.__cost

    @property
    def point(self):
        return self.__point


    def __lt__(self, other):
        if self.cost == other.cost:
            return self.point < other.point
        return self.cost < other.cost


def cor2rc(x, y):
    # 坐标返回在哪个格子
    raw = 99 - int(y // 0.5)
    col = int(x // 0.5)
    return raw, col


class PriorityQueue:
    def __init__(self):
        self.data = []

    def push(self, item):
        heapq.heappush(self.data, item)

    def pop(self):
        """
        弹出并返回优先队列首位元素
        :return: 队列首位元素
        """
        item = heapq.heappop(self.data)
        return item

    def top(self):
        """
        返回优先队列首位元素，不弹出
        :return: 队列首位元素
        """
        if len(self.data) == 0:
            return None
        return self.data[0]

    def empty(self):
        return len(self.data) == 0


class AStar:
    direction = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1),
                 (1, -1), (-1, -1), (-1, 1)]

    def __init__(self, map):
        self.__map = map
        self.__obstacle = set()  # 障碍集合
        self.__parent = dict()  # key:坐标，value:往回走一步到达的坐标
        self.__closed = {}  # 访问过的坐标
        self.__open = []
        self.__cost_dict = dict()  # key:坐标，value:起点到这的cost
        self.__robot_with_product = False
        self.__init()

    def __get_gray_map(self):
        return self.__map.map_gray

    def __init(self):
        """
        遍历map，将map中的障碍添加到self.__obstacle
        :return:
        """
        map_gray = self.__get_gray_map()

        for i in range(len(map_gray)):
            for j in range(len(map_gray[0])):
                if map_gray[i][j] == self.__map.BLOCK:
                    self.__obstacle.add((i, j))

    def __restore(self):
        self.__parent = dict()
        self.__closed = {}
        self.__open = []
        self.__cost_dict = dict()
        self.__robot_with_product = False

    def get_path_cor(self, start_cor, goal_cor, robot_with_product):
        start = (100 - int(start_cor[1] * 2 - 0.5), int(start_cor[0] * 2 - 0.5))
        goal = (100 - int(goal_cor[1] * 2 - 0.5), int(goal_cor[0] * 2 - 0.5))

        # 格子到格子可以查表
        self.get_path(start, goal, robot_with_product)
        path = self.get_actual_path(goal)
        path_cor = []
        for point in path:
            path_cor.append([point[1] * 0.5 + 0.25, (100 - point[0]) * 0.5 + 0.25])
        return path_cor

    def get_path(self, start, goal, robot_with_product):
        """
        计算从起点到终点的最短路径
        :param start:路径起点，x,y tuple
        :param goal: 希望前往的目标，x,y tuple
        :param robot_with_product: 机器人是否携带物品，bool
        :return:路径
        """
        self.__restore()
        self.__robot_with_product = robot_with_product
        self.__parent[start] = start
        self.__cost_dict[start] = 0
        self.__cost_dict[goal] = math.inf

        pq = PriorityQueue()
        pq.push(PqItem(self.__all_cost(start, goal), start))

        while not pq.empty():
            item = pq.pop()
            current = item.point
            if current == goal:
                break
            for next in self.get_neighbors(current):
                new_cost = self.__cost_dict[current] + self.__cost(current, next)
                if next not in self.__cost_dict or \
                        new_cost < self.__cost_dict[next]:
                    self.__cost_dict[next] = new_cost
                    self.__parent[next] = current
                    pq.push(PqItem(self.__all_cost(next, goal), next))

        path = self.get_actual_path(goal)
        for point in path:
            self.__map.map_gray[point[0]][point[1]] = self.__map.PATH

    def get_actual_path(self, goal):
        """
        从self.parent反向计算出从start->goal需要通过的点
        :param goal:
        :return:
        """
        path = [goal]
        pre = goal
        while pre != self.__parent[pre]:
            pre = self.__parent[pre]
            path.append(pre)
        path = reversed(path)
        return path

    ##:TODO:测试这个函数
    def __can_reach(self, start, goal):
        if start in self.__obstacle or goal in self.__obstacle:
            return False

        if self.__robot_with_product and \
                self.__map.map_gray[goal[0]][goal[1]] == self.__map.ROAD:
            return False

        if start[0] == goal[0] or start[1] == goal[1]:
            return True

        # start和goal在右斜对角线上
        if start[0] - goal[0] == start[1] - goal[1]:
            #  b1  goal
            # start b2
            block1 = (min(start[0], goal[0]), max(start[1], goal[1]))
            block2 = (max(start[0], goal[0]), min(start[1], goal[1]))

        else:
            #  goal  b2
            #  b1  start
            block1 = (min(start[0], goal[0]), min(start[1], goal[1]))
            block2 = (max(start[0], goal[0]), max(start[1], goal[1]))

        if block1 in self.__obstacle or block2 in self.__obstacle:
            return False

        return True

    def __cost(self, start, goal):
        """
        获得从start->goal的cost
        :param start:
        :param goal:
        :return:
        """
        if not self.__can_reach(start, goal):
            return math.inf
        return math.sqrt((goal[0] - start[0]) ** 2 + (goal[1] - start[1]) ** 2)

    def __all_cost(self, current, goal):
        """
        获得从start->current->goal的cost
        :param current:
        :param goal:
        :return:
        """
        return self.__cost_dict[current] + self.__cost(current, goal)

    def get_neighbors(self, current):
        for dir in self.direction:
            gray_map = self.__get_gray_map()
            next = (current[0] + dir[0], current[1] + dir[1])
            if next[0] >= len(gray_map[0]) or next[0] < 0 or \
                    next[1] >= len(gray_map) or next[1] < 0:
                continue
            yield next

