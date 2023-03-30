# coding=utf-8
import heapq
import math
import time
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
                new_cost = self.__cost_dict[current] + self.__cost(current,next)
                if next not in self.__cost_dict or \
                    new_cost < self.__cost_dict[next]:
                    self.__cost_dict[next] = new_cost
                    self.__parent[next] = current
                    pq.push(PqItem(self.__all_cost(next,goal),next))

        path = self.get_actual_path(goal)
        for point in path:
            self.__map.map_gray[point[0]][point[1]] = self.__map.PATH


    def get_actual_path(self,goal):
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
            next = (current[0] + dir[0],current[1] + dir[1])
            if next[0] >= len(gray_map[0]) or next[0] < 0 or \
                    next[1] >= len(gray_map) or next[1] < 0:
                continue
            yield next
