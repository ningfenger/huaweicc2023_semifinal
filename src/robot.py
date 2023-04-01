# coding=utf-8
import math
from typing import Optional, List, Tuple
import numpy as np
import time
'''
机器人类
'''
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



class Robot:
    # 状态常量 0 空闲, 1 购买途中, 2 等待购买, 3 出售途中, 4 等待出售
    FREE_STATUS = 0
    MOVE_TO_BUY_STATUS = 1
    WAIT_TO_BUY_STATUS = 2
    MOVE_TO_SELL_STATUS = 3
    WAIT_TO_SELL_STATUS = 4

    def __init__(self, ID: int, loc: Tuple[int]):
        self.ID = ID
        self.loc = loc
        self.workbench_ID = -1  # 所处工作台ID -1代表没有
        self.item_type = 0  # 携带物品类型
        self.time_value = 0.0  # 时间价值系数
        self.clash_value = 0.0  # 碰撞价值系数
        self.palstance = 0.0  # 角速度
        self.speed = (0.0, 0.0)  # 线速度
        self.toward = 0.0  # 朝向
        self.status: int = 0  # 0 空闲, 1 购买途中, 2 等待购买, 3 出售途中, 4 等待出售
        self.target = -1  # 当前机器人的目标控制台 -1代表无目标
        self.__plan = (-1, -1)  # 设定买和卖的目标工作台
        self.target_workbench_list = []  # 可到达的工作台列表
        self.path = []

    def set_plan(self, buy_ID: int, sell_ID: int):
        '''
        设置机器人计划, 传入购买和出售工作台的ID
        '''
        self.__plan = buy_ID, sell_ID

    def get_buy(self) -> int:
        '''
        获取买的目标
        '''
        return self.__plan[0]

    def get_sell(self) -> int:
        '''
        获取卖的目标
        '''
        return self.__plan[1]

    def find_temp_tar(self):
        robot_pos = np.array(list(self.loc))
        dists = np.sqrt(np.sum((self.path - robot_pos) ** 2, axis=1))
        nearest_row = np.argmin(dists)
        row1 = min(nearest_row + 1, len(self.path) - 1)
        row2 = min(nearest_row + 2, len(self.path) - 1)
        if nearest_row == 0:

            rowb1 = min(nearest_row + 1, len(self.path))
            rowb2 = nearest_row
        else:
            rowb1 = nearest_row
            rowb2 = nearest_row - 1
        target1 = self.path[row1, :]
        target2 = self.path[row2, :]
        # targetb1 = self.path[rowb1, :]
        # targetb2 = self.path[rowb2, :]
        return target1, target2
        # return line_ray_intersection2(target1, target2, targetb1, targetb2)
        # return line_ray_intersection(target1, target2, self.loc, self.toward)

    # 四个动作
    def forward(self, speed: float):
        '''
        设置前进速度，单位为米/秒。
        正数表示前进。 
        负数表示后退。
        '''
        print("forward", self.ID, speed)

    def rotate(self, palstance: float):
        '''
        设置旋转速度，单位为弧度/秒。
        负数表示顺时针旋转。
        正数表示逆时针旋转。
        '''
        print('rotate', self.ID, palstance)

    def buy(self):
        '''
        购买当前工作台的物品，以输入数据的身处工作台 ID 为准。
        所处工作台与目标工作台一致才出售
        '''
        if self.workbench_ID == self.target:
            print("buy", self.ID)
            return True
        return False

    def sell(self) -> bool:
        '''
        出售物品给当前工作台，以输入数据的身处工作台 ID 为准。
        所处工作台与目标工作台一致才出售
        '''
        if self.workbench_ID == self.target:
            print("sell", self.ID)
            return True
        return False

    def destroy(self):
        '''
        销毁物品。
        '''
        print("destroy", self.ID)

    def update(self, s: str):
        '''
        根据判题器的输入更新机器人状态, 记得更新status
        '''
        s = s.split()
        self.workbench_ID, self.item_type = map(int, s[:2])
        self.time_value, self.clash_value, self.palstance, speed_x, speed_y, self.toward, x, y = map(
            float, s[2:])
        self.speed = (speed_x, speed_y)
        self.loc = (x, y)
