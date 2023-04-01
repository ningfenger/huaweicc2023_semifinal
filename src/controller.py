# coding=utf-8
import numpy as np


from robot import Robot
from workbench import Workbench
from workmap import Workmap
from typing import Optional, List
import math
'''
控制类 决策，运动
'''


class Controller:
    # 总帧数
    TOTAL_FRAME = 50*60*5
    # 控制参数

    def __init__(self, robots: List[Robot], workbenchs: List[Workbench], m_map:Workmap):
        self.robots = robots
        self.workbenchs = workbenchs
        self.m_map = m_map
        self.m_map = np.array(m_map.map_gray)

    def path_opt(self, idx_robot):
        for point in self.robots[idx_robot].path:
            row, col = 100 - int(point[1] * 2 - 0.5), int(point[0] * 2 - 0.5)
            a=100000



    def move(self, idx_robot):
        # 机器人沿着指定路线移动
        k_r = 10
        self.path_opt(idx_robot)
        target_loc_local = self.robots[idx_robot].find_temp_tar()

        target_vec = [target_loc_local[0] - self.robots[idx_robot].loc[0], target_loc_local[1] - self.robots[idx_robot].loc[1]]
        target_theta = np.arctan2(target_vec[1], target_vec[0])
        robot_theta = self.robots[idx_robot].toward
        delta_theta = target_theta - robot_theta
        delta_theta = (delta_theta + math.pi) % (2 * math.pi) - math.pi
        self.robots[idx_robot].rotate(delta_theta * k_r)
        if abs(delta_theta) > math.pi / 4:
            print("forward", idx_robot, 0)
        else:
            print("forward", idx_robot, 6)
        pass

    def control(self, frame_id: int, money: int):

        print(frame_id)
        for i in range(4):
            if self.robots[i].status == Robot.MOVE_TO_BUY_STATUS:
                # 前往购买
                self.move(i)

