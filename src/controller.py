# coding=utf-8
import numpy as np


from robot import Robot
from workbench import Workbench
from workmap import Workmap
from typing import Optional, List
import tools
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

    def radar(self, idx_robot, d_theta):
        # 当前位置与朝向
        point = self.robots[idx_robot].loc
        theta = self.robots[idx_robot].toward + d_theta
        theta = (theta + math.pi) % (2 * math.pi) - math.pi
        # 当前位置所处的格子
        # raw, col = int(round(point[1] * 2 - 0.5)), int(round(point[0] * 2 - 0.5))
        raw, col = int(point[1] // 0.5), int(point[0] // 0.5)

        # 取出所有边界点
        if theta == 0:
            # 正右
            x_set_all = np.arange(col + 1, min(col + 4, 99), 1) * 0.5
            y_set_all = np.ones_like(x_set_all) * point[1]
        elif theta == math.pi / 2:
            # 正上
            y_set_all = np.arange(raw + 1, min(raw + 4, 99), 1) * 0.5
            x_set_all = np.ones_like(y_set_all) * point[0]
        elif theta == math.pi:
            # 正左
            x_set_all = np.arange(col, max(col - 3, 0), -1) * 0.5
            y_set_all = np.ones_like(x_set_all) * point[1]
        elif theta == -math.pi / 2:
            # 正下
            y_set_all = np.arange(raw, max(raw - 3, 0), -1) * 0.5
            x_set_all = np.ones_like(y_set_all) * point[0]
        else:
            # 其他方向

            # x方向栅格点集
            if -math.pi / 2 < theta < math.pi / 2:
                # 1 4 象限
                x_set_xgrid = np.arange(col + 1, min(col + 4, 99), 1) * 0.5
            else:
                # 2 3 象限
                x_set_xgrid = np.arange(col, max(col - 3, 0), -1) * 0.5
            y_set_xgrid = np.tan(theta) * (x_set_xgrid - point[0]) + point[1]

            # y方向栅格点集
            if 0 < theta < math.pi:
                # 1 2 象限
                y_set_ygrid = np.arange(raw + 1, min(raw + 4, 99), 1) * 0.5


            else:
                # 3 4 象限
                y_set_ygrid = np.arange(raw, max(raw - 3, 0), -1) * 0.5
            x_set_ygrid = 1 / np.tan(theta) * (y_set_ygrid - point[1]) + point[0]
            x_set_all = np.concatenate((x_set_xgrid, x_set_ygrid))
            y_set_all = np.concatenate((y_set_xgrid, y_set_ygrid))

            # 得到排序后的索引
            idx = np.argsort(y_set_all)
            # 将坐标按照排序后的索引进行排序
            if theta < 0:
                x_set_all = x_set_all[idx]
                y_set_all = y_set_all[idx]
            else:
                x_set_all = x_set_all[idx[::-1]]
                y_set_all = y_set_all[idx[::-1]]

        # 取出所有边界点↑
        x_set_near = x_set_all[:-1]
        x_set_far = x_set_all[1:]

        y_set_near = y_set_all[:-1]
        y_set_far = y_set_all[1:]

        x_set_mid = (x_set_near + x_set_far) / 2
        y_set_mid = (y_set_near + y_set_far) / 2

        mask = np.zeros_like(x_set_mid, dtype=bool)
        mask[(x_set_mid >= 0) & (x_set_mid <= 50) & (y_set_mid >= 0) & (y_set_mid <= 50)] = True
        x_set_mid = x_set_mid[mask]
        y_set_mid = y_set_mid[mask]
        idx_ob = -1
        for i_point in range(len(x_set_mid)):
            x = x_set_mid[i_point]
            y = y_set_mid[i_point]
            raw, col = tools.cor2rc(x, y)

            if self.m_map[raw, col] == 0:
                idx_ob = i_point
                break

        if idx_ob == -1:
            return 100
        else:
            return np.sqrt((x_set_near[idx_ob] - point[0]) ** 2 + (y_set_near[idx_ob] - point[1]) ** 2)



    def move(self, idx_robot):
        # 机器人沿着指定路线移动
        k_r = 10
        dis_l = self.radar(idx_robot, math.pi / 6)
        dis_r = self.radar(idx_robot, -math.pi / 6)
        target_loc_local, target_loc_further = self.robots[idx_robot].find_temp_tar()

        target_vec_local = [target_loc_local[0] - self.robots[idx_robot].loc[0], target_loc_local[1] - self.robots[idx_robot].loc[1]]
        target_theta_local = np.arctan2(target_vec_local[1], target_vec_local[0])

        target_vec_further = [target_loc_further[0] - self.robots[idx_robot].loc[0],
                            target_loc_further[1] - self.robots[idx_robot].loc[1]]
        target_theta_further = np.arctan2(target_vec_further[1], target_vec_local[0])

        robot_theta = self.robots[idx_robot].toward
        delta_theta_local = target_theta_local - robot_theta
        delta_theta_local = (delta_theta_local + math.pi) % (2 * math.pi) - math.pi

        delta_theta_further = target_theta_further - robot_theta
        delta_theta_further = (delta_theta_further + math.pi) % (2 * math.pi) - math.pi

        self.robots[idx_robot].rotate(delta_theta_local * k_r)
        if abs(delta_theta_local) > math.pi / 4:
            print("forward", idx_robot, 0)
        elif abs(delta_theta_further) > math.pi / 4:
            print("forward", idx_robot, 3)
        else:
            print("forward", idx_robot, 6)
        pass

    def control(self, frame_id: int, money: int):

        print(frame_id)
        for i in range(4):
            if self.robots[i].status == Robot.MOVE_TO_BUY_STATUS:
                # 前往购买
                self.move(i)

