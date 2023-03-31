# coding=utf-8
import sys
from workmap import Workmap
from robot import Robot
from workbench import Workbench
from controller import Controller
from typing import Optional, List
import numpy as np
import os
from tools import *
try:
    os.chdir('./src')
except:
    pass

def read_util_ok():
    while input() != "OK":
        pass


def finish():
    print('OK')
    sys.stdout.flush()


if __name__ == '__main__':
    workmap = Workmap(True)
    robots: List[Robot]= []  # 机器人列表
    workbenchs: List[Workbench] = []  # 工作台列表
    # 读入初始化地图
    for t, loc in workmap.read_map():
        if t == 'A':
            robots.append(Robot(len(robots), loc))
        else:
            workbenchs.append(Workbench(len(workbenchs), int(t), loc))
    workmap.init_roads()
    for idx, r2w in enumerate(workmap.robot2workbench()):
        robots[idx].target_workbench_list = r2w
    for idx, w2w in enumerate(workmap.workbench2workbench()):
        workbenchs[idx].target_workbench_list = w2w 
    # 计算一下路径
    workmap.gen_paths()
    # path = workmap.get_float_path(robots[1].loc, robots[1].target_workbench_list[0])
    # 测试路径，非正式决策路径
    astar = AStar(workmap)
    w_target = 1
    r_idx = 0
    robots[r_idx].target = w_target
    start_ = (robots[r_idx].loc[0], robots[r_idx].loc[1])
    goal_ = (workbenchs[w_target].loc[0], workbenchs[w_target].loc[1])
    start = (25.25, 49.75)
    goal = (18.75, 49.75)
    robots[r_idx].path = np.array(astar.get_path_cor(start, goal, False))
    robots[r_idx].status = Robot.MOVE_TO_BUY_STATUS
    # 测试路径，非正式决策路径
    controller = Controller(robots, workbenchs)
    finish()

    while True:
        frame_id, money = map(int, input())
        input()  # 工作台数量
        for workbench in workbenchs:  # 更新工作台
            workbench.update(input())
        for robot in robots:  # 更新机器人
            robot.update(input())
        OK_str = input()  # 读一个ok
        controller.control(frame_id, money)
        finish()
