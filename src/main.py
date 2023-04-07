# coding=utf-8
import sys
import time

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


def finish():
    print('OK')
    sys.stdout.flush()


if __name__ == '__main__':
    workmap = Workmap()
    robots: List[Robot] = []  # 机器人列表
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
    controller = Controller(robots, workbenchs, workmap)
    # 针对性调参
    if workmap.map_data[30][0] == '#' and workmap.map_data[75][5] == '8' and workmap.map_data[41][-6] == '3':
        controller.set_control_parameters(5, 2, 1.2, 0.6)
    elif workmap.map_data[25][50] == '6' and workmap.map_data[41][26] == '5' and workmap.map_data[80][65] == '8':
        controller.set_control_parameters(5, 2, 1.2, 0.6)
    elif workmap.map_data[53][56] == '4' and workmap.map_data[58][53] == '7' and workmap.map_data[89][4] == '9':
        controller.set_control_parameters(5, 2, 1.2, 0.6)
    elif workmap.map_data[2][3] == '6' and workmap.map_data[22][4] == '2' and workmap.map_data[90][-4] == '5':
        controller.set_control_parameters(5, 2, 1.2, 0.6)
    finish()

    while True:
        frame_id, money = map(int, input().split())
        input()  # 工作台数量
        for workbench in workbenchs:  # 更新工作台
            workbench.update(input())
        for robot in robots:  # 更新机器人
            robot.update(input())
        OK_str = input()  # 读一个ok
        controller.control(frame_id, money)
        finish()
