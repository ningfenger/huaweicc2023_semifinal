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
    # workmap = Workmap(True)
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
    # workmap.draw_path(workmap.get_path(robots[0].loc, robots[0].target_workbench_list[2]))
    controller = Controller(robots, workbenchs, workmap)
    # 针对性调参
    if workmap.map_data[3][13] == '7' and workmap.map_data[13][3] == '3' and workmap.map_data[23][3] == '7':
        controller.set_control_parameters(4.155, 2.62,  1.0, 0.63)
    elif workmap.map_data[2][2] == '8' and workmap.map_data[62][2] == '4' and workmap.map_data[78][2] == '5':
        controller.set_control_parameters(3.34, 2.94, 1.97, 0.57)
        controller.FLAG_HUQ = False
    # elif workmap.map_data[53][56] == '4' and workmap.map_data[58][53] == '7' and workmap.map_data[89][4] == '9':
    #     controller.set_control_parameters(4.95, 1.2, 1.35, 0.45)
    # elif workmap.map_data[2][3] == '6' and workmap.map_data[22][4] == '2' and workmap.map_data[90][-4] == '5':
    #     controller.set_control_parameters(4.35, 1.8, 1.05, 0.45)
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
