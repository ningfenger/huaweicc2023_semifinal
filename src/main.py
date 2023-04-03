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
    # workmap.draw_map()
    # print(time.time()-t1)
    # r_idx = 1
    # for target_workbench in robots[r_idx].target_workbench_list:
    #     path = workmap.get_path(robots[r_idx].loc, target_workbench)
    #     workmap.draw_path(path)
    # path = workmap.get_float_path(robots[r_idx].loc, robots[r_idx].target_workbench_list[0])
    # # 测试路径，非正式决策路径
    # robots[r_idx].path = np.array(path)
    # robots[r_idx].status = Robot.MOVE_TO_BUY_STATUS
    # 测试路径，非正式决策路径
    # time.sleep(10)
    controller = Controller(robots, workbenchs, workmap)
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
