# coding=utf-8
import sys
from workmap import Workmap
from robot import Robot
from workbench import Workbench
from controller import Controller
from typing import Optional, List
import os
try:
    os.chdir('./src')
except:
    pass


def read_util_ok():
    while input() != "OK":
        pass


def finish():
    sys.stdout.write('OK\n')
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
    import time
    t1 = time.time()
    # 检测一下地图连通性
    workmap.init_roads()
    # workmap.draw_map()
    for idx, r2w in enumerate(workmap.robot2workbench()):
        robots[idx].target_workbench_list = r2w
    for idx, w2w in enumerate(workmap.workbench2workbench()):
        workbenchs[idx].target_workbench_list = w2w 
    # 计算一下路径
    workmap.gen_paths()
    t2 = time.time()
    print("路径生成时间:", t2-t1)
    # path = workmap.get_path(robots[1].loc, robots[1].target_workbench_list[0])
    # workmap.draw_path(path)
    path = workmap.get_path(robots[1].loc, robots[1].target_workbench_list[0])
    print(path)
    print("查表时间: ", time.time()-t2)
    controller = Controller(robots, workbenchs)
    finish()

    while True:
        frame_id, money = map(int, input().split())
        input()  # 工作台数量
        for workbench in workbenchs:  # 更新工作台
            workbench.update(input())
        for robot in robots:  # 更新机器人
            robot.update(input())
        input()  # 读一个ok
        controller.control()
        finish()
