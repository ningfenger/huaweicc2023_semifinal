# coding=utf-8
import sys
from workmap import Workmap
from robot import Robot
from workbench import Workbench
from controller import Controller
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
    workmap = Workmap()
    robots = []  # 机器人列表
    workbenchs = []  # 工作台列表
    # 读入初始化地图
    for t, loc in workmap.read_map():
        if t == 'A':
            robots.append(Robot(len(robots), loc))
        else:
            workbenchs.append(Workbench(len(workbenchs), int(t), loc))
    # workmap.draw_map()
    # 检测一下地图连通性
    pass
    # 计算一下路径
    pass
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
