# coding=utf-8
from robot import Robot
from workbench import Workbench
from typing import Optional, List
'''
控制类 决策，运动
'''


class Controller:
    # 总帧数
    TOTAL_FRAME = 50*60*5
    # 控制参数

    def __init__(self, robots: List[Robot], workbenchs: List[Workbench]):
        self.robots = robots
        self.workbenchs = workbenchs

    def move(self):
        pass

    def control(self, frame_id: int, money: int):
        pass
