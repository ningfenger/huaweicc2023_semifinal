# coding=utf-8

import os
from sys import platform

Robot = 'Robot.exe'
if platform == 'linux':
    Robot = './Robot'

total_score = 0
for map_ID in range(1,5):
    cmd = f'{Robot} "python src/main.py" maps/{map_ID}.txt'
    res = os.popen(cmd).readlines()[-1]
    score = eval(res)['score']
    total_score += score
    print(f'地图{map}得分为: {score}')
print(f'总得分为: {total_score}')

