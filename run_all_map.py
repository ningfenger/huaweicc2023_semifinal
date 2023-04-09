# coding=utf-8

import os
from sys import platform
from queue import Queue
import threading

results = Queue()
Robot = 'Robot.exe'
if platform == 'linux':
    Robot = './Robot'


def run_a_map(cmd, map_ID):
    res = os.popen(cmd).readlines()[-1]
    score = eval(res)['score']
    results.put((map_ID, score))


if __name__ == '__main__':
    total_score = 0
    for map_ID in range(1, 3):
        cmd = f'{Robot} "python src/main.py" -f -m maps/{map_ID}.txt'
        threading.Thread(target=run_a_map, args=(cmd, map_ID)).start()
    for _ in range(2):
        map_ID, score = results.get()
        print(f"地图{map_ID}得分为: {score}")
        total_score += score
    print(f'总得分为: {total_score}')
