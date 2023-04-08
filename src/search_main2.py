# coding=utf-8
import random
import sys
import numpy as np
from queue import Queue
import os
import time

map_file = "maps/1.txt"
# maps = [ "./maps/2.txt"]
# 参数范围
move_speed = np.arange(4.5, 6, 0.3)
max_wait = np.arange(1.2, 2, 0.1)
sell_weight = np.arange(1, 2, 0.1)
sell_debuff = np.arange(0.4, 1, 0.1)
python = "python"  # 怎么调用python
works = 4  # 最多取cpu个数的一半
import threading

results = Queue()

judgment = "Robot.exe" if sys.platform == "win32" else "./Robot"


def run_a_process(params, idx):
    '''
    启动一个跑分线程调用系统
    :param params: 七个参数
    :param idx: 例子标记
    :return: 标记和分数的tuple
    '''
    cmd = f'{judgment} ' \
          f'"{python} src/main.py ' \
          f'--move_speed {params[0]} ' \
          f'--max_wait_mul {params[1]} ' \
          f'--sell_weight {params[2]} ' \
          f'--sell_debuff {params[3]} ' \
          f'-f -m {map_file}'
    print(cmd)
    res = os.popen(cmd).readlines()[-1]
    score = eval(res)['score']
    # print(res)
    results.put((idx, score))


# 定义要最大化的函数 f
def f(x1, x2, x3, x4, x5, x6, x7):
    # TODO: 实现函数f()的代码
    return -(x1 + x2 + x3 + x4 + x5 + x6 + x7)


import random


class Particle:
    def __init__(self, x_min, x_max, v_min, v_max):
        self.position = [random.uniform(x_min[i], x_max[i]) for i in range(7)]
        self.velocity = [random.uniform(v_min[i], v_max[i]) for i in range(7)]
        self.best_position = self.position
        self.best_fitness = float('-inf')
        self.temp_fitness = float('-inf')

    def update_position(self):
        self.position = [min(max(p + v, x_min[i]), x_max[i]) for i, (p, v) in
                         enumerate(zip(self.position, self.velocity))]  # 我不理解！

    def update_velocity(self, global_best_position, w, c1, c2):
        r1 = random.random()
        r2 = random.random()
        self.velocity = [w * v + c1 * r1 * (bp - p) + c2 * r2 * (gbp - p)
                         for p, v, bp, gbp in
                         zip(self.position, self.velocity, self.best_position, global_best_position)]

    def evaluate_fitness(self):
        fitness = self.temp_fitness
        if fitness > self.best_fitness:
            self.best_position = self.position
            self.best_fitness = fitness


def particle_swarm_optimization(x_min, x_max, v_min, v_max, num_particles, num_iterations, w, c1,
                                c2):
    particles = [Particle(x_min, x_max, v_min, v_max) for _ in range(num_particles)]
    global_best_position = particles[0].position
    global_best_fitness = float('-inf')

    for _ in range(num_iterations):
        running_processes = 0
        end_processes = 0
        idx = 0
        while idx < num_particles:
            print(f"idx:{idx}\n")
            if running_processes < works:
                particle = particles[idx]
                threading.Thread(target=run_a_process, args=(particle.position[:], idx)).start()
                # run_a_process(particle.position[:], idx)
                running_processes += 1
                idx+=1
            else:
                end_idx, score = results.get()
                running_processes -= 1
                end_processes += 1
                particles[end_idx].temp_fitness = score
                print('======================================')
                print(f"params:{particles[end_idx].position}, score:{particles[end_idx].temp_fitness}")
                print(f"best_params:{global_best_position}, best_score:{global_best_fitness}")
                print('======================================')
        while end_processes < num_particles:
            end_idx, score = results.get()
            end_processes += 1
            particles[end_idx].temp_fitness = score
        for particle in particles:
            # 可以加个log

            # 并行
            particle.evaluate_fitness()
            if particle.best_fitness > global_best_fitness:
                global_best_position = particle.best_position
                global_best_fitness = particle.best_fitness
        for particle in particles:
            particle.update_velocity(global_best_position, w, c1, c2)
            particle.update_position()

    return global_best_position, global_best_fitness


if __name__ == '__main__':
    '''
    设置参数， 建议取值范围:
    move_speed: 3-5 估算移动时间
    max_wait: 1-5 最大等待时间
    sell_weight: 1-2 优先卖给格子被部分占用的
    sell_debuff: 0.5-1 将456卖给9的惩罚因子
    thr_dis: 4-8 对撞检测距离
    thr_theta: 4-10 对撞检测角度
    theta_rotate: 4-10 对撞躲避角度
    '''
    x_min = [3, 1, 1, 0.5]  # x1到x7的最小值
    x_max = [5, 5, 2, 1]  # x1到x7的最大值
    v_min = [-0.1, -0.1, -0.1, -0.1]  # 速度的最小值
    v_max = [0.1, 0.1, 0.1, 0.1]  # 速度的最大值
    num_particles = 50  # 粒子数量
    num_iterations = 20  # 迭代次数
    w = 0.5  # 惯性权重
    c1 = 1.5  # 个体学习因子
    c2 = 1.5  # 全局学习因子

    T1 = time.time()
    global_best_position, global_best_fitness = particle_swarm_optimization(x_min, x_max, v_min, v_max,
                                                                            num_particles, num_iterations, w, c1, c2)
    T2 = time.time()
    print(f'耗时{T2-T1}秒')
    # 输出结果
    print("最优参数：", global_best_position)
    print("最优值：", global_best_fitness)
