import random

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
        
    def update_position(self):
        self.position = [min(max(p + v, x_min[i]), x_max[i]) for i, (p, v) in enumerate(zip(self.position, self.velocity))] # 我不理解！

        
    def update_velocity(self, global_best_position, w, c1, c2):
        r1 = random.random()
        r2 = random.random()
        self.velocity = [w*v + c1*r1*(bp - p) + c2*r2*(gbp - p) 
                         for p, v, bp, gbp in zip(self.position, self.velocity, self.best_position, global_best_position)]
        
    def evaluate_fitness(self, objective_function):
        fitness = objective_function(*self.position)
        if fitness > self.best_fitness:
            self.best_position = self.position
            self.best_fitness = fitness

def particle_swarm_optimization(objective_function, x_min, x_max, v_min, v_max, num_particles, num_iterations, w, c1, c2):
    particles = [Particle(x_min, x_max, v_min, v_max) for _ in range(num_particles)]
    global_best_position = particles[0].position
    global_best_fitness = float('-inf')
    
    for _ in range(num_iterations):
        for particle in particles:
            # 并行
            particle.evaluate_fitness(objective_function)
            if particle.best_fitness > global_best_fitness:
                global_best_position = particle.best_position
                global_best_fitness = particle.best_fitness
        for particle in particles:
            particle.update_velocity(global_best_position, w, c1, c2)
            particle.update_position()
    
    return global_best_position, global_best_fitness


if __name__ == '__main__':
    x_min = [0, 0, 0, 0, 0, 0, 0] # x1到x7的最小值
    x_max = [1, 1, 1, 1, 1, 1, 2] # x1到x7的最大值
    v_min = [-0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1] # 速度的最小值
    v_max = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1] # 速度的最大值
    num_particles = 50 # 粒子数量
    num_iterations = 100 # 迭代次数
    w = 0.5 # 惯性权重
    c1 = 1.5 # 个体学习因子
    c2 = 1.5 # 全局学习因子
    
    global_best_position, global_best_fitness = particle_swarm_optimization(f, x_min, x_max, v_min, v_max, num_particles, num_iterations, w, c1, c2)
    
    # 输出结果
    print("最优参数：", global_best_position)
    print("最优值：", global_best_fitness)