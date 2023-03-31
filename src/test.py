# -*-coding:utf-8-*-
# @Time       : 2023/3/31 14:07
# @Author     : Feng Rui
# @Site       : 
# @File       : test.py
# @Software   : PyCharm
# @Description:
import numpy as np
import time

if __name__ == '__main__':
    bool_map = np.random.choice(a=[False, True], size=(100, 100))
    int_map = np.random.choice([0, 100], size=(100, 100))

    T1 = time.time()
    for q in range(100):
        for i in range(100):
            for j in range(100):
                if bool_map[i, j]:
                    a = 100
    T2 = time.time()
    print(f'布尔地图遍历100遍耗时：{T2-T1}s')

    T1 = time.time()
    for q in range(100):
        for i in range(100):
            for j in range(100):
                if int_map[i, j]:
                    a = 100
    T2 = time.time()
    print(f'整数地图遍历100遍耗时：{T2 - T1}s')
    a = 10000000000