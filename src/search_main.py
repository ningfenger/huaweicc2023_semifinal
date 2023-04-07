import subprocess
import threading
import os
import json
import argparse
import numpy as np
import threading
import logging
import sys
import time
import csv
from concurrent.futures import ThreadPoolExecutor
from threading import Lock

# 第一步，创建一个logger
logger = logging.getLogger()
logger.setLevel(logging.INFO)  # Log等级总开关

# 第二步，创建一个handler，用于写入日志文件
logfile = '../log.txt'
fh = logging.FileHandler(logfile, mode='a')  # open的打开模式这里可以进行参考
fh.setLevel(logging.INFO)  # 输出到file的log等级的开关

# 第三步，再创建一个handler，用于输出到控制台
ch = logging.StreamHandler()
ch.setLevel(logging.INFO)  # 输出到console的log等级的开关

# 第四步，定义handler的输出格式
formatter = logging.Formatter("%(asctime)s : %(message)s")
fh.setFormatter(formatter)
ch.setFormatter(formatter)

# 第五步，将logger添加到handler里面
logger.addHandler(fh)
logger.addHandler(ch)

maps = ["./maps/1.txt", "./maps/2.txt", "./maps/3.txt", "./maps/4.txt"]
#maps = [ "./maps/2.txt"]
# 参数范围
move_speed = np.arange(4.5, 6, 0.3)
max_wait = np.arange(1.2, 2, 0.1)
sell_weight = np.arange(1, 2, 0.1)
sell_debuff = np.arange(0.4, 1, 0.1)

python = "python"  # 怎么调用python
works = 10  # 最多取cpu个数的一半

loops = 1  # 每个超参的测试次数

judgment = "./Robot.exe" if sys.platform == "win32" else "./Robot"


class Search:
    def __init__(self):
        self.max_score = 0
        self.max_score_param = ""

        self.map_2_score_and_super_param = dict()
        for map in maps:
            self.map_2_score_and_super_param[map] = (0, "")
        self.file_index = 0
        self.mutex = threading.Lock()
        self.pool = ThreadPoolExecutor(works)

        self.open_csv()
        self.is_end = False
        self.is_end_mutex = Lock()

    def __run(self, super_param):
        all_score = 0
        map_2_score = dict()

        for _ in range(loops):
            for map in maps:
                if map not in map_2_score:
                    map_2_score[map] = 0
                proc = subprocess.Popen(
                    [
                        f"{judgment}",
                        f"{python} ./src/main.py %s" % (super_param),
                        "-f",
                        "-m",
                        map
                    ],
                    stdout=subprocess.PIPE
                )
                while True:
                    try:
                        line = proc.stdout.readline()
                        if not line:
                            break
                        result = json.loads(line.rstrip())
                        score = result['score']
                        all_score += score
                        map_2_score[map] = map_2_score[map] + score
                    except Exception as e:
                        print(e)
                        break
        all_score /= loops
        score_data = dict()
        score_data["all_score"] = all_score
        score_data["param"] = super_param
        for map, score in map_2_score.items():
            score_data[map] = score

        self.mutex.acquire()
        logger.info(f"all score={all_score},{super_param}")
        for map, score in map_2_score.items():
            score /= loops
            old_max_score = self.map_2_score_and_super_param[map][0]
            if score > old_max_score:
                self.map_2_score_and_super_param[map] = (score, super_param)
            score, param = self.map_2_score_and_super_param[map]
            logger.info(
                f"{map} max score={score},{param}")
        if all_score > self.max_score:
            self.max_score_param = super_param
            self.max_score = all_score
        logger.info(f"max score score={self.max_score},{self.max_score_param}")

        self.write_score(score_data)
        self.mutex.release()

    def run(self):
        params = [
            move_speed, max_wait, sell_weight, sell_debuff
        ]
        param = []

        def help(index, params, param):
            if index >= len(params):
                super_param = "--move_speed %d --max_wait_mul %d --sell_weight %f --sell_debuff %f" \
                    % (param[0], param[1], param[2], param[3])
                self.pool.submit(Search.__run, self, super_param)
                return
            for i in params[index]:
                param.append(i)
                help(index+1, params, param)
                param.pop(len(param) - 1)
        help(0, params, param)
        self.pool.shutdown(True)
        self.close()

    def open_csv(self):
        self.f = open("./score.csv", 'w', newline='')
        fieldnames = ['param', 'all_score']
        for map in maps:
            fieldnames.append(map)
        self.writer = csv.DictWriter(self.f, fieldnames=fieldnames)
        self.writer.writeheader()

    def close_csv(self):
        self.f.close()

    def close(self):
        self.close_csv()
        self.is_end_mutex.acquire()
        self.is_end = True
        self.is_end_mutex.release()

    def write_score(self, data):
        self.writer.writerow(data)
        self.f.flush()

    def remove_replay(self):
        while True:
            self.is_end_mutex.acquire()
            if self.is_end:
                self.is_end_mutex.release()
                break
            self.is_end_mutex.release()
            replays = os.listdir("./replay")
            for replay in replays:
                replay_path = os.path.join(os.curdir, "replay", replay)
                os.remove(replay_path)
            time.sleep(60)


if __name__ == "__main__":
    os.chdir("..")
    search = Search()
    search.pool.submit(Search.remove_replay, search)

    search.run()
