# coding=utf-8
import sys
import numpy as np
import random
from robot import Robot
from workbench import Workbench
from workmap import Workmap
from typing import Optional, List
import tools
import math
import copy

'''
控制类 决策，运动
'''


class Controller:
    # 总帧数
    TOTAL_FRAME = 50 * 60 * 5
    # 控制参数
    MOVE_SPEED = 1 / 4 * 50 * 2  # 除以2是因为每个格子0.5, 所以直接用格子数乘以它就好了
    MAX_WAIT = 3 * 50  # 最大等待时间
    SELL_WEIGHT = 1.5  # 优先卖给格子被部分占用的
    SELL_DEBUFF = 0.6  # 非 7 卖给89的惩罚
    CONSERVATIVE = 1 + 1 / MOVE_SPEED * 1  # 保守程度 最后时刻要不要操作
    STARVE_WEIGHT = SELL_WEIGHT

    FRAME_DIFF_TO_DETECT_DEADLOCK = 20  # 单位为帧,一个机器人 frame_now - pre_frame >这个值时开始检测死锁
    FRAME_DIFF = 10  # 单位为帧
    MIN_DIS_TO_DETECT_DEADLOCK = 0.15  # 如果机器人在一个时间段内移动的距离小于这个值,
    MIN_TOWARD_DIF_TO_DETECT_STUCK = np.pi / 30  # 并且角度转动小于这个值,需要进行检测

    # 判断两个机器人是否死锁的距离阈值,单位为米
    MIN_DIS_TO_DETECT_DEADLOCK_BETWEEN_N_N = 0.92  # 两个机器人都未装载物品
    MIN_DIS_TO_DETECT_DEADLOCK_BETWEEN_N_Y = 1.01  # 一个机器人装载物品,一个未装
    MIN_DIS_TO_DETECT_DEADLOCK_BETWEEN_Y_Y = 1.1  # 两个机器人都装载物品
    WILL_CLASH_DIS = 1.5  # 很可能要撞的距离
    WILL_HUQ_DIS = 2.5  # 可能冲突的距离

    # 避让等待的帧数
    AVOID_FRAME_WAIT = 20

    def __init__(self, robots: List[Robot], workbenchs: List[Workbench], m_map: Workmap):
        self.robots = robots
        self.workbenchs = workbenchs
        self.m_map = m_map
        self.m_map_arr = np.array(m_map.map_gray)
        self.starve = {4: 0, 5: 0, 6: 0}  # 当7急需4 5 6 中的一个时, +1 鼓励生产
        # 预防跳帧使用, 接口先留着
        self.buy_list = []  # 执行过出售操作的机器人列表
        self.sell_list = []  # 执行过购买操作的机器人列表

    def dis2target(self, idx_robot):
        idx_workbench = self.robots[idx_robot].target
        w_loc = self.workbenchs[idx_workbench].loc
        r_loc = self.robots[idx_robot].loc
        return np.sqrt((r_loc[0] - w_loc[0]) ** 2 + (r_loc[1] - w_loc[1]) ** 2)

    def detect_deadlock(self, frame):
        # if frame % 10 != 0:
        #     return
        """
        每帧调用一次。当检测到两个机器人卡在一起时,会把机器人的成员变量 is_deadlock 设为True。
        在采取措施解决两个机器人死锁时, call set_robot_state_undeadlock(self,robot_idx, frame)
        把该机器人设为不死锁状态
        @param: frame: 当前的帧数
        """
        for robot in self.robots:
            frame_diff_to_detect = self.FRAME_DIFF_TO_DETECT_DEADLOCK + \
                                   random.randint(5, 30)
            distance = np.sqrt(
                np.sum(np.square(robot.loc_np - robot.pre_position)))
            toward_diff = abs(robot.toward - robot.pre_toward)
            toward_diff = min(toward_diff, 2 * np.pi - toward_diff)
            # 一帧内移动距离大于MIN_DIS_TO_DETECT_DEADLOCK，说明没有死锁，update

            if distance > self.MIN_DIS_TO_DETECT_DEADLOCK or \
                    toward_diff > self.MIN_TOWARD_DIF_TO_DETECT_STUCK:
                robot.update_frame_pisition(frame)
                if robot.is_deadlock or robot.is_stuck:
                    if distance > self.MIN_DIS_TO_DETECT_DEADLOCK:
                        sys.stderr.write(
                            "因为移动解除死锁卡墙" + ",robot_id:" + str(robot.ID) + "\n")
                    else:
                        sys.stderr.write(
                            "因为旋转解除死锁卡墙" + ",robot_id:" + str(robot.ID) + "\n")

                robot.is_deadlock = False
                robot.is_stuck = False
                continue

            if robot.is_deadlock or robot.is_stuck:
                robot.update_frame_pisition(frame)
                continue

            if frame - robot.pre_frame < frame_diff_to_detect:
                continue

            # 50帧内移动距离小于MIN_DIS_TO_DETECT_DEADLOCK

            for robot2 in self.robots:
                if id(robot) == id(robot2):
                    continue

                distance = np.sqrt(
                    np.sum(np.square(robot2.loc_np - robot2.pre_position)))
                toward_diff = abs(robot2.toward - robot2.pre_toward)
                toward_diff = min(toward_diff, 2 * np.pi - toward_diff)

                if distance > self.MIN_DIS_TO_DETECT_DEADLOCK or \
                        toward_diff > self.MIN_TOWARD_DIF_TO_DETECT_STUCK:
                    continue

                if not robot2.is_deadlock and not robot2.is_stuck and \
                        frame - robot2.pre_frame < self.FRAME_DIFF:
                    continue

                deadlock_dis_threshold = None
                if robot.status < Robot.MOVE_TO_SELL_STATUS and robot2.status < Robot.MOVE_TO_SELL_STATUS:
                    deadlock_dis_threshold = self.MIN_DIS_TO_DETECT_DEADLOCK_BETWEEN_N_N
                elif robot.status >= Robot.MOVE_TO_SELL_STATUS and robot2.status >= Robot.MOVE_TO_SELL_STATUS:
                    deadlock_dis_threshold = self.MIN_DIS_TO_DETECT_DEADLOCK_BETWEEN_Y_Y
                else:
                    deadlock_dis_threshold = self.MIN_DIS_TO_DETECT_DEADLOCK_BETWEEN_N_Y

                distance = np.sqrt(
                    np.sum(np.square(robot.loc_np - robot2.loc_np)))

                if distance <= deadlock_dis_threshold:
                    robot2.is_deadlock = True
                    robot.is_deadlock = True
                    robot.update_frame_pisition(frame)
                    robot.deadlock_with = robot2.ID
                    robot2.deadlock_with = robot.ID
                    robot2.update_frame_pisition(frame)
                    sys.stderr.write("检测到死锁" + ",robot_id:" + str(robot.ID) + "\n")
                    sys.stderr.write("检测到死锁" + ",robot_id:" + str(robot2.ID) + "\n")
                    sys.stderr.write("两个机器人距离为:" + str(distance) + "\n")

            if not robot.is_deadlock:
                if robot.status == robot.WAIT_TO_BUY_STATUS or robot.status == robot.WAIT_TO_SELL_STATUS:
                    robot.update_frame_pisition(frame)
                    continue
                robot.is_stuck = True
                sys.stderr.write("检测到卡墙" + ",robot_id:" + str(robot.ID) + "\n")
        sys.stderr.flush()
        # if need_update:
        #     for robot in self.robots:
        #         if not robot.is_deadlock:
        #             robot.pre_frame = frame
        #             robot.pre_position = copy.deepcopy(robot.loc_np)

    def set_robot_state_undeadlock(self, robot_idx, frame):
        """
        当开始做出解除死锁的动作时,调用此函数,把机器人设置为不死锁
        @param: robot_idx 机器人的idx
        @param: frame 当前的帧数
        """
        if robot_idx == 3:
            return
        robot = self.robots[robot_idx]
        robot.update_frame_pisition(frame)
        robot.is_deadlock = False
        robot.is_stuck = False

    def radar(self, idx_robot, d_theta):
        # 当前位置与朝向
        point = self.robots[idx_robot].loc
        theta = self.robots[idx_robot].toward + d_theta
        theta = (theta + math.pi) % (2 * math.pi) - math.pi
        # 当前位置所处的格子
        # raw, col = int(round(point[1] * 2 - 0.5)), int(round(point[0] * 2 - 0.5))
        raw, col = int(point[1] // 0.5), int(point[0] // 0.5)

        # 取出所有边界点
        if theta == 0:
            # 正右
            x_set_all = np.arange(col + 1, min(col + 4, 99), 1) * 0.5
            y_set_all = np.ones_like(x_set_all) * point[1]
        elif theta == math.pi / 2:
            # 正上
            y_set_all = np.arange(raw + 1, min(raw + 4, 99), 1) * 0.5
            x_set_all = np.ones_like(y_set_all) * point[0]
        elif theta == math.pi:
            # 正左
            x_set_all = np.arange(col, max(col - 3, 0), -1) * 0.5
            y_set_all = np.ones_like(x_set_all) * point[1]
        elif theta == -math.pi / 2:
            # 正下
            y_set_all = np.arange(raw, max(raw - 3, 0), -1) * 0.5
            x_set_all = np.ones_like(y_set_all) * point[0]
        else:
            # 其他方向

            # x方向栅格点集
            if -math.pi / 2 < theta < math.pi / 2:
                # 1 4 象限
                x_set_xgrid = np.arange(col + 1, min(col + 4, 99), 1) * 0.5
            else:
                # 2 3 象限
                x_set_xgrid = np.arange(col, max(col - 3, 0), -1) * 0.5
            y_set_xgrid = np.tan(theta) * (x_set_xgrid - point[0]) + point[1]

            # y方向栅格点集
            if 0 < theta < math.pi:
                # 1 2 象限
                y_set_ygrid = np.arange(raw + 1, min(raw + 4, 99), 1) * 0.5

            else:
                # 3 4 象限
                y_set_ygrid = np.arange(raw, max(raw - 3, 0), -1) * 0.5
            x_set_ygrid = 1 / np.tan(theta) * \
                          (y_set_ygrid - point[1]) + point[0]
            x_set_all = np.concatenate((x_set_xgrid, x_set_ygrid))
            y_set_all = np.concatenate((y_set_xgrid, y_set_ygrid))

            # 得到排序后的索引
            idx = np.argsort(y_set_all)
            # 将坐标按照排序后的索引进行排序
            if theta < 0:
                x_set_all = x_set_all[idx]
                y_set_all = y_set_all[idx]
            else:
                x_set_all = x_set_all[idx[::-1]]
                y_set_all = y_set_all[idx[::-1]]

        # 取出所有边界点↑
        x_set_near = x_set_all[:-1]
        x_set_far = x_set_all[1:]

        y_set_near = y_set_all[:-1]
        y_set_far = y_set_all[1:]

        x_set_mid = (x_set_near + x_set_far) / 2
        y_set_mid = (y_set_near + y_set_far) / 2

        mask = np.zeros_like(x_set_mid, dtype=bool)
        mask[(x_set_mid >= 0) & (x_set_mid <= 50) & (
                y_set_mid >= 0) & (y_set_mid <= 50)] = True
        x_set_mid = x_set_mid[mask]
        y_set_mid = y_set_mid[mask]
        idx_ob = -1
        for i_point in range(len(x_set_mid)):
            x = x_set_mid[i_point]
            y = y_set_mid[i_point]
            raw, col = tools.cor2rc(x, y)

            if self.m_map_arr[raw, col] == 0:
                idx_ob = i_point
                break

        if idx_ob == -1:
            return 100
        else:
            # 障碍物距离
            return np.sqrt((x_set_near[idx_ob] - point[0]) ** 2 + (y_set_near[idx_ob] - point[1]) ** 2)

    def could_run(self, loc0, loc1, carry_flag):
        # 位置0到位置1区间是否有符合要求

        # loc0→loc1的距离
        dis = np.sqrt(np.sum((loc1 - loc0) ** 2))

        # loc0→loc1的方向
        theta = np.arctan2(loc1[1] - loc0[1], loc1[0] - loc0[0])

        # loc0所处的格子

        raw, col = int(loc0[1] // 0.5), int(loc0[0] // 0.5)

        max_num = np.ceil(dis / 0.5)
        # 取出所有边界点
        if theta == 0:
            # 正右
            x_set_all = np.arange(
                col + 1, min(col + max_num + 1, 101), 1) * 0.5
            y_set_all = np.ones_like(x_set_all) * loc0[1]
        elif theta == math.pi / 2:
            # 正上
            y_set_all = np.arange(
                raw + 1, min(raw + max_num + 1, 101), 1) * 0.5
            x_set_all = np.ones_like(y_set_all) * loc0[0]
        elif theta == math.pi:
            # 正左
            x_set_all = np.arange(col, max(col - max_num - 1, -1), -1) * 0.5
            y_set_all = np.ones_like(x_set_all) * loc0[1]
        elif theta == -math.pi / 2:
            # 正下
            y_set_all = np.arange(raw, max(raw - max_num - 1, -1), -1) * 0.5
            x_set_all = np.ones_like(y_set_all) * loc0[0]
        else:
            # 其他方向

            # x方向栅格点集
            if -math.pi / 2 < theta < math.pi / 2:
                # 1 4 象限
                x_set_xgrid = np.arange(
                    col + 1, min(col + max_num + 1, 101), 1) * 0.5
            else:
                # 2 3 象限
                x_set_xgrid = np.arange(
                    col, max(col - max_num - 1, -1), -1) * 0.5
            y_set_xgrid = np.tan(theta) * (x_set_xgrid - loc0[0]) + loc0[1]

            # y方向栅格点集
            if 0 < theta < math.pi:
                # 1 2 象限
                y_set_ygrid = np.arange(
                    raw + 1, min(raw + max_num + 1, 101), 1) * 0.5

            else:
                # 3 4 象限
                y_set_ygrid = np.arange(
                    raw, max(raw - max_num - 1, -1), -1) * 0.5
            x_set_ygrid = 1 / np.tan(theta) * \
                          (y_set_ygrid - loc0[1]) + loc0[0]
            x_set_all = np.concatenate((x_set_xgrid, x_set_ygrid))
            y_set_all = np.concatenate((y_set_xgrid, y_set_ygrid))

            # 得到排序后的索引
            idx = np.argsort(y_set_all)
            # 将坐标按照排序后的索引进行排序
            if theta < 0:
                x_set_all = x_set_all[idx]
                y_set_all = y_set_all[idx]
            else:
                x_set_all = x_set_all[idx[::-1]]
                y_set_all = y_set_all[idx[::-1]]

        # 取出所有边界点↑
        x_set_near = x_set_all[:-1]
        x_set_far = x_set_all[1:]

        y_set_near = y_set_all[:-1]
        y_set_far = y_set_all[1:]

        x_set_mid = (x_set_near + x_set_far) / 2
        y_set_mid = (y_set_near + y_set_far) / 2

        dis_set = np.sqrt(
            (x_set_near - loc0[0]) ** 2 + (y_set_near - loc0[1]) ** 2)
        mask = np.zeros_like(x_set_mid, dtype=bool)
        mask[dis_set <= dis] = True
        x_set_mid = x_set_mid[mask]
        y_set_mid = y_set_mid[mask]
        idx_ob = -1
        if carry_flag:
            # 携带物品
            for i_point in range(len(x_set_mid)):
                x = x_set_mid[i_point]
                y = y_set_mid[i_point]
                raw, col = tools.cor2rc(x, y)
                if raw <= -1 or raw >= 100 or col <= -1 or col >= 100 or self.m_map_arr[
                    raw, col] < Workmap.SUPER_BROAD_ROAD:
                    return False
                # if raw <= -1 or raw >= 100 or col <= -1 or col >= 100 or self.m_map_arr[raw, col] == 0 or self.m_map_arr[raw, col] == 2:
                #     # 障碍物
                #     idx_ob = i_point
                #     break
        else:
            for i_point in range(len(x_set_mid)):
                x = x_set_mid[i_point]
                y = y_set_mid[i_point]
                raw, col = tools.cor2rc(x, y)
                if raw <= -1 or raw >= 100 or col <= -1 or col >= 100 or self.m_map_arr[
                    raw, col] < Workmap.BROAD_ROAD or (raw, col) in self.m_map.broad_shifting:
                    return False
                # if raw <= -1 or raw >= 100 or col <= -1 or col >= 100 or self.m_map_arr[raw, col] == 0:
                #     # 障碍物
                #     idx_ob = i_point
                #     break
        return True
        #     # 全程无障碍
        #     return True
        # else:
        #     # 出现障碍
        #     return False

    def obt_detect(self, loc0, loc1):
        # 位置0到位置1区间是否有符合要求

        # loc0→loc1的距离
        dis = np.sqrt(np.sum((loc1 - loc0) ** 2))

        # loc0→loc1的方向
        theta = np.arctan2(loc1[1] - loc0[1], loc1[0] - loc0[0])

        # loc0所处的格子

        raw, col = int(loc0[1] // 0.5), int(loc0[0] // 0.5)

        max_num = np.ceil(dis / 0.5)
        # 取出所有边界点
        if theta == 0:
            # 正右
            x_set_all = np.arange(
                col + 1, min(col + max_num + 1, 101), 1) * 0.5
            y_set_all = np.ones_like(x_set_all) * loc0[1]
        elif theta == math.pi / 2:
            # 正上
            y_set_all = np.arange(
                raw + 1, min(raw + max_num + 1, 101), 1) * 0.5
            x_set_all = np.ones_like(y_set_all) * loc0[0]
        elif theta == math.pi:
            # 正左
            x_set_all = np.arange(col, max(col - max_num - 1, -1), -1) * 0.5
            y_set_all = np.ones_like(x_set_all) * loc0[1]
        elif theta == -math.pi / 2:
            # 正下
            y_set_all = np.arange(raw, max(raw - max_num - 1, -1), -1) * 0.5
            x_set_all = np.ones_like(y_set_all) * loc0[0]
        else:
            # 其他方向

            # x方向栅格点集
            if -math.pi / 2 < theta < math.pi / 2:
                # 1 4 象限
                x_set_xgrid = np.arange(
                    col + 1, min(col + max_num + 1, 101), 1) * 0.5
            else:
                # 2 3 象限
                x_set_xgrid = np.arange(
                    col, max(col - max_num - 1, -1), -1) * 0.5
            y_set_xgrid = np.tan(theta) * (x_set_xgrid - loc0[0]) + loc0[1]

            # y方向栅格点集
            if 0 < theta < math.pi:
                # 1 2 象限
                y_set_ygrid = np.arange(
                    raw + 1, min(raw + max_num + 1, 101), 1) * 0.5

            else:
                # 3 4 象限
                y_set_ygrid = np.arange(
                    raw, max(raw - max_num - 1, -1), -1) * 0.5
            x_set_ygrid = 1 / np.tan(theta) * \
                          (y_set_ygrid - loc0[1]) + loc0[0]
            x_set_all = np.concatenate((x_set_xgrid, x_set_ygrid))
            y_set_all = np.concatenate((y_set_xgrid, y_set_ygrid))

            # 得到排序后的索引
            idx = np.argsort(y_set_all)
            # 将坐标按照排序后的索引进行排序
            if theta < 0:
                x_set_all = x_set_all[idx]
                y_set_all = y_set_all[idx]
            else:
                x_set_all = x_set_all[idx[::-1]]
                y_set_all = y_set_all[idx[::-1]]

        # 取出所有边界点↑
        x_set_near = x_set_all[:-1]
        x_set_far = x_set_all[1:]

        y_set_near = y_set_all[:-1]
        y_set_far = y_set_all[1:]

        x_set_mid = (x_set_near + x_set_far) / 2
        y_set_mid = (y_set_near + y_set_far) / 2

        dis_set = np.sqrt(
            (x_set_near - loc0[0]) ** 2 + (y_set_near - loc0[1]) ** 2)
        mask = np.zeros_like(x_set_mid, dtype=bool)
        mask[dis_set <= dis] = True
        x_set_mid = x_set_mid[mask]
        y_set_mid = y_set_mid[mask]
        idx_ob = -1
        for i_point in range(len(x_set_mid)):
            x = x_set_mid[i_point]
            y = y_set_mid[i_point]
            raw, col = tools.cor2rc(x, y)
            if raw <= -1 or raw >= 100 or col <= -1 or col >= 100 or self.m_map_arr[raw, col] == 0:
                return False
        return True

    def select_target(self, idx_robot):
        robot = self.robots[idx_robot]
        robot_loc_m = np.array(robot.loc).copy()
        path_loc_m = robot.path.copy()

        vec_r2p = robot_loc_m - path_loc_m
        dis_r2p = np.sqrt(np.sum(vec_r2p ** 2, axis=1))
        mask_greater = dis_r2p < 40
        mask_smaller = 0.3 < dis_r2p
        mask = mask_greater & mask_smaller
        path_loc_m = path_loc_m[mask]
        if self.robots[idx_robot].item_type == 0:
            carry_flag = False
        else:
            carry_flag = True

        # robot_loc_m 指向各个点的方向
        theta_set = np.arctan2(
            path_loc_m[:, 1] - robot_loc_m[1], path_loc_m[:, 0] - robot_loc_m[0]).reshape(-1, 1)

        len_path = path_loc_m.shape[0]
        detect_m = np.full(len_path, False)

        for idx_point in range(len_path):
            loc0 = robot_loc_m
            loc1 = path_loc_m[idx_point, :]
            detect_m[idx_point] = self.could_run(loc0, loc1, carry_flag)

        if detect_m.any():
            m_index = np.where(detect_m)[0]
            idx_target = m_index[len(m_index) - 1]
            target_point = path_loc_m[idx_target, :]
            idx_point = np.where(mask)[0][idx_target]
        # elif detect_2.any():
        #     m_index = np.where(detect_2)[0]
        #     idx_target = m_index[len(m_index) - 1]
        #     target_point = path_loc_m[idx_target, :]
        #     idx_point = np.where(mask)[0][idx_target]
        else:
            idx_target = robot.find_temp_tar_idx()
            target_point = robot.path[idx_target, :]
            idx_point = idx_target
        # ,

        return target_point, idx_point

    def select_target_bck(self, idx_robot):
        # 三条线
        robot = self.robots[idx_robot]
        robot_loc_m = np.array(robot.loc).copy()
        path_loc_m = robot.path.copy()

        vec_r2p = robot_loc_m - path_loc_m
        dis_r2p = np.sqrt(np.sum(vec_r2p ** 2, axis=1))
        mask_greater = dis_r2p < 40
        mask_smaller = 0.3 < dis_r2p
        mask = mask_greater & mask_smaller
        path_loc_m = path_loc_m[mask]
        if self.robots[idx_robot].item_type == 0:
            carry_flag = False
            width = 0.46
        else:
            carry_flag = True
            width = 0.54

        # robot_loc_m 指向各个点的方向
        theta_set = np.arctan2(
            path_loc_m[:, 1] - robot_loc_m[1], path_loc_m[:, 0] - robot_loc_m[0]).reshape(-1, 1)

        # 方向左旋90度
        theta_l_set = theta_set + math.pi / 2

        # 方向右旋90度
        theta_r_set = theta_set - math.pi / 2

        robot_loc_l = robot_loc_m + width * \
                      np.concatenate((np.cos(theta_l_set), np.sin(theta_l_set)), axis=1)
        robot_loc_r = robot_loc_m + width * \
                      np.concatenate((np.cos(theta_r_set), np.sin(theta_r_set)), axis=1)

        path_loc_l = path_loc_m + width * \
                     np.concatenate((np.cos(theta_l_set), np.sin(theta_l_set)), axis=1)
        path_loc_r = path_loc_m + width * \
                     np.concatenate((np.cos(theta_r_set), np.sin(theta_r_set)), axis=1)
        len_path = path_loc_m.shape[0]
        detect_m = np.full(len_path, False)
        detect_l = np.full(len_path, False)
        detect_r = np.full(len_path, False)

        for idx_point in range(len_path):
            loc0 = robot_loc_m
            loc1 = path_loc_m[idx_point, :]
            detect_m[idx_point] = self.could_run(loc0, loc1, carry_flag)

            loc0 = robot_loc_l[idx_point, :]
            loc1 = path_loc_l[idx_point, :]
            detect_l[idx_point] = self.could_run(loc0, loc1, carry_flag)

            loc0 = robot_loc_r[idx_point, :]
            loc1 = path_loc_r[idx_point, :]
            detect_r[idx_point] = self.could_run(loc0, loc1, carry_flag)

        detect_all = detect_r & detect_l & detect_m
        detect_2 = detect_m & (detect_l | detect_r)

        if detect_all.any():
            m_index = np.where(detect_all)[0]
            idx_target = m_index[len(m_index) - 1]
            target_point = path_loc_m[idx_target, :]
            idx_point = np.where(mask)[0][idx_target]
        # elif detect_2.any():
        #     m_index = np.where(detect_2)[0]
        #     idx_target = m_index[len(m_index) - 1]
        #     target_point = path_loc_m[idx_target, :]
        #     idx_point = np.where(mask)[0][idx_target]
        else:
            idx_target = robot.find_temp_tar_idx()
            target_point = robot.path[idx_target, :]
            idx_point = idx_target
        # ,

        return target_point, idx_point

    def set_control_parameters(self, move_speed: float, max_wait: int, sell_weight: float, sell_debuff: float):
        # 设置参数
        self.MOVE_SPEED = move_speed  # 估算移动时间
        self.MAX_WAIT = max_wait  # 最大等待时间
        self.SELL_WEIGHT = sell_weight  # 优先卖给格子被部分占用的
        self.SELL_DEBUFF = sell_debuff  # 将456卖给9的惩罚因子

    def path_opt(self, idx_robot):
        for point in self.robots[idx_robot].path:
            row, col = 100 - int(point[1] * 2 - 0.5), int(point[0] * 2 - 0.5)
            a = 100000

    def get_other_col_info(self, idx_robot, idx_other):
        obt_flag = self.obt_detect(np.array(self.robots[idx_robot].loc), np.array(self.robots[idx_other].loc))
        if not obt_flag:
            # 有障碍
            return False, None, None, 0.5, 0.5
        robot_this = self.robots[idx_robot]
        robot_other = self.robots[idx_other]
        vec_o2t = np.array(robot_this.loc) - np.array(robot_other.loc)
        theta_o2t = np.arctan2(vec_o2t[1], vec_o2t[0])
        theta_other = robot_other.toward
        delta_theta = theta_other - theta_o2t
        delta_theta = (delta_theta +
                       math.pi) % (2 * math.pi) - math.pi

        dis = np.sqrt(np.dot(vec_o2t, vec_o2t))
        if dis < 1.07:
            return True, None, None, 0.5, 0.5
        else:
            return False, None, None, 0.5, 0.5
        vx_robot = robot_this.speed[0]
        vy_robot = robot_this.speed[1]
        x_robot = robot_this.loc[0]
        y_robot = robot_this.loc[1]

        vx_other = robot_other.speed[0]
        vy_other = robot_other.speed[1]
        x_other = robot_other.loc[0]
        y_other = robot_other.loc[1]

        # 判断是否路上正向对撞
        col_flag, x_col, y_col, dist_robot, dist_other = tools.will_collide(
            x_robot, y_robot, vx_robot, vy_robot, x_other, y_other, vx_other, vy_other, 0.3)
        # 判断是否路上侧向撞上其他机器人
        # 判断是否同时到终点僵持
        return col_flag, x_col, y_col, dist_robot, dist_other

    def get_other_col_info2(self, idx_robot, idx_other, t_max=0.3):
        # 返回t_max时间内 最短质心距离
        obt_flag = self.obt_detect(np.array(self.robots[idx_robot].loc), np.array(self.robots[idx_other].loc))
        if not obt_flag:
            # 有障碍
            return 100
        robot_this = self.robots[idx_robot]
        robot_other = self.robots[idx_other]
        vec_o2t = np.array(robot_this.loc) - np.array(robot_other.loc)
        theta_o2t = np.arctan2(vec_o2t[1], vec_o2t[0])
        theta_other = robot_other.toward
        delta_theta = theta_other - theta_o2t
        delta_theta = (delta_theta +
                       math.pi) % (2 * math.pi) - math.pi

        vx_robot = robot_this.speed[0]
        vy_robot = robot_this.speed[1]
        x_robot = robot_this.loc[0]
        y_robot = robot_this.loc[1]

        vx_other = robot_other.speed[0]
        vy_other = robot_other.speed[1]
        x_other = robot_other.loc[0]
        y_other = robot_other.loc[1]

        # 判断是否路上正向对撞
        d = tools.will_collide2(
            x_robot, y_robot, vx_robot, vy_robot, x_other, y_other, vx_other, vy_other, t_max)
        # 判断是否路上侧向撞上其他机器人
        # 判断是否同时到终点僵持
        return d

    def avoid_deadlock_ssbb(self, idx_robot, idx_other, thr_theta=math.pi / 5, thr_dis=4):
        # 同买同卖时的虚空锁定 True idx_robot需要减速
        # 不会虚空锁定 False
        robot_this = self.robots[idx_robot]
        robot_other = self.robots[idx_other]
        obt_flag = self.obt_detect(
            np.array(robot_this.loc), np.array(robot_other.loc))
        if not obt_flag:
            # 有障碍
            # 不会虚空锁定
            return False

        if ((robot_this.status == Robot.MOVE_TO_SELL_STATUS and robot_other.status == Robot.MOVE_TO_SELL_STATUS)
            or
            (robot_this.status == Robot.MOVE_TO_BUY_STATUS and robot_other.status == Robot.MOVE_TO_BUY_STATUS)) \
                and \
                robot_this.target == robot_other.target:
            # 都去一个地方卖/买
            dis_this = self.dis2target(idx_robot)
            dis_other = self.dis2target(idx_other)

            # sys.stderr.write(f"dis_this:{dis_this}, dis_other:{dis_other}, dis:{dis}\n")
            if dis_other < dis_this and dis_other < 1.5 and dis_this < 4:
                # sys.stderr.write(f"idx_robot{idx_robot}_avoid\n")
                # 对方离目标点更近
                return True
            else:
                return False
        else:
            return False

    def avoid_deadlock_sb(self, idx_robot, idx_other, thr_theta=math.pi / 3, thr_dis=5):
        # 自己买别人卖时的虚空锁定 True idx_robot需要避让
        # 不会虚空锁定 False
        robot_this = self.robots[idx_robot]
        robot_other = self.robots[idx_other]
        obt_flag = self.obt_detect(
            np.array(robot_this.loc), np.array(robot_other.loc))
        if not obt_flag:
            # 有障碍
            # 不会虚空锁定
            # sys.stderr.write("111111111")
            return False
        # sys.stderr.write("22222\n")
        # sys.stderr.write(f"{robot_this.status}--{robot_other.status}--{robot_this.target}--{robot_other.target}\n")

        if robot_this.status in [Robot.WAIT_TO_BUY_STATUS,
                                 Robot.MOVE_TO_BUY_STATUS] and robot_other.status == Robot.MOVE_TO_SELL_STATUS and robot_this.workbench_ID == robot_other.target:

            # sys.stderr.write("33333\n")
            vec_o2t = np.array(robot_this.loc) - np.array(robot_other.loc)
            theta_o2t = np.arctan2(vec_o2t[1], vec_o2t[0])
            theta_other = robot_other.toward
            delta_theta = theta_other - theta_o2t
            delta_theta = (delta_theta +
                           math.pi) % (2 * math.pi) - math.pi

            dis = np.sqrt(np.dot(vec_o2t, vec_o2t))
            # sys.stderr.write(f"{dis}")
            if abs(delta_theta) < thr_theta and dis < thr_dis:

                return True
            else:
                return False

        else:
            return False

    def AF(self, loc_robot):
        row, col = self.m_map.loc_float2int(loc_robot[0], loc_robot[1])
        row_start = max(row - 2, 0)
        row_stop = min(row + 2, 99)
        col_start = max(col - 2, 0)
        col_stop = min(col + 2, 99)
        offset = np.array([0, 0])
        for i_row in range(row_start, row_stop + 1):
            for i_col in range(col_start, col_stop + 1):
                if self.m_map_arr[i_row, i_col] == 0:
                    # 这是一个障碍物
                    loc_obt = np.array(self.m_map.loc_int2float(i_row, i_col))
                    dis2loc = tools.np_norm(loc_robot, loc_obt)
                    if dis2loc < 1:
                        theta = tools.np_theta(loc_obt, loc_robot)

                        offset = offset + 0.5 * np.array([np.cos(theta), np.cos(theta)])
        # sys.stderr.write(f"势场修正:{offset[0]},{offset[1]}\n")
        return offset

    def obt_near(self, robot):
        row, col = self.m_map.loc_float2int(*robot.loc)
        for row_offset, col_offset in Workmap.TURNS:
            new_row = row + row_offset
            new_col = col + col_offset
            if new_col < 0 or new_col > 99 or new_row < 0 or new_row > 99 or self.m_map.map_gray[new_row][
                new_col] == Workmap.BLOCK:
                return False
        return True

    def obt_near_count(self, robot):
        row, col = self.m_map.loc_float2int(*robot.loc)
        count = 0
        for row_offset, col_offset in Workmap.TURNS:
            new_row = row + row_offset
            new_col = col + col_offset
            if new_col < 0 or new_col > 99 or new_row < 0 or new_col > 99 or self.m_map.map_gray[new_row][
                new_col] == Workmap.BLOCK:
                count += 1
        return count

    def move(self, idx_robot):
        robot = self.robots[idx_robot]
        k_r = 8

        flag_obt_near = self.obt_near(robot)

        # 到工作台距离 用于判定是否接近道路终点
        dis2workbench = self.dis2target(idx_robot)

        # 判定是否有临时目标点
        if self.robots[idx_robot].temp_target is None:
            # 没有临时目标点则重新规划
            self.re_path(robot)
            if flag_obt_near:
                target_loc, target_idx = self.select_target(idx_robot)
            else:
                target_idx = robot.find_temp_tar_idx()
                target_loc = robot.path[target_idx, :]
            robot.temp_target = target_loc
        else:
            # 有临时目标点
            dis_temp_target = np.sqrt(
                np.sum((robot.temp_target - np.array(robot.loc)) ** 2))
            if dis_temp_target > 0.24:
                # 距离大于给定值时 继续追踪
                target_loc = robot.temp_target
            else:
                self.re_path(robot)
                # 足够接近时 重新选择
                if flag_obt_near:
                    target_loc, target_idx = self.select_target(idx_robot)
                else:
                    target_idx = robot.find_temp_tar_idx()
                    target_loc = robot.path[target_idx, :]
                robot.temp_target = target_loc
        # 因为移动过程中可能导致阻塞而避让, 可以解除顶牛, 可能导致HUQ
        col_flag = False
        # 因为买卖而产生的避让
        sb_flag = False
        # 是否要采取保持距离的方式
        sb_safe_dis = False
        d = 100
        # 要避让的机器人序号
        idx_huq = -1
        for idx_other in range(4):
            if not idx_other == idx_robot:
                d = min(self.get_other_col_info2(
                    idx_robot, idx_other), d)
                if d < self.WILL_CLASH_DIS:
                    col_flag = True
                    idx_huq = idx_other
                    break
        robot_target = robot.target
        robot_frame_reman = robot.get_frame_reman()
        # 初始化一个较大值
        other_dis2workbench = self.WILL_HUQ_DIS

        if dis2workbench < self.WILL_HUQ_DIS and not col_flag:
            for idx_other in range(4):
                # 锐总说这不合适吧
                if (not idx_other == idx_robot) and self.robots[idx_other].frame_wait == 0 and robot_target == self.robots[idx_other].target:
                    # 另一个机器人到工作台的距离
                    other_dis2workbench = self.dis2target(idx_other)
                    if other_dis2workbench > self.WILL_HUQ_DIS:
                        continue
                    other_frame_reman = self.robots[idx_other].get_frame_reman()
                    if robot_frame_reman > other_frame_reman:
                        sb_flag = True
                    elif robot_frame_reman == other_frame_reman:
                        if dis2workbench > other_dis2workbench:
                            sb_flag = True
                        elif dis2workbench == other_dis2workbench and idx_robot > idx_other:
                            sb_flag = True
                    if sb_flag:
                        idx_huq = idx_other
                        break
        if sb_flag and dis2workbench > other_dis2workbench:
            sb_safe_dis = True

        if col_flag or (sb_flag and not sb_safe_dis):
            priority_idx = -1
            if col_flag:
                huq_frame_reman = self.robots[idx_huq].get_frame_reman()
                if robot_frame_reman > huq_frame_reman:
                    priority_idx = idx_robot
                elif huq_frame_reman > robot_frame_reman:
                    priority_idx = idx_huq
            else:
                priority_idx = idx_robot
            self.re_path(robot)
            self.re_path(self.robots[idx_huq])
            avoid_idx, avoid_path = self.process_deadlock(
                idx_robot, idx_huq, priority_idx)
            # sys.stderr.write(f"avoid_idx: {avoid_idx}\n")
            if avoid_idx == -1:
                # sys.stderr.write(
                #     f"REVERSE idx_robot: {idx_robot}\n")
                # 如果出现可能有坑 一个机器人堵了两个机器人
                sb_safe_dis = True
                pass
            elif avoid_idx == idx_robot:
                # sys.stderr.write(f"idx_robot{idx_robot}, robot.item{robot.item_type}, avoid_path{avoid_path}\n")
                self.robots[idx_robot].set_path(avoid_path)
                self.robots[idx_robot].frame_wait = self.AVOID_FRAME_WAIT
                # sys.stderr.write(f"idx_robot: {idx_robot}\n")
                flag_obt_near = self.obt_near(robot)
                if flag_obt_near:
                    target_loc, target_idx = self.select_target(
                        idx_robot)
                else:
                    target_idx = robot.find_temp_tar_idx()
                    target_loc = robot.path[target_idx, :]
                robot.temp_target = target_loc

        # 根据给定目标点修正
        target_vec = [target_loc[0] - robot.loc[0],
                      target_loc[1] - robot.loc[1]]
        dis_target = np.sqrt(np.dot(target_vec, target_vec))

        target_theta = np.arctan2(
            target_vec[1], target_vec[0])

        robot_theta = self.robots[idx_robot].toward
        delta_theta = target_theta - robot_theta

        delta_theta = (delta_theta +
                       math.pi) % (2 * math.pi) - math.pi
        if sb_safe_dis:
            # 保持安全车距等待买卖
            print("forward", idx_robot, (d - 3) * 6)
        elif abs(delta_theta) > math.pi * 5 / 6 and dis_target < 2:
            # 角度相差太大倒车走
            print("forward", idx_robot, -2)
            # delta_theta += math.pi
        elif abs(delta_theta) > math.pi / 6:
            # 角度相差较大 原地转向
            print("forward", idx_robot, 0)
        elif dis2workbench < 1.5:

            print("forward", idx_robot, dis2workbench * 6)

        else:
            print("forward", idx_robot, (dis_target) * 10)

        delta_theta = (delta_theta +
                       math.pi) % (2 * math.pi) - math.pi

        self.robots[idx_robot].rotate(delta_theta * k_r)

        if idx_robot == 3:
            a = 10000000000000
            a = 10000000000000
        pass

    def get_time_rate(self, frame_sell: float) -> float:
        # 计算时间损失
        if frame_sell >= 9000:
            return 0.8
        sqrt_num = math.sqrt(1 - (1 - frame_sell / 9000) ** 2)
        return (1 - sqrt_num) * 0.2 + 0.8

    def choise(self, frame_id: int, robot: Robot) -> bool:
        # 进行一次决策
        max_radio = 0  # 记录最优性价比
        for idx_workbench_to_buy in robot.target_workbench_list:
            workbench_buy = self.workbenchs[idx_workbench_to_buy]
            if workbench_buy.product_time == -1 and workbench_buy.product_status == 0 or workbench_buy.product_pro == 1:  # 被预定了,后序考虑优化
                continue
            # 生产所需时间，如果已有商品则为0
            frame_wait_buy = workbench_buy.product_time if workbench_buy.product_status == 0 else 0
            # if frame_wait_buy > self.MAX_WAIT: 由于移动时间更长了，所以直接生产等待时间比较不合理
            #     continue
            frame_move_to_buy = len(self.m_map.get_path(
                robot.loc, idx_workbench_to_buy)) * self.MOVE_SPEED
            if frame_wait_buy - frame_move_to_buy > self.MAX_WAIT:  # 等待时间超出移动时间的部分才有效
                continue
            # 需要这个产品的工作台
            for idx_workbench_to_sell in workbench_buy.target_workbench_list:
                workbench_sell = self.workbenchs[idx_workbench_to_sell]
                if workbench_sell.check_material_pro(workbench_buy.typeID):
                    continue
                # 格子里有这个原料
                # 判断是不是8或9 不是8或9 且这个原料格子已经被占用的情况, 生产完了并不一定能继续生产
                frame_wait_sell = 0
                if workbench_sell.check_material(workbench_buy.typeID):
                    continue
                    # 阻塞或者材料格没满
                    if workbench_sell.product_time in [-1, 0] or not workbench_sell.check_materials_full():
                        continue
                    elif workbench_sell.product_status == 1:  # 到这里说明材料格和产品格都满了不会再消耗原料格了
                        continue
                    else:
                        frame_wait_sell = workbench_sell.product_time
                # frame_move_to_buy, frame_move_to_sell= self.get_time_rww(idx_robot, idx_workstand, idx_worksand_to_sell)
                frame_move_to_sell = len(self.m_map.get_path(workbench_buy.loc, idx_workbench_to_sell,
                                                             True)) * self.MOVE_SPEED
                frame_buy = max(frame_move_to_buy, frame_wait_buy)  # 购买时间
                frame_sell = max(frame_move_to_sell,
                                 frame_wait_sell - frame_buy)  # 出售时间
                total_frame = frame_buy + frame_sell  # 总时间
                if total_frame * self.CONSERVATIVE + frame_id > self.TOTAL_FRAME:  # 完成这套动作就超时了
                    continue
                time_rate = self.get_time_rate(
                    frame_move_to_sell)  # 时间损耗
                # sell_weight = self.SELL_WEIGHT**workbench_sell.get_materials_num # 已经占用的格子越多优先级越高
                sell_weight = self.SELL_WEIGHT if workbench_sell.material else 1  # 已经占用格子的优先级越高
                sell_debuff = self.SELL_DEBUFF if workbench_sell.typeID == 9 and workbench_sell.typeID != 7 else 1
                strave_wight = self.STARVE_WEIGHT if self.starve.get(workbench_sell.typeID, 0) > 0 else 1  # 鼓励生产急需商品
                radio = (workbench_buy.sell_price * time_rate -
                         workbench_buy.buy_price) / total_frame * sell_weight * sell_debuff * strave_wight
                # sys.stderr.write(f"radio:{radio} strave_wight{strave_wight}\n")
                if radio > max_radio:
                    max_radio = radio
                    robot.set_plan(idx_workbench_to_buy, idx_workbench_to_sell)
                    robot.frame_reman_buy = frame_buy
                    robot.frame_reman_sell = frame_sell
        if max_radio > 0:  # 开始执行计划
            # 设置机器人移动目标
            idx_workbench_to_buy = robot.get_buy()
            idx_workbench_to_sell = robot.get_sell()
            robot.target = idx_workbench_to_buy
            robot.set_path(self.m_map.get_float_path(
                robot.loc, idx_workbench_to_buy))
            # 预定工作台
            self.workbenchs[idx_workbench_to_buy].pro_buy()
            self.workbenchs[idx_workbench_to_sell].pro_sell(
                self.workbenchs[idx_workbench_to_buy].typeID)
            return True
        return False

    def re_path(self, robot):
        '''
        为机器人重新规划路劲
        '''
        if robot.status in [Robot.MOVE_TO_BUY_STATUS, Robot.WAIT_TO_BUY_STATUS]:
            # 重新规划路径
            robot.set_path(self.m_map.get_float_path(
                robot.loc, robot.get_buy()))
            robot.status = Robot.MOVE_TO_BUY_STATUS
        elif robot.status in [Robot.MOVE_TO_SELL_STATUS, Robot.WAIT_TO_SELL_STATUS]:
            robot.set_path(self.m_map.get_float_path(
                robot.loc, robot.get_sell(), True))
            robot.status = Robot.MOVE_TO_SELL_STATUS

    def process_deadlock(self, robot1_idx, robot2_idx, priority_idx=-1, safe_dis: float = None):
        '''
        处理死锁
        robot1_idx, robot2_idx 相互死锁的两个机器人ID
        priority_idx 优先要避让的机器人ID
        返回 避让的id及路径
        如果都无路可退 id为-1 建议让两个直接倒车
        '''
        robot1, robot2 = self.robots[robot1_idx], self.robots[robot2_idx]
        other_locs = []  # 记录另外两个机器人的坐标
        for idx, robot in enumerate(self.robots):
            if idx not in [robot1_idx, robot2_idx]:
                other_locs.append(robot.loc)
        # 机器人1的退避路径
        avoid_path1 = self.m_map.get_avoid_path(
            robot1.loc, robot2.path, other_locs + [robot2.loc], robot1.item_type != 0, safe_dis)
        # 机器人2的退避路径
        avoid_path2 = self.m_map.get_avoid_path(
            robot2.loc, robot1.path, other_locs + [robot1.loc], robot2.item_type != 0, safe_dis)
        # 记录一下要避让的机器人
        avoid_robot = -1
        avoid_path = []
        if priority_idx == robot1_idx and avoid_path1:
            return robot1_idx, avoid_path1
        elif priority_idx == robot2_idx and avoid_path2:
            return robot2_idx, avoid_path2
        if not avoid_path1 and avoid_path2:
            avoid_robot = robot2_idx
            avoid_path = avoid_path2
        elif not avoid_path2 and avoid_path1:
            avoid_robot = robot1_idx
            avoid_path = avoid_path1
        elif avoid_path1 and avoid_path2:
            # 1 要走得路多，2让
            if len(avoid_path1) > len(avoid_path2) or (
                    len(avoid_path1) == len(avoid_path2) and robot1_idx < robot2_idx):
                avoid_robot = robot2_idx
                avoid_path = avoid_path2
            else:
                avoid_robot = robot1_idx
                avoid_path = avoid_path1
        # sys.stderr.write(f"avoid_robot: {avoid_robot} avoid_path{avoid_path}\n")
        return avoid_robot, avoid_path

    def control(self, frame_id: int, money: int):
        # self.process_deadlock(frame_id)
        print(frame_id)
        sell_out_list = []  # 等待处理预售的机器人列表
        idx_robot = 0
        while idx_robot < 4:
            robot = self.robots[idx_robot]
            robot_status = robot.status
            # if idx_robot == 1:
            #     sys.stderr.write(f"idx_robot: {idx_robot} robot_loc{robot.loc} temp_target{robot.temp_target}\n")
            if robot_status == Robot.FREE_STATUS:
                # 判断是否出现了因跳帧导致的出售失败, 不太对, 预售预购的处理
                # if robot.item_type != 0:
                #     robot.status = Robot.MOVE_TO_SELL_STATUS
                #     continue
                # 【空闲】执行调度策略
                if self.choise(frame_id, robot):
                    robot.status = Robot.MOVE_TO_BUY_STATUS
                    continue
            elif robot_status == Robot.MOVE_TO_BUY_STATUS:
                # 【购买途中】
                self.move(idx_robot)
                # 判断距离是否够近

                # 判定是否进入交互范围
                if robot.workbench_ID == robot.target:
                    # 记录任务分配时的时间 距离 角度相差
                    robot.status = Robot.WAIT_TO_BUY_STATUS  # 切换为 【等待购买】
                    continue
            elif robot_status == Robot.WAIT_TO_BUY_STATUS:
                # 【等待购买】
                self.move(idx_robot)
                idx_workbench_to_buy = robot.get_buy()
                product_status = self.workbenchs[idx_workbench_to_buy].product_status
                # self.pre_rotate(idx_robot, next_walkstand)
                # 如果在等待，提前转向
                if product_status == 1:  # 这里判定是否生产完成可以购买
                    # 可以购买
                    if robot.buy():  # 防止购买失败
                        # 取消预售
                        self.workbenchs[idx_workbench_to_buy].pro_buy(False)
                        idx_workbench_to_sell = robot.get_sell()
                        robot.target = idx_workbench_to_sell  # 更新目标到卖出地点
                        robot.status = Robot.MOVE_TO_SELL_STATUS  # 切换为 【出售途中】
                        robot.set_path(self.m_map.get_float_path(
                            robot.loc, idx_workbench_to_sell, True))
                        continue
                    else:
                        robot.status = Robot.MOVE_TO_BUY_STATUS
                        robot.set_path(self.m_map.get_float_path(
                            robot.loc, robot.get_buy()))
                        continue
                # else:
                # robot.forward(0)
                # robot.rotate(0)
            elif robot_status == Robot.MOVE_TO_SELL_STATUS:
                # 【出售途中】
                # 判断是否出现了因跳帧导致的购买失败
                # if robot.item_type == 0:
                #     robot.status = Robot.MOVE_TO_BUY_STATUS
                #     continue
                # 移动
                self.move(idx_robot)
                # 判断距离是否够近

                # 判定是否进入交互范围
                if robot.workbench_ID == robot.target:
                    robot.status = Robot.WAIT_TO_SELL_STATUS  # 切换为 【等待出售】
                    # logging.debug(f"{idx_robot}->ready to sell")
                    # 记录任务分配时的时间 距离 角度相差
                    # 记录任务分配时的时间 距离 角度相差
                    continue

            elif robot_status == Robot.WAIT_TO_SELL_STATUS:
                # 【等待出售】
                self.move(idx_robot)
                idx_workbench_to_sell = robot.get_sell()
                workbench_sell = self.workbenchs[idx_workbench_to_sell]
                # 如果在等待，提前转向
                # raise Exception(F"{material},{material_type}")
                # 这里判定是否生产完成可以出售 不是真的1
                if not Workbench.WORKSTAND_OUT[workbench_sell.typeID] or not workbench_sell.check_material(
                        robot.item_type):
                    # 可以出售
                    if robot.sell():  # 防止出售失败
                        # 维护一下急需列表
                        # 7且不等于0 说明有材料且没满
                        if workbench_sell.typeID == 7 and workbench_sell.material != 0:
                            # sys.stderr.write(f"material: {workbench_sell.material}\n")
                            if workbench_sell.material in Workbench.WORKSTAND_STARVE:  # 就差一个了
                                self.starve[robot.item_type] -= 1
                            else:
                                self.starve[Workbench.WORKSTAND_STARVE[workbench_sell.material + (
                                        1 << robot.item_type)]] += 1
                            # sys.stderr.write(f"material: {self.starve}\n")
                        # 取消预定
                        sell_out_list.append(idx_robot)
                        robot.status = Robot.FREE_STATUS
                        # 这个机器人不能再决策了, 不然目标更新，状态会被清错
                        # continue
                        # logging.debug(f"{idx_robot}->wait")
                    else:
                        robot.status = Robot.MOVE_TO_SELL_STATUS  # 购买失败说明位置不对，切换为 【出售途中】
                        robot.set_path(self.m_map.get_float_path(
                            robot.loc, idx_workbench_to_sell, True))
                        continue
            # 根据状态更新预估时间
            robot.update_frame_reman()
            idx_robot += 1
        for idx_robot in sell_out_list:
            robot = self.robots[idx_robot]
            workbench_sell = self.workbenchs[robot.get_sell()]
            workbench_sell.pro_sell(robot.item_type, False)
