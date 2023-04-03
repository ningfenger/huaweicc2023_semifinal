# coding=utf-8
import sys
import numpy as np

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
    MOVE_SPEED = 1 / 2 * 50 * 2  # 除以2是因为每个格子0.5, 所以直接用格子数乘以它就好了
    MAX_WAIT = 3.14 * 50  # 最大等待时间
    SELL_WEIGHT = 1.5  # 优先卖给格子被部分占用的
    SELL_DEBUFF = 0.8  # 非 7 卖给89的惩罚
    CONSERVATIVE = 1 + 1 / MOVE_SPEED * 4  # 保守程度 最后时刻要不要操作

    FRAME_DIFF_TO_DETECT_DEADLOCK = 30  # 单位为帧,一个机器人 frame_now - pre_frame >这个值时开始检测死锁
    FRAME_DIFF = 25  # 单位为帧
    MIN_DIS_TO_DETECT_DEADLOCK = 0.1  # 如果机器人在一个时间段内移动的距离小于这个值,需要进行检测

    # 判断两个机器人是否死锁的距离阈值,单位为米
    MIN_DIS_TO_DETECT_DEADLOCK_BETWEEN_N_N = 0.92  # 两个机器人都未装载物品
    MIN_DIS_TO_DETECT_DEADLOCK_BETWEEN_N_Y = 1.01  # 一个机器人装载物品,一个未装
    MIN_DIS_TO_DETECT_DEADLOCK_BETWEEN_Y_Y = 1.1  # 两个机器人都装载物品

    def __init__(self, robots: List[Robot], workbenchs: List[Workbench], m_map: Workmap):
        self.robots = robots
        self.workbenchs = workbenchs
        self.m_map = m_map
        self.m_map_arr = np.array(m_map.map_gray)

    def dis2target(self, idx_robot):
        idx_workbench = self.robots[idx_robot].target
        w_loc = self.workbenchs[idx_workbench].loc
        r_loc = self.robots[idx_robot].loc
        return np.sqrt((r_loc[0] - w_loc[0])**2 + (r_loc[1] - w_loc[1])**2)

    def detect_deadlock(self, frame):
        """
        每帧调用一次。当检测到两个机器人卡在一起时,会把机器人的成员变量 is_deadlock 设为True。
        在采取措施解决两个机器人死锁时, call set_robot_state_undeadlock(self,robot_idx, frame)
        把该机器人设为不死锁状态
        @param: frame: 当前的帧数
        """
        for robot in self.robots:
            # if robot.pre_frame == -1:
            #     robot.pre_frame = frame
            #     robot.pre_position = copy.deepcopy(robot.loc_np)
            #     continue
            distance = np.sqrt(
                np.sum(np.square(robot.loc_np - robot.pre_position)))

            # 一帧内移动距离大于MIN_DIS_TO_DETECT_DEADLOCK，说明没有死锁，update

            if distance > self.MIN_DIS_TO_DETECT_DEADLOCK:
                robot.update_frame_pisition(frame)
                robot.is_deadlock = False
                continue

            if robot.is_deadlock:
                robot.update_frame_pisition(frame)
                continue

            if frame - robot.pre_frame < self.FRAME_DIFF_TO_DETECT_DEADLOCK:
                continue

            # 30帧内移动距离小于MIN_DIS_TO_DETECT_DEADLOCK

            for robot2 in self.robots:
                if id(robot) == id(robot2):
                    continue

                distance = np.sqrt(
                    np.sum(np.square(robot2.loc_np - robot2.pre_position)))
                if distance > self.MIN_DIS_TO_DETECT_DEADLOCK:
                    continue

                if not robot2.is_deadlock and frame - robot2.pre_frame < self.FRAME_DIFF:
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
                    robot2.update_frame_pisition(frame)
                    sys.stderr.write("检测到死锁"+",robot_id:"+str(robot.ID)+"\n")
                    sys.stderr.write("检测到死锁"+",robot_id:"+str(robot2.ID)+"\n")
                    sys.stderr.write("两个机器人距离为:"+str(distance)+"\n")

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
        robot = self.robots[robot_idx]
        robot.update_frame_pisition(frame)
        robot.is_deadlock = False

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
            return np.sqrt((x_set_near[idx_ob] - point[0]) ** 2 + (y_set_near[idx_ob] - point[1]) ** 2)

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

    def move(self, idx_robot):
        # 机器人沿着指定路线移动
        k_r = 8
        dis_l = self.radar(idx_robot, math.pi / 3)
        dis_r = self.radar(idx_robot, -math.pi / 3)
        far_flag = False
        if self.dis2target(idx_robot) > 2:
            far_flag = True
        target_loc_local, target_loc_further = self.robots[idx_robot].find_temp_tar(
        )

        target_vec_local = [target_loc_local[0] - self.robots[idx_robot].loc[0],
                            target_loc_local[1] - self.robots[idx_robot].loc[1]]
        target_theta_local = np.arctan2(
            target_vec_local[1], target_vec_local[0])

        target_vec_further = [target_loc_further[0] - self.robots[idx_robot].loc[0],
                              target_loc_further[1] - self.robots[idx_robot].loc[1]]
        target_theta_further = np.arctan2(
            target_vec_further[1], target_vec_local[0])

        robot_theta = self.robots[idx_robot].toward
        delta_theta_local = target_theta_local - robot_theta
        # safe_dis = 1.2
        # if abs(delta_theta_local) < math.pi / 6:
        #     if dis_l < safe_dis and far_flag:
        #         delta_theta_local -= math.pi * (safe_dis - dis_l) / 5
        #     if dis_r < safe_dis and far_flag:
        #         delta_theta_local += math.pi * (safe_dis - dis_l) / 5
        delta_theta_local = (delta_theta_local +
                             math.pi) % (2 * math.pi) - math.pi
        delta_theta_further = target_theta_further - robot_theta
        delta_theta_further = (delta_theta_further +
                               math.pi) % (2 * math.pi) - math.pi

        self.robots[idx_robot].rotate(delta_theta_local * k_r)
        if abs(delta_theta_local) > math.pi / 4:
            print("forward", idx_robot, 0)
        elif abs(delta_theta_further) > math.pi / 4:
            print("forward", idx_robot, 3)
        else:
            print("forward", idx_robot, 6)
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
                radio = (workbench_buy.sell_price * time_rate -
                         workbench_buy.buy_price) / total_frame * sell_weight * sell_debuff
                if radio > max_radio:
                    max_radio = radio
                    robot.set_plan(idx_workbench_to_buy, idx_workbench_to_sell)
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

    def control(self, frame_id: int, money: int):
        # 在这里执行冲突检测和化解并记得记录上一个机器人的状态
        # 如果冲突无法化解，让每个机器人都倒一下车
        # self.detect_deadlock(frame_id)

        print(frame_id)
        sell_out_list = []  # 等待处理预售的机器人列表
        idx_robot = 0
        while idx_robot < 4:
            robot = self.robots[idx_robot]
            robot_status = robot.status
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
                else:
                    robot.forward(0)
                    robot.rotate(0)
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
                idx_workbench_to_sell = robot.get_sell()
                workbench_sell = self.workbenchs[idx_workbench_to_sell]
                # 如果在等待，提前转向
                # raise Exception(F"{material},{material_type}")
                # 这里判定是否生产完成可以出售 不是真的1
                if not Workbench.WORKSTAND_OUT[workbench_sell.typeID] or not workbench_sell.check_material(
                        robot.item_type):
                    # 可以出售
                    if robot.sell():  # 防止出售失败
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
                else:
                    robot.forward(0)
                    robot.rotate(0)
            elif robot_status == Robot.AVOID_CLASH:
                # 距离足够近则取消冲突避免状态
                if (robot.loc[0]-robot.path[-1][0])**2 + (robot.loc[1]-robot.path[-1][1])**2 < 0.05:
                    if robot.last_status in [Robot.MOVE_TO_BUY_STATUS, Robot.WAIT_TO_BUY_STATUS]:
                        # 重新规划路径
                        robot.set_path(self.m_map.get_float_path(robot.loc, robot.get_buy()))
                        robot.status = Robot.MOVE_TO_BUY_STATUS
                    elif robot.last_status in [Robot.MOVE_TO_SELL_STATUS, Robot.WAIT_TO_SELL_STATUS]:
                        robot.set_path(self.m_map.get_float_path(robot.loc, robot.get_sell()))
                        robot.status = Robot.MOVE_TO_SELL_STATUS
                    else:
                        robot.status = robot.last_status
                    continue
                else:
                    self.move(idx_robot)
            idx_robot += 1
        for idx_robot in sell_out_list:
            robot = self.robots[idx_robot]
            workbench_sell = self.workbenchs[robot.get_sell()]
            workbench_sell.pro_sell(robot.item_type, False)
