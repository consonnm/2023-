#!/bin/bash
import sys
import numpy as np
import time

def read_util_ok():
    while input() != "OK":
        pass


def finish():
    sys.stdout.write('OK\n')
    sys.stdout.flush()


# 比赛状态类
class GameStatus:
    def __init__(self):
        # 比赛状态
        self.map = []
        self.frame_id = 1
        self.money = 200000
        self.workbench_count = 0
        self.workbench = []     # 工作台
        self.buy_workbench = [] # 可购买的工作台的索引
        self.sell_workbench = []    # 可卖出的工作台的索引
        self.robot = [Robot(0),Robot(1),Robot(2),Robot(3)]

    # 读取初始化地图 是一个 100 行 100 列的字符矩阵、
    # ‘.’ ： 空地。
    # ‘A’ ： 机器人起始位置，总共 4 个。
    # 数字 1-9：工作台 1-9，总个数<=50，注意同一类工作台可能多次出现
    # 遍历地图 找到工作台的位置
    def read_map(self):
        for i in range(100):
            line = input()
            self.map.append(line)
            for j in range(100):
                if line[j] != '.' and line[j] != 'A':
                    self.workbench.append([int(line[j]), float(j), float(i), -1, 0, 0])
        for i in range(len(self.workbench)):
            if self.workbench[i][0] >= 1 and self.workbench[i][0] <= 7:
                        self.buy_workbench.append(i)
            if self.workbench[i][0] >= 4 and self.workbench[i][0] <= 9:
                        self.sell_workbench.append(i)

    # 读取帧
    def read_frame(self):
        # 第一行输入 2 个整数，表示帧序号（从 1 开始递增）、 当前金钱数。
        line = input()
        parts = line.split(' ')
        self.frame_id = int(parts[0])
        self.money = int(parts[1])
        # 第二行输入 1 个整数，表示场上工作台的数量 K（K<=50）。
        self.workbench_count = int(input())
        # 紧接着 K 行数据，每一行表示一个工作台， 分别由如下所示的数据构成，共计 6 个数字：
        for i in range(self.workbench_count):
            line = input()
            parts = line.split(' ')
            # 工作台类型 整数 范围[1, 9]
            # 坐标 2 个浮点 x,y 无
            # 剩余生产时间 （帧数）整数  -1： 表示没有生产。0：表示生产因输出格满而阻塞。>=0： 表示剩余生产帧数。
            # 原材料格状态 整数 二进制位表描述，例如 48(110000) 表示拥有物品 4 和 5。
            # 产品格状态 整数  0： 表示无。 1：表示有。
            # 修改工作台的状态
            self.workbench[i] = [int(parts[0]), float(parts[1]), float(parts[2]), int(parts[3]), int(parts[4]), int(parts[5])]
            self.read_workbench()
        # 接下来的 4 行数据，每一行表示一个机器人，分别由如下表格中所示的数据构成，每行 10 个数字。
        for i in range(4):
            k_speed = 10
            bias = 200
            line = input()
            parts = line.split(' ')
            self.robot[i].update(int(parts[0]), int(parts[1]), float(parts[2]), float(parts[3]), float(parts[4]), [float(parts[5]), float(parts[6])], float(parts[7]), [float(parts[8]), float(parts[9])])
            if self.robot[i].flag == 1:
                # 计算机器人的路径长度
                length = (np.sqrt((self.robot[i].position[0] - self.workbench[self.robot[i].target_buy_workbench][1]) ** 2 + (self.robot[i].position[1] - self.workbench[self.robot[i].target_buy_workbench][1]) ** 2) + np.sqrt((self.workbench[self.robot[i].target_sell_workbench][1] - self.workbench[self.robot[i].target_buy_workbench][1]) ** 2 + (self.workbench[self.robot[i].target_sell_workbench][2] - self.workbench[self.robot[i].target_buy_workbench][2]) ** 2)) 
                self.robot[i].remain_frames = length * k_speed + bias 
            if self.robot[i].flag == -1 or self.robot[i].flag == 0:
                self.robot[i].remain_frames = 0
            if self.robot[i].flag == 2:
                # 计算机器人的路径长度
                length = np.sqrt((self.robot[i].position[0] - self.workbench[self.robot[i].target_sell_workbench][1]) ** 2 + (self.robot[i].position[1] - self.workbench[self.robot[i].target_sell_workbench][1]) ** 2)
                self.robot[i].remain_frames = length * 10

    # 读取可购买的工作台和可卖出的工作台
    # 可购买：工作台类型为 1-7 产品格状态为 1 不被机器人路径占用
    # 可卖出：工作台类型为 4-9 原材料格状态在机器人规划中判断 不被机器人路径占用
    def read_workbench(self):
        self.buy_workbench = []
        self.sell_workbench = []
        used_buy_workbench = []
        used_sell_workbench = []
        for j in range(len(self.robot)):
            used_buy_workbench.append(self.robot[j].target_buy_workbench)
            used_sell_workbench.append(self.robot[j].target_sell_workbench)

        for i in range(len(self.workbench)):
            if self.workbench[i][0] >= 1 and self.workbench[i][0] <= 7 and self.workbench[i][5] == 1:
                if i not in used_buy_workbench:
                    self.buy_workbench.append(i)
            if self.workbench[i][0] >= 4 and self.workbench[i][0] <= 9:
                if i not in used_sell_workbench:
                    self.sell_workbench.append(i)


    # 单个机器人的动态规划策略
    def robot_dynamic_planning(self, robot):
        # 找到一条购买卖出的最高价值路径 
        find_target = False
        robot.target_buy_workbench = -1
        robot.target_sell_workbench = -1
        max_value = 0
        value = 0
        for i in self.buy_workbench:
            if self.workbench[i][5] == 1:   # 产品格状态为1 可购买
                product = self.workbench[i][0]
                for j in self.sell_workbench:
                    # 找到可卖出的工作台 该工作台的原材料格状态空缺
                    # 可以通过二进制位运算，判定是否拥有某个物品，比如判定是否拥有物品3 ：if (bit & (1<<3)) ...
                    if product == 1 and (self.workbench[j][0] == 4 or self.workbench[j][0] == 5 or self.workbench[j][0] == 9) and (self.workbench[j][4] & (1<<product)==0):
                        # 计算路径长度 = 购买路径长度 + 卖出路径长度
                        path_length = np.sqrt((robot.position[0] - self.workbench[i][1]) ** 2 + (robot.position[1] - self.workbench[i][2]) ** 2) + np.sqrt((self.workbench[i][1] - self.workbench[j][1]) ** 2 + (self.workbench[i][2] - self.workbench[j][2]) ** 2)
                        value = (6000 - 3000) / path_length
                        if value > max_value:
                            max_value = value
                            robot.target_buy_workbench = i
                            robot.target_sell_workbench = j
                        continue
                    if product == 2 and (self.workbench[j][0] == 4 or self.workbench[j][0] == 6 or self.workbench[j][0] == 9) and (self.workbench[j][4] & (1<<product)==0):
                        path_length = np.sqrt((robot.position[0] - self.workbench[i][1]) ** 2 + (robot.position[1] - self.workbench[i][2]) ** 2) + np.sqrt((self.workbench[i][1] - self.workbench[j][1]) ** 2 + (self.workbench[i][2] - self.workbench[j][2]) ** 2)
                        value = (7600 - 4400) / path_length
                        if value > max_value:
                            max_value = value
                            robot.target_buy_workbench = i
                            robot.target_sell_workbench = j  
                        continue
                    if product == 3 and (self.workbench[j][0] == 5 or self.workbench[j][0] == 6 or self.workbench[j][0] == 9) and (self.workbench[j][4] & (1<<product)==0):
                        path_length = np.sqrt((robot.position[0] - self.workbench[i][1]) ** 2 + (robot.position[1] - self.workbench[i][2]) ** 2) + np.sqrt((self.workbench[i][1] - self.workbench[j][1]) ** 2 + (self.workbench[i][2] - self.workbench[j][2]) ** 2)
                        value = (9200 - 5800) / path_length
                        if value > max_value:
                            max_value = value
                            robot.target_buy_workbench = i
                            robot.target_sell_workbench = j
                        continue
                    if product == 4 and (self.workbench[j][0] == 7 or self.workbench[j][0] == 9) and (self.workbench[j][4] & (1<<product)==0):
                        path_length = np.sqrt((robot.position[0] - self.workbench[i][1]) ** 2 + (robot.position[1] - self.workbench[i][2]) ** 2) + np.sqrt((self.workbench[i][1] - self.workbench[j][1]) ** 2 + (self.workbench[i][2] - self.workbench[j][2]) ** 2)
                        value = (22500 - 15400) / path_length
                        if value > max_value:
                            max_value = value
                            robot.target_buy_workbench = i
                            robot.target_sell_workbench = j
                        continue
                    if product == 5 and (self.workbench[j][0] == 7 or self.workbench[j][0] == 9) and (self.workbench[j][4] & (1<<product)==0):
                        path_length = np.sqrt((robot.position[0] - self.workbench[i][1]) ** 2 + (robot.position[1] - self.workbench[i][2]) ** 2) + np.sqrt((self.workbench[i][1] - self.workbench[j][1]) ** 2 + (self.workbench[i][2] - self.workbench[j][2]) ** 2)
                        value = (25000 - 17200) / path_length
                        if value > max_value:
                            max_value = value
                            robot.target_buy_workbench = i
                            robot.target_sell_workbench = j
                        continue
                    if product == 6 and (self.workbench[j][0] == 7 or self.workbench[j][0] == 9) and (self.workbench[j][4] & (1<<product)==0):
                        path_length = np.sqrt((robot.position[0] - self.workbench[i][1]) ** 2 + (robot.position[1] - self.workbench[i][2]) ** 2) + np.sqrt((self.workbench[i][1] - self.workbench[j][1]) ** 2 + (self.workbench[i][2] - self.workbench[j][2]) ** 2)
                        value = (27500 - 19200) / path_length
                        if value > max_value:
                            max_value = value
                            robot.target_buy_workbench = i
                            robot.target_sell_workbench = j
                        continue
                    if product == 7 and (self.workbench[j][0] == 8 or self.workbench[j][0] == 9) and (self.workbench[j][4] & (1<<product)==0):
                        path_length = np.sqrt((robot.position[0] - self.workbench[i][1]) ** 2 + (robot.position[1] - self.workbench[i][2]) ** 2) + np.sqrt((self.workbench[i][1] - self.workbench[j][1]) ** 2 + (self.workbench[i][2] - self.workbench[j][2]) ** 2)
                        value = (105000 - 76000) / path_length
                        if value > max_value:
                            max_value = value
                            robot.target_buy_workbench = i
                            robot.target_sell_workbench = j
                        continue
        if max_value > 0:
            find_target = True
        return find_target
    

    def find_nearest_sell_workbench(self, robot):
        find_target = False
        min_length = 10000000
        product = robot.carrying_item
        for j in self.sell_workbench:
            if product == 1 and (self.workbench[j][0] == 4 or self.workbench[j][0] == 5 or self.workbench[j][0] == 9) and (self.workbench[j][4] & (1<<product)==0):
                length = np.sqrt((robot.position[0] - self.workbench[j][1]) ** 2 + (robot.position[1] - self.workbench[j][2]) ** 2)
                if length < min_length:
                    min_length = length
                    robot.target_sell_workbench = j
                continue
            if product == 2 and (self.workbench[j][0] == 4 or self.workbench[j][0] == 6 or self.workbench[j][0] == 9) and (self.workbench[j][4] & (1<<product)==0):
                length = np.sqrt((robot.position[0] - self.workbench[j][1]) ** 2 + (robot.position[1] - self.workbench[j][2]) ** 2)
                if length < min_length:
                    min_length = length
                    robot.target_sell_workbench = j
                continue
            if product == 3 and (self.workbench[j][0] == 5 or self.workbench[j][0] == 6 or self.workbench[j][0] == 9) and (self.workbench[j][4] & (1<<product)==0):
                length = np.sqrt((robot.position[0] - self.workbench[j][1]) ** 2 + (robot.position[1] - self.workbench[j][2]) ** 2)
                if length < min_length:
                    min_length = length
                    robot.target_sell_workbench = j
                continue
            if product == 4 and (self.workbench[j][0] == 7 or self.workbench[j][0] == 9) and (self.workbench[j][4] & (1<<product)==0):
                length = np.sqrt((robot.position[0] - self.workbench[j][1]) ** 2 + (robot.position[1] - self.workbench[j][2]) ** 2)
                if length < min_length:
                    min_length = length
                    robot.target_sell_workbench = j
                continue
            if product == 5 and (self.workbench[j][0] == 7 or self.workbench[j][0] == 9) and (self.workbench[j][4] & (1<<product)==0):
                length = np.sqrt((robot.position[0] - self.workbench[j][1]) ** 2 + (robot.position[1] - self.workbench[j][2]) ** 2)
                if length < min_length:
                    min_length = length
                    robot.target_sell_workbench = j
                continue
            if product == 6 and (self.workbench[j][0] == 7 or self.workbench[j][0] == 9) and (self.workbench[j][4] & (1<<product)==0):
                length = np.sqrt((robot.position[0] - self.workbench[j][1]) ** 2 + (robot.position[1] - self.workbench[j][2]) ** 2)
                if length < min_length:
                    min_length = length
                    robot.target_sell_workbench = j
                continue
            if product == 7 and (self.workbench[j][0] == 8 or self.workbench[j][0] == 9) and (self.workbench[j][4] & (1<<product)==0):
                length = np.sqrt((robot.position[0] - self.workbench[j][1]) ** 2 + (robot.position[1] - self.workbench[j][2]) ** 2)
                if length < min_length:
                    min_length = length
                    robot.target_sell_workbench = j
                continue
        if min_length < 10000000:
            find_target = True
        return find_target
        
    # 总体策略
    def strategy(self):
        for robot in self.robot:
            # robot = self.robot[0]
            if robot.flag == -1: # 机器人处于结束状态
                # 机器人慢速前往地图中央
                robot.forward(-1)
                robot.flag = 0
            if robot.flag == 0: # 机器人处于空闲状态
                # 进行动态规划，找到最优的购买和出售工作台
                find_target = self.robot_dynamic_planning(robot)
                # 判断是否找到目标 若找到目标则进入购买路上
                if find_target:
                    # 购买表卖出表中删除路径上的工作台
                    self.buy_workbench.remove(robot.target_buy_workbench)
                    self.sell_workbench.remove(robot.target_sell_workbench)
                    robot.flag = 1
            if robot.flag == 1: # 机器人处于购买路上
                # 判断是否到达购买工作台
                if np.sqrt((robot.position[0] - self.workbench[robot.target_buy_workbench][1]) ** 2 + (robot.position[1] - self.workbench[robot.target_buy_workbench][2]) ** 2) < 0.4:
                    # 执行购买操作
                    if robot.buy_flag == 0:
                        robot.buy()
                        robot.buy_flag = 1
                    elif robot.buy_flag == 1:
                        robot.buy_flag = 0
                        # 判断是否购买成功
                        if robot.carrying_item != 0:
                            # 购买成功，进入卖出路上
                            robot.flag = 2
                        else:
                            # 购买失败，进入空闲状态去找一条新的路径
                            robot.flag = 0
                else:
                    # 未到达购买工作台，继续前进
                    robot.goto(self.workbench[robot.target_buy_workbench][1], self.workbench[robot.target_buy_workbench][2])
            if robot.flag == 2: # 机器人处于卖出路上
                # 判断是否到达出售工作台
                if np.sqrt((robot.position[0] - self.workbench[robot.target_sell_workbench][1]) ** 2 + (robot.position[1] - self.workbench[robot.target_sell_workbench][2]) ** 2) < 0.4:
                    # 执行出售操作
                    if robot.sell_flag == 0:
                        robot.sell()
                        robot.sell_flag = 1
                    elif robot.sell_flag == 1:
                        robot.sell_flag = 0
                        # 判断是否出售成功
                        if robot.carrying_item == 0:
                            self.buy_workbench.append(robot.target_buy_workbench)
                            self.sell_workbench.append(robot.target_sell_workbench)
                            robot.target_buy_workbench = -1
                            robot.target_sell_workbench = -1
                            robot.flag = -1
                        else:
                            # 出售失败，找到最近可出售工作台
                            find_target = self.find_nearest_sell_workbench(robot)
                            if find_target==False:
                                # 没有找到最近可出售工作台, 丢弃物品
                                robot.destroy()

                else:
                    # 未到达出售工作台，继续前进
                    robot.goto(self.workbench[robot.target_sell_workbench][1], self.workbench[robot.target_sell_workbench][2])

    # 收尾策略 只卖出不买入
    def end_strategy(self):
        for robot in self.robot:
            if robot.carrying_item == 0:
                robot.forward(0)
            # if robot.remain_frames > 9000 - self.frame_id:  # 完成任务需要的时间超过了剩余的时间 停止运动
            #     robot.forward(0)
            
# 机器人类
class Robot:
    def __init__(self, robot_id):
        self.robot_id = robot_id
        # 所处工作台 ID 整数 -1：表示当前没有处于任何工作台附近 [0,工作台总数-1] ： 表示某工作台的下标，从 0 开始， 按输入顺序定。当前机器人的所有购买、出售行为均针对该工作台进行。
        self.workbench_id = -1
        # 携带物品类型 整数 范围[0,7]。0 表示未携带物品。1-7 表示对应物品。
        self.carrying_item = 0
        # 时间价值系数 浮点 携带物品时为[0.8, 1]的浮点数，不携带物品时为 0。
        self.time_value = 0
        # 碰撞价值系数 浮点 携带物品时为[0.8, 1]的浮点数，不携带物品时为 0。
        self.collision_value = 0
        # 角速度 浮点 单位：弧度/秒。 正数：表示逆时针。 负数：表示顺时针。
        self.angle_speed = 0
        # 线速度 2 个浮点x,y 由二维向量描述线速度，单位：米/秒
        self.line_speed = [0, 0]
        # 朝向 浮点 弧度，范围[-π,π]。 方向示例： 0：表示右方向。 π /2： 表示上方向。 -π /2： 表示下方向。
        self.direction = 0
        # 坐标 2 个浮点x,y     
        self.position = [0, 0]

        # 补充变量
        self.error_direction_last = 0 # 上一时刻的朝向误差
        self.distance_last = 0 # 上一时刻的距离误差
        self.flag = 0 # 0：表示机器人处于空闲状态 1：表示机器人处于购买路上 2：表示机器人处于出售路上 -1：表示机器人已经完成任务
        self.target_buy_workbench = -1 # 目标购买工作台
        self.target_sell_workbench = -1 # 目标出售工作台
        self.buy_flag = 0 # 0：表示机器人未购买物品 1：表示机器人已购买物品
        self.sell_flag = 0 # 0：表示机器人未出售物品 1：表示机器人已出售物品
        self.remain_frames = 0 # 机器人预计完成当前任务（买入->卖出）的剩余帧数  需要测试出一个与距离相关的公式


    # 更新机器人的状态
    def update(self, workbench_id, carrying_item, time_value, collision_value, angle_speed, line_speed, direction, position):
        self.workbench_id = workbench_id
        self.carrying_item = carrying_item
        self.time_value = time_value
        self.collision_value = collision_value
        self.angle_speed = angle_speed
        self.line_speed = line_speed
        self.direction = direction
        self.position = position
    

    # 前往坐标（x,y）
    def goto(self, x, y):
        # 利用PD控制器控制机器人的角速度
        Kp = 3; Kd = 0.5
        # 利用PD控制器控制机器人的线速度
        Kp_line = 6.5; Kd_line = 2
        # 计算当前机器人的朝向 范围[-π,π]
        expect_direction = np.arctan2(y - self.position[1], x - self.position[0])
        error_direction = expect_direction - self.direction
        # 计算当前机器人到目标点的距离
        distance = np.sqrt((x - self.position[0]) ** 2 + (y - self.position[1]) ** 2)
        # 限制角度范围[-π,π]
        if error_direction > np.pi:
            error_direction = error_direction - 2 * np.pi
        elif error_direction < -np.pi:
            error_direction = error_direction + 2 * np.pi
        # 计算角速度
        angle_speed = error_direction * Kp + (error_direction - self.error_direction_last) * Kd
        self.error_direction_last = error_direction
        # 计算线速度
        line_speed = distance * Kp_line + (distance - self.distance_last) * Kd_line
        self.distance_last = distance
        # 限制线速度 朝向误差过大且离工作台太近时，线速度为0
        if abs(error_direction) > np.pi / 4:
            line_speed = 1
        # 避免撞墙 坐标离墙太近且朝墙
        if (self.position[0] < 0.5 and self.line_speed[0] < 0) or (self.position[0] > 49.5 and self.line_speed[0] > 0) or (self.position[1] < 0.5 and self.line_speed[1] < 0) or (self.position[1] > 49.5 and self.line_speed[1] > 0):
            line_speed = 0

        # 控制机器人
        self.rotate(angle_speed)
        self.forward(line_speed)

        
    def forward(self, line_speed):
        sys.stdout.write('forward %d %d\n' % (self.robot_id, line_speed))
    def rotate(self, angle_speed):
        sys.stdout.write('rotate %d %f\n' % (self.robot_id, angle_speed))
    def buy(self):
        sys.stdout.write('buy %d\n' % (self.robot_id))
    def sell(self):
        sys.stdout.write('sell %d\n' % (self.robot_id))
    def destroy(self):
        sys.stdout.write('destroy %d\n' % (self.robot_id))


if __name__ == '__main__':
    GameStatus = GameStatus()
    # time.sleep(5)
    # 读取地图
    GameStatus.read_map()
    read_util_ok()
    finish()
    while True:
        GameStatus.read_frame()
        read_util_ok()
        sys.stdout.write('%d\n' % (GameStatus.frame_id))

        GameStatus.strategy()
        if GameStatus.frame_id > 8700:  # 最后300帧 只卖出物品 不再购买
            GameStatus.end_strategy()
        finish()
