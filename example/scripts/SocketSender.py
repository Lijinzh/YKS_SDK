#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Time    : 2025/3/19 17:17
Author  : mrtang
Email   : 810899799@qq.com
"""

from SocketMotorData import *
import time
import math

if __name__ == '__main__':
    sender = MotorCmdSender()
    motorCmds = [MotorData() for _ in range(13)]  # 使用列表推导式简化初始化

    count = 1
    start_time = time.time()  # 记录开始时间用于计算经过的时间
    incrementing = True
    while True:
        current_time = time.time() - start_time  # 当前循环相对于开始的时间
        amplitude = 2  # 正弦波幅度
        frequency = 0.3  # 频率
        pos_des = amplitude * math.sin(2 * math.pi * frequency * current_time)
        # 设置电机的状态
        for i in range(13):
            motorCmds[i].mode = 2
            motorCmds[i].index = i

            if i == 0:
                motorCmds[i].pos_des_ = pos_des
                # 定义阶跃信号的变化时间点
                # if (current_time // 2) % 2 == 0:
                #     pos_des = 0
                # else:
                #     pos_des = 1
            elif i == 2:
                # if (current_time // 2) % 2 == 0:
                #     motorCmds[i].pos_des_ = 0
                # else:
                #     motorCmds[i].pos_des_ = 1
                motorCmds[i].pos_des_ = pos_des
            else:
                motorCmds[i].pos_des_ = 0  # 其他电机保持静态或其他设定值

            # motorCmds[i].pos_des_ = 0
            motorCmds[i].vel_des_ = 0
            motorCmds[i].kp_ = 500
            motorCmds[i].kd_ = 50
            motorCmds[i].ff_ = 0

        if incrementing:
            count += 1
            if count >= 10000:
                incrementing = False
        else:
            count -= 1
            if count <= 0:
                incrementing = True
        sender.send(motorCmds)  # 调用send将命令下发
        print('Motor 0 target position: %f' % motorCmds[0].pos_des_, motorCmds[0].index)

        time.sleep(0.001)  # 控制循环速度
