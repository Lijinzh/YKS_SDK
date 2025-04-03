#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Time    : 2025/3/19 17:27
Author  : mr tang
Email   : 810899799@qq.com
"""

from SocketMotorData import *
import time

if __name__ == '__main__':
    motor_state = MotorStateReader(motor_num=13)

    while True:
        data = motor_state.motors_data  # 根据需要读取数据,data为列表，包含motorNum个
        for i in range(len(data)):
            if i == 0:
                print(data[i].pos_, data[i].vel_, data[i].tau_)

        time.sleep(0.001)
