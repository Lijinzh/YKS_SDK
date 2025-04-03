#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Time    : 2025/3/19 16:17
Author  : mr tang
Email   : 810899799@qq.com
"""

import socket
import threading
import numpy as np

YKS_PORT = 4035
USR_PORT = 4015


# 电机指令的结构，其中reserve为保留字，可根据需要启用
class MotorData:
    mode = 0  # 0-disable 1-enable
    index = 0  # 电机序号，这个可以不用设置，靠列表的顺序也可
    tau_, pos_, vel_ = 0, 0, 0  # 当前的位置rad、速度rad/s、力矩N.m
    pos_des_, vel_des_, kp_, kd_, ff_ = 0, 0, 100, 50, 0  # 期望的位置、速度、比例、积分、力矩N.m
    # mfn -> 调试新加
    error_, temperature_, mos_temperature_= 0, 0, 0
    @property
    def bytes_len(self):
        return 10 * 8  # 10个double类型数据，每个double占8字节


class MotorCmdSender:
    def __init__(self, remote_port=YKS_PORT):
        self.remoteAddr = ("127.0.0.1", remote_port)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, commands):
        """
        :param commands:  MotorCmd类型的列表
        :return:
        """
        cmd = []
        for cm in commands:
            # 注意下面这个顺序是很关键的，不要改
            cmd.append(cm.mode)
            cmd.append(cm.index)
            cmd.append(cm.tau_)
            cmd.append(cm.pos_)
            cmd.append(cm.vel_)
            cmd.append(cm.pos_des_)
            cmd.append(cm.vel_des_)
            cmd.append(cm.kp_)
            cmd.append(cm.kd_)
            cmd.append(cm.ff_)
            cmd.append(cm.error_)
            cmd.append(cm.temperature_)
            cmd.append(cm.mos_temperature_)

        buf = np.array(cmd, dtype=np.float64).tobytes()
        self._sock.sendto(buf, self.remoteAddr)

    def __del__(self):
        try:
            self._sock.close()
        except:
            pass


class MotorStateReader(threading.Thread):
    def __init__(self, motor_num=13, host_port=USR_PORT):
        super(MotorStateReader, self).__init__()
        self.daemon = True

        self._lock = threading.Lock()

        self.data = np.zeros(1024, dtype=np.float64)

        self.motors = []
        self.motorNum = motor_num
        for i in range(motor_num):
            self.motors.append(MotorData())

        self.total_bytes_len = motor_num * self.motors[0].bytes_len

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind(("127.0.0.1", host_port))
        self.running = True
        self.start()

    @property
    def motors_data(self):
        self._lock.acquire()
        data = self.data.copy()
        self._lock.release()

        for i in range(self.motorNum):
            start_index = i * 10
            motor_data = data[start_index:start_index + 10]

            self.motors[i].mode = motor_data[0]
            self.motors[i].index = motor_data[1]
            self.motors[i].tau_ = motor_data[2]
            self.motors[i].pos_ = motor_data[3]
            self.motors[i].vel_ = motor_data[4]
            self.motors[i].pos_des_ = motor_data[5]
            self.motors[i].vel_des_ = motor_data[6]
            self.motors[i].kp_ = motor_data[7]
            self.motors[i].kd_ = motor_data[8]
            self.motors[i].ff_ = motor_data[9]
            self.motors[i].error_ = motor_data[10]
            self.motors[i].temperature_ = motor_data[11]
            self.motors[i].mos_temperature_ = motor_data[12]
            # mfn -> 新加

        return self.motors

    def __del__(self):
        try:
            self._sock.close()
        except:
            pass

    def run(self):
        while self.running:
            buf, _ = self._sock.recvfrom(65536)
            if len(buf) != self.total_bytes_len:  # 数据长度校验
                print("package length error!")
                continue

            self._lock.acquire()
            self.data = np.frombuffer(buf, dtype=np.float64)
            self._lock.release()

        self._sock.close()
