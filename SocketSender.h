#ifndef SOCKET_SENDER_H
#define SOCKET_SENDER_H

#include <iostream>
#include <cstring>
#include <arpa/inet.h> // For socket functions
#include <unistd.h>    // For close()
#include <thread>
#include <memory>
#include <atomic>
#include <mutex>
#include "app/command.h"
#include "Z1Legs.h"
#define MOTOR_NUM Z1_NUM_MOTOR
#define BUFFER_SIZE 65535
#define YKS_PORT 4035
#define USR_PORT 4015

// 匹配python中的数据结构，
typedef struct {
    double mode, index; //这里必须是double才行，否则和python那边发过来的对不上，就无法正常通信，电机控制模式，可以起到软急停的作用，模式为0代表停止运动，目前还没实现
    double tau_, pos_, vel_; //当前的位置rad、速度rad/s、力矩N.m
    double pos_des_, vel_des_, kp_, kd_, ff_; //期望的位置、速度、比例、积分、力矩N.m
    double error_, temperature_, mos_temperature_; // 读到的错误类型，电机温度和MOS温度
} InteractiveMotorData;

constexpr int packageLen = MOTOR_NUM * sizeof(InteractiveMotorData);

class SocketSender {
public:
    SocketSender(const char *ipAddress, int port);

    ~SocketSender();

    void startSending();

    void stopSending();

    void sendSocketMotorData(const YKSMotorData *data);

private:
    const char *ipAddress;
    int port;
    std::shared_ptr<std::thread> socket_sender_thread_;
    mutable std::mutex mutex_;
    std::atomic<bool> stop_;

    void run();

    YKSMotorData motor_data_[MOTOR_NUM]{};
    InteractiveMotorData motor_cmds[MOTOR_NUM]{};
};

#endif // SOCKET_SENDER_H
