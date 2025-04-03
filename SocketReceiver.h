#ifndef SOCKET_RECEIVER_H
#define SOCKET_RECEIVER_H

#include "SocketSender.h"

class SocketReceiver {
public:
    explicit SocketReceiver(int port);

    ~SocketReceiver();

    void startListening();

    void stopListening();

    void getSocketMotorCMD(YKSMotorData *data) const;

private:
    int port;
    std::shared_ptr<std::thread> socket_recv_thread_;
    mutable std::mutex mutex_;
    std::atomic<bool> stop_;
    YKSMotorData motor_data_[MOTOR_NUM]{}; //私有的电机结构体数组
    InteractiveMotorData motor_cmds[MOTOR_NUM]{}; // 开辟内存
    void run();
};

#endif // SOCKET_RECEIVER_H
