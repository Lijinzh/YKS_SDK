#include "SocketSender.h"
#include <chrono>
#include <thread>

SocketSender::SocketSender(const char *ipAddress, const int port)
    : ipAddress(ipAddress), port(port), stop_(false) {
    startSending();
}

SocketSender::~SocketSender() {
    stopSending();
}

void SocketSender::startSending() {
    socket_sender_thread_ = std::make_shared<std::thread>(&SocketSender::run, this);
}

void SocketSender::stopSending() {
    std::lock_guard lock(mutex_);
    stop_ = true;
    if (socket_sender_thread_ && socket_sender_thread_->joinable()) {
        socket_sender_thread_->join();
    }
}

void SocketSender::sendSocketMotorData(const YKSMotorData *data) {
    std::lock_guard<std::mutex> lock(mutex_);
    std::memcpy(motor_data_, data, Z1_NUM_MOTOR * sizeof(YKSMotorData));
}

void SocketSender::run() {
    // Create a UDP socket
    const int clientSocket = socket(AF_INET, SOCK_DGRAM, 0);
    if (clientSocket < 0) {
        std::cerr << "Failed to create socket" << std::endl;
        return;
    }

    std::cout << "socket start" << std::endl;

    // 接收方地址
    sockaddr_in remoteAddress{};
    remoteAddress.sin_family = AF_INET; // IPv4
    remoteAddress.sin_port = htons(port);
    inet_pton(AF_INET, ipAddress, &remoteAddress.sin_addr);

    const auto p_buffer = reinterpret_cast<char *>(motor_cmds);

    while (true) {
        {
            std::lock_guard lock(mutex_);
            if (stop_) {
                break;
            }
            for (int i = 0; i < MOTOR_NUM; i++) {
                motor_cmds[i].pos_ = motor_data_[i].pos_;
                motor_cmds[i].vel_ = motor_data_[i].vel_;
                motor_cmds[i].tau_ = motor_data_[i].tau_;
                // mfn -> 调试新加
                motor_cmds[i].mode = motor_data_[i].mode;
                motor_cmds[i].kp_ = motor_data_[i].kp_;
                motor_cmds[i].error_ = motor_data_[i].error_;
                motor_cmds[i].temperature_ = motor_data_[i].temperature_;
                motor_cmds[i].mos_temperature_ = motor_data_[i].mos_temperature_;
            }
        }
        // 发送数据
        int bytesSent = sendto(clientSocket, p_buffer, packageLen, 0,
                               (struct sockaddr *) &remoteAddress, sizeof(remoteAddress));
        // 控制发送频率，这里是为了调试,实际如何控制根据实际情况来
        std::this_thread::sleep_for(std::chrono::microseconds(1000)); //一千微妙 1ms
    }
    // Close the socket
    close(clientSocket);
    std::cout << "UDP client closed." << std::endl;
}
