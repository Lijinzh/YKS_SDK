#include "SocketReceiver.h"

SocketReceiver::SocketReceiver(const int port) : port(port), stop_(false) {
    startListening();
}

SocketReceiver::~SocketReceiver() {
    stopListening();
}

void SocketReceiver::startListening() {
    socket_recv_thread_ = std::make_shared<std::thread>(&SocketReceiver::run, this);
}

void SocketReceiver::stopListening() {
    std::lock_guard lock(mutex_);
    stop_ = true;
    if (socket_recv_thread_ && socket_recv_thread_->joinable()) {
        socket_recv_thread_->join();
    }
}

void SocketReceiver::run() {
    // Create a UDP socket
    const int serverSocket = socket(AF_INET, SOCK_DGRAM, 0);
    if (serverSocket < 0) {
        std::cerr << "\033[31mFailed to create socket\033[0m" << std::endl;
        return;
    }
    // Bind the socket to the receiver's port
    sockaddr_in hostAddress{};
    hostAddress.sin_family = AF_INET; // IPv4
    hostAddress.sin_port = htons(port); // Receiver's port
    hostAddress.sin_addr.s_addr = INADDR_ANY; // Bind to any available interface
    if (bind(serverSocket, reinterpret_cast<struct sockaddr *>(&hostAddress), sizeof(hostAddress)) < 0) {
        std::cerr << "Failed to bind socket to port " << port << std::endl;
        close(serverSocket);
        return;
    }
    std::cout << "\033[32mUDP receiver is listening on port \033[0m" << port << "..." << std::endl;
    sockaddr_in clientAddress{};
    socklen_t clientAddressLength = sizeof(clientAddress);

    const auto buffer = reinterpret_cast<char *>(motor_cmds);
    while (true) {
        {
            std::lock_guard lock(mutex_);
            if (stop_) {
                break;
            }
        }
        // Receive a message
        const int bytesReceived = recvfrom(serverSocket, buffer, BUFFER_SIZE, 0,
                                           reinterpret_cast<struct sockaddr *>(&clientAddress), &clientAddressLength);
        if (bytesReceived < 0) {
            std::cerr << "\033[31mFailed to receive message\033[0m" << std::endl;
            continue;
        }
        // 做一个简单的校验
        if (bytesReceived != packageLen)
            continue;
        // 到这里，数据已经接受到motorCmds中
        // do something here，在这里可以进行对应的电机控制
        // std::cout << "Received package length " << bytesReceived << std::endl;
        {
            std::lock_guard lock(mutex_);
            for (int i = 0; i < MOTOR_NUM; i++) {
                motor_data_[i].mode = static_cast<int>(motor_cmds[i].mode);
                motor_data_[i].pos_des_ = motor_cmds[i].pos_des_;
                motor_data_[i].vel_des_ = motor_cmds[i].vel_des_;
                motor_data_[i].kp_ = motor_cmds[i].kp_;
                motor_data_[i].kd_ = motor_cmds[i].kd_;
                motor_data_[i].ff_ = motor_cmds[i].ff_;
            }

        }
    }
    // Close the socket (this will never be reached in this example)
    close(serverSocket);
}

void SocketReceiver::getSocketMotorCMD(YKSMotorData *data) const {
    std::lock_guard lock(mutex_);
    std::memcpy(data, motor_data_, MOTOR_NUM * sizeof(YKSMotorData));
}
