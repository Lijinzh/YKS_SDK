//
// Created by gold on 25-3-12.
//

#ifndef SBUSRECEIVER_H
#define SBUSRECEIVER_H

#include <string>
#include <memory>
#include <thread>
#include <vector>
#include <mutex>
#include <atomic>
#include <chrono>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cerrno>
#include <iostream>
#include <sstream>
#include <cstring>
#include <iomanip>

struct SBusData {
    bool lost_frame = false;
    bool failsafe = false;
    static constexpr int8_t NUM_CH = 16;
    int ch[NUM_CH];
};

class SBusReceiver {
public:
    explicit SBusReceiver(const char *device);

    ~SBusReceiver();

    SBusData getData() const;

    static void print_data(const SBusData &data);

private:
    void run();

    void parseLine(const std::string &line);

    const char *device_;
    SBusData data_{};
    std::shared_ptr<std::thread> serial_thread_;
    mutable std::mutex mutex_;
    std::atomic<bool> stop_;
};


#endif //SBUSRECEIVER_H
