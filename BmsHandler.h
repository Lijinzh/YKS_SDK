#ifndef BMSHANDLER_H
#define BMSHANDLER_H

#include <array>
#include <atomic>
#include <mutex>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <iostream>
#include <string>
#include <vector>
#include <thread>
#include <chrono>
#include <iomanip>
#include <ctime>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <map>
#include <sstream>

struct BmsState {
    int voltage[13]; // 假设最多有13个电压值
    float temperature;
    int cycle_count;
    int soc;
    bool low_voltage;
    bool over_current;
    bool over_temp;
};

class BmsHandler {
public:
    explicit BmsHandler(const char *device);

    ~BmsHandler();

    BmsState getState();

    static void print_state(const BmsState &state);

private:
    void run();

    uint8_t crc8_cal(const std::vector<uint8_t> &data);

    std::string parse_voltage(const std::vector<uint8_t> &data);

    std::string parse_temp(const std::vector<uint8_t> &data);

    std::pair<std::string, bool> parse_soc(const std::vector<uint8_t> &data);

    std::shared_ptr<std::thread> bms_thread_;
    mutable std::mutex mutex_;
    BmsState state_;
    const char *device_;
    int fd_;
    volatile bool stop_;
};

#endif // BMSHANDLER_H
