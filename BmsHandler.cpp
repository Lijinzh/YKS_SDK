#include "BmsHandler.h"

const std::vector<uint8_t> CRC8_TABLE = {
    0x00,0x07,0x0E,0x09,0x1C,0x1B,0x12,0x15,0x38,0x3F,0x36,0x31,0x24,0x23,0x2A,0x2D,
    0x70,0x77,0x7E,0x79,0x6C,0x6B,0x62,0x65,0x48,0x4F,0x46,0x41,0x54,0x53,0x5A,0x5D,
    0xE0,0xE7,0xEE,0xE9,0xFC,0xFB,0xF2,0xF5,0xD8,0xDF,0xD6,0xD1,0xC4,0xC3,0xCA,0xCD,
    0x90,0x97,0x9E,0x99,0x8C,0x8B,0x82,0x85,0xA8,0xAF,0xA6,0xA1,0xB4,0xB3,0xBA,0xBD,
    0xC7,0xC0,0xC9,0xCE,0xDB,0xDC,0xD5,0xD2,0xFF,0xF8,0xF1,0xF6,0xE3,0xE4,0xED,0xEA,
    0xB7,0xB0,0xB9,0xBE,0xAB,0xAC,0xA5,0xA2,0x8F,0x88,0x81,0x86,0x93,0x94,0x9D,0x9A,
    0x27,0x20,0x29,0x2E,0x3B,0x3C,0x35,0x32,0x1F,0x18,0x11,0x16,0x03,0x04,0x0D,0x0A,
    0x57,0x50,0x59,0x5E,0x4B,0x4C,0x45,0x42,0x6F,0x68,0x61,0x66,0x73,0x74,0x7D,0x7A,
    0x89,0x8E,0x87,0x80,0x95,0x92,0x9B,0x9C,0xB1,0xB6,0xBF,0xB8,0xAD,0xAA,0xA3,0xA4,
    0xF9,0xFE,0xF7,0xF0,0xE5,0xE2,0xEB,0xEC,0xC1,0xC6,0xCF,0xC8,0xDD,0xDA,0xD3,0xD4,
    0x69,0x6E,0x67,0x60,0x75,0x72,0x7B,0x7C,0x51,0x56,0x5F,0x58,0x4D,0x4A,0x43,0x44,
    0x19,0x1E,0x17,0x10,0x05,0x02,0x0B,0x0C,0x21,0x26,0x2F,0x28,0x3D,0x3A,0x33,0x34,
    0x4E,0x49,0x40,0x47,0x52,0x55,0x5C,0x5B,0x76,0x71,0x78,0x7F,0x6A,0x6D,0x64,0x63,
    0x3E,0x39,0x30,0x37,0x22,0x25,0x2C,0x2B,0x06,0x01,0x08,0x0F,0x1A,0x1D,0x14,0x13,
    0xAE,0xA9,0xA0,0xA7,0xB2,0xB5,0xBC,0xBB,0x96,0x91,0x98,0x9F,0x8A,0x8D,0x84,0x83,
    0xDE,0xD9,0xD0,0xD7,0xC2,0xC5,0xCC,0xCB,0xE6,0xE1,0xE8,0xEF,0xFA,0xFD,0xF4,0xF3
};

BmsHandler::BmsHandler(const char *device)
    : device_(device), stop_(false) {
    bms_thread_ = std::make_shared<std::thread>(&BmsHandler::run, this);
}

BmsHandler::~BmsHandler() {
    stop_ = true;
    if (bms_thread_->joinable()) {
        bms_thread_->join();
    }
    close(fd_);
}

BmsState BmsHandler::getState(){
    // std::lock_guard<std::mutex> lock(mutex_);
    return state_;
}

void BmsHandler::print_state(const BmsState &state) {
    for (int i = 0; i < 13; ++i) {
        std::cout << "Voltage " << i << ": " << state.voltage[i] / 100.0f << "V ";
    }
    std::cout << "\nTemperature: " << state.temperature << "°C\n";
    std::cout << "Cycle Count: " << state.cycle_count << "\n";
    std::cout << "SOC: " << state.soc << "%\n";
    std::cout << "Low Voltage: " << (state.low_voltage ? "Yes" : "No") << "\n";
    std::cout << "Over Current: " << (state.over_current ? "Yes" : "No") << "\n";
    std::cout << "Over Temperature: " << (state.over_temp ? "Yes" : "No") << "\n";
}

void BmsHandler::run() {
    fd_ = open(device_, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd_ == -1) {
        perror("open_port: Unable to open /dev/ttyUSB0 - ");
        return;
    }

    struct termios options;
    tcgetattr(fd_, &options);
    cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);

    options.c_cflag |= (CLOCAL | CREAD); // Enable receiver, ignore modem control lines
    options.c_cflag &= ~PARENB;          // No parity bit
    options.c_cflag &= ~CSTOPB;          // One stop bit
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;              // 8 bits per byte

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Raw input mode
    options.c_oflag &= ~OPOST;                         // Raw output mode

    tcsetattr(fd_, TCSANOW, &options);

    const std::map<std::string, std::vector<uint8_t>> COMMANDS = {
        {"电压查询", {0x0A, 0x66, 0x1C, 0x1E, 0x01}},
        {"温度查询", {0x0A, 0x66, 0x06, 0x08, 0x02}},
        {"SOC查询",  {0x0A, 0x66, 0x14, 0x16, 0x03}},
        {"充放电次数", {0x0A, 0x66, 0x0A, 0x0C, 0x05}}
    };

    while (!stop_) {
        try {
            bool alarm_triggered = false;

            for (const auto& [cmd_name, cmd_bytes] : COMMANDS) {
                write(fd_, cmd_bytes.data(), cmd_bytes.size());

                std::this_thread::sleep_for(std::chrono::milliseconds(100));

                char buffer[256];
                ssize_t bytes_read = read(fd_, buffer, sizeof(buffer));
                std::vector<uint8_t> response(buffer, buffer + bytes_read);

                if (response.size() < 3) {
                    std::cout << cmd_name << " 无响应" << std::endl;
                    continue;
                }

                uint8_t crc_received = response.back();
                std::vector<uint8_t> data_part(response.begin(), response.end() - 1);
                uint8_t crc_calculated = crc8_cal(data_part);

                if (crc_received != crc_calculated) {
                    std::cout << cmd_name << " CRC校验失败" << std::endl;
                    continue;
                }

                std::lock_guard<std::mutex> lock(mutex_);

                if (cmd_name == "电压查询") {
                  parse_voltage(std::vector<uint8_t>(data_part.begin() + 2, data_part.end()));
                } else if (cmd_name == "温度查询") {
                    parse_temp(std::vector<uint8_t>(data_part.begin() + 2, data_part.end()));
                    if (state_.temperature > 40) {
                        alarm_triggered = true;
                    }
                } else if (cmd_name == "SOC查询") {
                    auto [soc_str, low_soc] = parse_soc(std::vector<uint8_t>(data_part.begin() + 2, data_part.end()));
//                    std::cout << soc_str << std::endl;
                    if (low_soc) {
                        alarm_triggered = true;
                    }
                } else if (cmd_name == "充放电次数") {
                    // Assuming the cycle count is in the first four bytes of the data part
                    int cycle_count = (data_part[2] << 24 | data_part[3] << 16 | data_part[4] << 8 | data_part[5]);
//                    std::cout << "充放电次数: " << cycle_count << std::endl;
                    state_.cycle_count = cycle_count;
                }
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
            if (alarm_triggered) {
                std::cout << "\n警告: 温度过高或SOC过低!\n";
            }
            std::time_t now = std::time(nullptr);
//            std::cout << "\n" << std::put_time(std::localtime(&now), "%Y-%m-%d %H:%M:%S") << " 完成一轮查询\n";
//            std::this_thread::sleep_for(std::chrono::seconds(5));

        } catch (const std::exception& e) {
            std::cerr << "\n程序终止: " << e.what() << std::endl;
            close(fd_);
            return;
        } catch (...) {
            std::cerr << "\n程序终止: 未知错误" << std::endl;
            close(fd_);
            return;
        }
    }
}

uint8_t BmsHandler::crc8_cal(const std::vector<uint8_t>& data) {
    uint8_t crc = 0;
    for (auto byte : data) {
        crc = CRC8_TABLE[crc ^ byte];
    }
    return crc;
}

std::string BmsHandler::parse_voltage(const std::vector<uint8_t>& data) {
    std::vector<float> voltages;
    size_t index = 0;
    for (size_t i = 0; i < data.size(); i += 2) {
        float voltage = (data[i] << 8 | data[i + 1]) / 100.0f;
        voltages.push_back(voltage);
        state_.voltage[index++] = static_cast<int>(voltage * 100); // Convert to int and scale by 100
    }
    std::ostringstream oss;
    oss << "电池电压: ";
    for (float v : voltages) {
        oss << v << "V ";
    }
    return oss.str();
}

std::string BmsHandler::parse_temp(const std::vector<uint8_t>& data) {
    std::vector<float> temps;
    for (size_t i = 0; i < data.size(); i += 2) {
        float temp = (data[i] << 8 | data[i + 1]) / 10.0f;
        temps.push_back(temp);
    }
    std::ostringstream oss;
    oss << "温度: ";
    for (float t : temps) {
        oss << t << "°C ";
    }
    if (!temps.empty()) {
        state_.temperature = temps[0]; // Assuming only one temperature value
    }

    return oss.str();
}

std::pair<std::string, bool> BmsHandler::parse_soc(const std::vector<uint8_t>& data) {
    float soc = (data[0] << 8 | data[1]) / 1.0f;
    int32_t current = ((int32_t)(data[2] << 24 | data[3] << 16 | data[4] << 8 | data[5])) / 1000.0f;
    float voltage = (data[6] << 24 | data[7] << 16 | data[8] << 8 | data[9]) / 1000.0f;
    std::ostringstream oss;
    oss << "SOC: " << soc << "%  电流: " << current << "A  总电压: " << voltage << "V";
    state_.soc = static_cast<int>(soc);
    return {oss.str(), soc < 10};
}



