//
// Created by gold on 25-3-12.
//

#include "SBusReceiver.h"

SBusReceiver::SBusReceiver(const char *device) : device_(device), stop_(false) {
    serial_thread_ = std::make_shared<std::thread>(&SBusReceiver::run, this);
}

SBusReceiver::~SBusReceiver() {
    std::lock_guard<std::mutex> lock(mutex_);
    stop_ = true;
    if (serial_thread_->joinable()) {
        serial_thread_->join();
    }
}

SBusData SBusReceiver::getData() const {
    std::lock_guard lock(mutex_);
    if (data_.failsafe) {
        const auto now = std::chrono::system_clock::now();
        auto now_c = std::chrono::system_clock::to_time_t(now);
        std::cerr << "No data available, lost frame！ 遥控器已断开，失控保护开启！" << " 时间: "
                << std::put_time(std::localtime(&now_c), "%Y-%m-%d %H:%M:%S")
                << std::endl;;
        return SBusData{};
    }
    return data_;
}

void SBusReceiver::print_data(const SBusData &data) {
    std::cout << "Lost Frame: " << data.lost_frame << ", Failsafe: " << data.failsafe;
    std::cout << ", Channels: ";
    for (const short i: data.ch) {
        std::cout << i << " ";
    }
    std::cout << "\n";
}

void SBusReceiver::run() {
    const int fd = open(device_, O_RDWR | O_NOCTTY);

    if (fd == -1) {
        std::cerr << "\033[35mSBUS接收机未连接！ Error opening serial port: \033[0m" << strerror(errno) << std::endl;
        return;
    }
    termios tty = {};
    if (tcgetattr(fd, &tty) != 0) {
        std::cerr << "Error from tcgetattr: " << strerror(errno) << std::endl;
        close(fd);
        return;
    }

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag |= (CLOCAL | CREAD); // Enable the receiver and set local mode
    tty.c_cflag &= ~PARENB; // No parity bit
    tty.c_cflag &= ~CSTOPB; // Only need 1 stop bit
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8; // 8 bits per byte
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Raw input
    tty.c_oflag &= ~OPOST; // Raw output

    tty.c_cc[VTIME] = 10; // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cerr << "Error from tcsetattr: " << strerror(errno) << std::endl;
        close(fd);
        return;
    }
    char read_buf[256];
    std::string line;

    while (true) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (stop_) break;
        }
        if (const int num_bytes = read(fd, &read_buf, sizeof(read_buf) - 1); num_bytes < 0) {
            std::cerr << "Error reading from serial port: " << strerror(errno) << std::endl;
            break;
        } else if (num_bytes > 0) {
            read_buf[num_bytes] = '\0'; // Ensure string termination
            line += read_buf;

            size_t pos;
            while ((pos = line.find('\n')) != std::string::npos) {
                std::string current_line = line.substr(0, pos);
                line.erase(0, pos + 1);
                parseLine(current_line);
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(15)); //读取周期为15ms，S.BUS信号的标准帧速率大约是14毫秒（ms）每帧
    }
    close(fd);
}

void SBusReceiver::parseLine(const std::string &line) {
    std::vector<int> values;
    std::stringstream ss(line);
    std::string item;

    while (std::getline(ss, item, ' ')) {
        try {
            int value = std::stoi(item);
            values.push_back(value);
        } catch (const std::invalid_argument &e) {
            std::cerr << "Invalid integer conversion: " << item << std::endl;
        }
    }

    if (values.size() >= 18) {
        // 2 flags + 16 channels
        SBusData new_data{};
        new_data.lost_frame = (values[0] == 1);
        new_data.failsafe = (values[1] == 1);
        // printf("%d\n",values[2]);
        for (int i = 0; i < SBusData::NUM_CH; ++i) {
            new_data.ch[i] = values[i + 2];
        }
        // Update the data
        {
            std::lock_guard<std::mutex> lock(mutex_);
            data_ = new_data;
        }
        // Print parsed data
        // print_data(new_data);
    } else {
        std::cerr << "Invalid data format: " << line << std::endl;
    }
}
