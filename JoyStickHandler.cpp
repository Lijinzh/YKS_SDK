//
// Created by gold on 25-2-26.
//

#include "JoyStickHandler.h"


JoyStickHandler::JoyStickHandler(const char *device) : device_(device), stop_(false) {
    ff_pack.cmd = 0x03;

    ff_pack.enable.strong = 1;
    ff_pack.enable.weak = 1;
    ff_pack.enable.left = 1;
    ff_pack.enable.right = 1;

    ff_pack.strength.strong = 40;
    ff_pack.strength.weak = 30;
    ff_pack.strength.left = 20;
    ff_pack.strength.right = 20;

    ff_pack.pulse.sustain_10ms = 5;
    ff_pack.pulse.release_10ms = 5;
    ff_pack.pulse.loop_count = 3;
    joystick_thread_ = std::make_shared<std::thread>(&JoyStickHandler::run, this);
}


JoystickState JoyStickHandler::getState() const {
    std::lock_guard lock(mutex_);
    return state_;
}

void JoyStickHandler::print_state(const JoystickState &state) {
    std::cout << "Axes: ";
    for (short axe: state.axes) {
        std::cout << axe << " ";
    }
    std::cout << "Buttons: ";
    for (unsigned char button: state.buttons) {
        std::cout << static_cast<int>(button) << " ";
    }
    std::cout << "\n";
}

void JoyStickHandler::run() {
    const int fd = open(device_, O_RDONLY | O_NONBLOCK);
    if (fd == -1) {
        perror("\033[35mFailed to open joystick device 手柄未连接！\033[0m");
        return;
    }

    js_event event{};
    auto last_time = std::chrono::steady_clock::now();
    double frequency_sum = 0.0;
    int count = 0;

    while (true) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (stop_) break;
        }

        ssize_t bytes = read(fd, &event, sizeof(event));
        if (bytes == sizeof(event)) {
            JoystickEvent je = {event.time, event.value, event.type, event.number};
            updateState(je);
        } else if (bytes == -1 && errno != EAGAIN) {
            perror("Error reading from joystick device");
            break;
        } {
            std::lock_guard<std::mutex> lock(mutex_);
            state_.left_stick_x = state_.axes[0];
            state_.left_stick_y = state_.axes[1];
            state_.right_stick_x = state_.axes[2];
            state_.right_stick_y = state_.axes[3];
            state_.left_trigger = state_.axes[4];
            state_.right_trigger = state_.axes[5];
            state_.DPAD_x = state_.axes[6];
            state_.DPAD_y = state_.axes[7];
            state_.A_button = state_.buttons[0];
            state_.B_button = state_.buttons[1];
            state_.X_button = state_.buttons[2];
            state_.Y_button = state_.buttons[3];
            state_.left_shoulder_button = state_.buttons[4];
            state_.right_shoulder_button = state_.buttons[5];
            state_.view_button = state_.buttons[6];
            state_.menu_button = state_.buttons[7];
            state_.Xbox_button = state_.buttons[8];
        }
        // Calculate frequency
        auto now = std::chrono::steady_clock::now();
        double elapsed_seconds = std::chrono::duration<double>(now - last_time).count();
        frequency_sum += 1.0 / elapsed_seconds;
        count++;
        last_time = now;

        // Print frequency every second
        if (count >= 50.0) {
            double average_frequency = frequency_sum / count;
            //            std::cout << "Frequency: " << average_frequency << " Hz\n";
            frequency_sum = 0.0;
            count = 0;
        }

        // Limit frequency to 50Hz
        std::this_thread::sleep_until(last_time + std::chrono::milliseconds(2));
    }

    close(fd);
}

void JoyStickHandler::updateState(const JoystickEvent &event) {
    std::lock_guard<std::mutex> lock(mutex_);
    switch (event.type & ~JS_EVENT_INIT) {
        case JS_EVENT_AXIS:
            if (event.number < state_.axes.size()) {
                state_.axes[event.number] = event.value;
            }
            break;
        case JS_EVENT_BUTTON:
            if (event.number < state_.buttons.size()) {
                state_.buttons[event.number] = event.value;
            }
            break;
        default:
            break;
    }
}

JoyStickHandler::~JoyStickHandler() {
    std::lock_guard<std::mutex> lock(mutex_);
    stop_ = true;
    if (joystick_thread_->joinable()) {
        joystick_thread_->join();
    }
}
