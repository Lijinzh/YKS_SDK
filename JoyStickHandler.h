//
// Created by gold on 25-2-26.
//

#ifndef YKS_SDK_JOYSTICKHANDLER_H
#define YKS_SDK_JOYSTICKHANDLER_H

#include <string>
#include <memory>
#include <thread>
#include <array>
#include <mutex>
#include <atomic>
#include <chrono>
#include <fcntl.h>
#include <unistd.h>
#include <linux/joystick.h>
#include <cerrno>
#include <iostream>

typedef unsigned char u8;

struct ff_pack_t {
    char cmd;

    struct {
        u8 weak: 1;
        u8 strong: 1;
        u8 right: 1;
        u8 left: 1;
    } enable;

    struct {
        u8 left;
        u8 right;
        u8 strong;
        u8 weak;
    } strength;

    struct {
        u8 sustain_10ms;
        u8 release_10ms;
        u8 loop_count;
    } pulse;
};

struct JoystickEvent {
    uint32_t time;
    int16_t value;
    uint8_t type;
    uint8_t number;
};

struct JoystickState {
    std::array<int16_t, 8> axes;
    std::array<uint8_t, 11> buttons;
    int left_trigger; //左触发按键
    int right_trigger; //右触发按键
    int left_stick_x; //左摇杆
    int left_stick_y; //左摇杆
    int right_stick_x; //右摇杆
    int right_stick_y; //右摇杆
    int DPAD_x; //十字键左右
    int DPAD_y; //十字键上下
    int X_button; //X按钮
    int Y_button; //Y按钮
    int A_button; //A按钮
    int B_button; //B按钮
    int left_shoulder_button; //左肩键
    int right_shoulder_button; //右肩键
    int Xbox_button; //Xbox按钮
    int menu_button; //菜单按钮
    int view_button; //显示按钮
};

class JoyStickHandler {
public:
    explicit JoyStickHandler(const char *device);

    ~JoyStickHandler();

    JoystickState getState() const;

    static void print_state(const JoystickState &state);

private:
    void updateState(const JoystickEvent &event);

    void run();

    const char *device_;
    JoystickState state_{};
    ff_pack_t ff_pack{}; //用于实现手柄的力反馈功能
    std::shared_ptr<std::thread> joystick_thread_;
    mutable std::mutex mutex_;
    std::atomic<bool> stop_;
};


#endif //YKS_SDK_JOYSTICKHANDLER_H
