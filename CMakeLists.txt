cmake_minimum_required(VERSION 3.5)
project(YKS_SDK)

# 查找并添加第三方库目录
add_subdirectory(app/thirdparty)

# 查找 Boost 和 PkgConfig 包
find_package(Boost REQUIRED)
find_package(PkgConfig REQUIRED)


# 查找 ncurses 库
pkg_check_modules(NCURSES REQUIRED ncurses)

# 设置源文件
set(SOURCES
        app/motor_control.c
        app/motor_control.h
        app/math_ops.c
        app/math_ops.h
        app/transmit.cpp
        app/transmit.h
        app/config.h
        app/queue.h
        app/command.cpp
        app/command.h
)

# 创建 motor 库
add_library(motor ${SOURCES})
target_include_directories(motor PRIVATE
        app/thirdparty/soem/soem
        app/thirdparty/soem/osal
        app/thirdparty/soem/osal/linux
        app/thirdparty/soem/oshw/linux
        app/thirdparty/cpp-readline/src
)
target_link_libraries(motor PRIVATE soem)

# 强制使用 C++17 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF) # 如果你不想使用 GNU 扩展，可以将此选项设为 OFF

# 添加可执行文件 YKS_SDK
add_executable(YKS_SDK main.cpp JoyStickHandler.cpp Z1Legs.cpp BmsHandler.cpp SBusReceiver.cpp SocketSender.cpp SocketReceiver.cpp)

# 为 YKS_SDK 添加头文件搜索路径
target_include_directories(YKS_SDK PRIVATE
        ${Boost_INCLUDE_DIRS}
        ${NCURSES_INCLUDE_DIRS}
        app/thirdparty/soem/soem
        app/thirdparty/soem/osal
        app/thirdparty/soem/osal/linux
        app/thirdparty/soem/oshw/linux
        app/thirdparty/cpp-readline/src
)

# 链接所有需要的库到 YKS_SDK
target_link_libraries(YKS_SDK PRIVATE
        motor
        soem
        cpp-readline
        ${NCURSES_LIBRARIES}
)