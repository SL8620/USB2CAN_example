#include "usb2can.h"
#include <iostream>
#include <csignal>

// std::atomic<bool> keepRunning(true);

// void signalHandler(int)
// {
//     keepRunning = false;
//     std::cout << "\n终止程序..." << std::endl;
// }

// int main()
// {
//     // 捕获 Ctrl+C 等终止信号
//     std::signal(SIGINT, signalHandler);

//     // 创建 USB2CAN 对象，使用默认串口和波特率
//     USB2CAN usb2can("/dev/ttyACM0");

//     // 打开串口设备
//     if (!usb2can.openUSB2CAN())
//     {
//         std::cerr << "无法打开 USB2CAN 设备" << std::endl;
//         return -1;
//     }

//     std::cout << "USB2CAN 设备已打开，开始监听 CAN 报文..." << std::endl;

//     // 主循环，不断读取队列中的 CAN 报文
//     while (keepRunning)
//     {
//         CanFrame frame;
//         if (usb2can.readUSB2CAN(frame))
//         {
//             std::cout << "接收到报文: "
//                       << "ID: 0x" << std::hex << frame.id
//                       << " DLC: " << std::dec << (int)frame.dlc
//                       << " Data: ";

//             for (int i = 0; i < frame.dlc; ++i)
//                 std::cout << std::hex << std::uppercase << (int)frame.data[i] << " ";

//             std::cout << " Channel: " << ((frame.m_CANChannel == CAN_Channel1) ? "CAN1" : "CAN2") << std::endl;
//         }

//         // 延时避免高 CPU 占用
//         std::this_thread::sleep_for(std::chrono::milliseconds(10));
//     }

//     usb2can.closeUSB2CAN();
//     std::cout << "已关闭 USB2CAN 设备" << std::endl;
//     return 0;
// }

int main() {
    USB2CAN usb2can("/dev/ttyACM0", 921600);  // 根据实际串口名修改

    if (!usb2can.openUSB2CAN()) {
        std::cerr << "Failed to open USB2CAN device." << std::endl;
        return -1;
    }

    std::cout << "Device opened successfully." << std::endl;

    CanFrame frame;
    frame.dlc = 8;
    frame.m_CANType = Classic_CAN;
    frame.m_CANChannel = CAN_Channel1;

    // 填充固定数据
    for (int i = 0; i < 8; ++i) {
        frame.data[i] = i;
    }

    // 连续发送100个报文
    for (uint32_t i = 0; i < 100; ++i) {
        frame.id = i;

        if (!usb2can.sendUSB2CAN(frame)) {
            std::cerr << "Failed to send frame with ID: " << i << std::endl;
        } else {
            std::cout << "Sent frame with ID: " << i << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));  // 20ms 间隔
    }

    usb2can.closeUSB2CAN();
    std::cout << "Device closed." << std::endl;

    return 0;
}