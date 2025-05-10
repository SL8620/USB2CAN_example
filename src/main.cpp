#include "usb2can.h"
#include <csignal>
#include <iostream>
#include <thread>
#include <chrono>

volatile bool keepRunning = true;
void signalHandler(int signum) { keepRunning = false; }

int main()
{
    std::signal(SIGINT, signalHandler);

    USB2CAN usb2can("/dev/ttyACM0");
    if (!usb2can.openUSB2CAN())
    {
        std::cerr << "无法打开 USB2CAN 设备" << std::endl;
        return -1;
    }

    std::cout << "USB2CAN 已启动，开始发送和接收测试..." << std::endl;

    // 发送线程
    std::thread sender([&usb2can]() {
        CanFrame testFrame;
        testFrame.m_CANChannel = CAN_Channel1;
        testFrame.m_CANType = Extended_CAN; // 可切换 Classic_CAN
        testFrame.id = 0x1ABCDE3;
        testFrame.dlc = 8;
        for (int i = 0; i < 8; ++i)
            testFrame.data[i] = i + 1;

        while (keepRunning)
        {
            usb2can.sendUSB2CAN(testFrame);
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
    });

    // 接收线程
    while (keepRunning)
    {
        CanFrame recvFrame;
        if (usb2can.readUSB2CAN(recvFrame))
        {
            std::cout << "[接收] "
                      << ((recvFrame.m_CANType == Extended_CAN) ? "扩展帧" : "标准帧")
                      << " | 通道: " << ((recvFrame.m_CANChannel == CAN_Channel1) ? "CAN1" : "CAN2")
                      << " | ID: 0x" << std::hex << recvFrame.id
                      << " | 长度: " << std::dec << (int)recvFrame.dlc
                      << " | 数据: ";
            for (int i = 0; i < recvFrame.dlc; ++i)
                std::cout << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << (int)recvFrame.data[i] << " ";
            std::cout << std::dec << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    sender.join();
    usb2can.closeUSB2CAN();
    std::cout << "测试结束，已关闭 USB2CAN" << std::endl;
    return 0;
}
