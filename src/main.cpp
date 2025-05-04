/*** 
 * @Author: SL8620
 * @Date: 2025-04-30 16:09:20
 * @LastEditTime: 2025-05-04 15:55:49
 * @LastEditors: SL8620
 * @Description: 
 * @FilePath: \USB2CAN\src\main.cpp
 * @
 */
#include "USB2CAN.h"
#include <iostream>
#include <csignal>

std::atomic<bool> keepRunning(true);

void signalHandler(int)
{
    keepRunning = false;
    std::cout << "\n终止程序..." << std::endl;
}

int main()
{
    // 捕获 Ctrl+C 等终止信号
    std::signal(SIGINT, signalHandler);

    // 创建 USB2CAN 对象，使用默认串口和波特率
    USB2CAN usb2can;

    // 打开串口设备
    if (!usb2can.openUSB2CAN())
    {
        std::cerr << "无法打开 USB2CAN 设备" << std::endl;
        return -1;
    }

    std::cout << "USB2CAN 设备已打开，开始监听 CAN 报文..." << std::endl;

    // 主循环，不断读取队列中的 CAN 报文
    while (keepRunning)
    {
        CanFrame frame;
        if (usb2can.readUSB2CAN(frame))
        {
            std::cout << "接收到报文: "
                      << "ID: 0x" << std::hex << frame.id
                      << " DLC: " << std::dec << (int)frame.dlc
                      << " Data: ";

            for (int i = 0; i < frame.dlc; ++i)
                std::cout << std::hex << std::uppercase << (int)frame.data[i] << " ";

            std::cout << " Channel: " << ((frame.m_CANChannel == CAN_Channel1) ? "CAN1" : "CAN2") << std::endl;
        }

        // 延时避免高 CPU 占用
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    usb2can.closeUSB2CAN();
    std::cout << "已关闭 USB2CAN 设备" << std::endl;
    return 0;
}
