#include <iostream>
#include <thread>
#include <atomic>
#include <signal.h>
#include <chrono>
#include "usb2can.h"

// 终止程序时的信号处理函数
bool keepRunning = true;
void signalHandler(int signum) {
    keepRunning = false;
    std::cout << "\nInterrupt signal (" << signum << ") received. Shutting down..." << std::endl;
}

// 发送测试线程函数
void sendTestThread(USB2CAN& usb2can) {
    CanFrame sendFrame1,sendFrame2;
    sendFrame1.m_CanIdType = CanId_Classic;  // 使用标准 ID
    sendFrame1.id = 0x23;  // CAN ID
    sendFrame1.dlc = 8;  // 数据长度为 8 字节
    sendFrame1.m_CANChannel = CAN_Channel1;  // 发送到 CAN1

    sendFrame2.m_CanIdType = CanId_Classic;  // 使用标准 ID
    sendFrame2.id = 0x23;  // CAN ID
    sendFrame2.dlc = 6;  // 数据长度为 8 字节
    sendFrame2.m_CANChannel = CAN_Channel2;  // 发送到 CAN2
   

    // 填充数据
    for (int i = 0; i < 8; ++i) {
        sendFrame1.data[i] = i;  // 填充数据
    }

    for (int i = 0; i < 6; ++i) {
        sendFrame2.data[i] = i;  // 填充数据
    }

    while (keepRunning) 
    {
        usb2can.sendUSB2CAN(sendFrame1);
        usb2can.sendUSB2CAN(sendFrame2);
        // 延时一段时间后继续发送
        std::this_thread::sleep_for(std::chrono::milliseconds(500));  // 每秒发送一次
    }
}

// 接收测试函数
void receiveTest(USB2CAN& usb2can) {
    while (keepRunning) {
        CanFrame frame;
        if (usb2can.readUSB2CAN(frame)) {
            // 打印接收到的 CAN 帧的信息
            std::cout << "接收到报文: "
                      << "ID: 0x" << std::hex << frame.id
                      << " DLC: " << std::dec << (int)frame.dlc
                      << " Frame Type: " << ((frame.m_CanIdType == CanId_Classic) ? "Standard" : "Extended")
                      << " Data: ";

            for (int i = 0; i < frame.dlc; ++i) {
                std::cout << std::hex << std::uppercase << (int)frame.data[i] << " ";
            }
            std::cout << " Channel: " << ((frame.m_CANChannel == CAN_Channel1) ? "CAN1" : "CAN2") << std::endl;
        }

        // 延时避免高 CPU 占用
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

int main() {
    // 捕获 Ctrl+C 等终止信号
    std::signal(SIGINT, signalHandler);

    // 创建 USB2CAN 对象，使用默认串口和波特率
    USB2CAN usb2can("/dev/ttyACM0");

    // 打开串口设备
    if (!usb2can.openUSB2CAN()) {
        std::cerr << "无法打开 USB2CAN 设备" << std::endl;
        return -1;
    }

    std::cout << "USB2CAN 设备已打开" << std::endl;

    // 创建并启动发送线程
    std::thread sendThread(sendTestThread, std::ref(usb2can));

    // 启动接收函数
    receiveTest(usb2can);

    // 等待发送线程结束
    sendThread.join();

    usb2can.closeUSB2CAN();
    std::cout << "已关闭 USB2CAN 设备" << std::endl;

    return 0;
}
