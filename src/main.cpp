#include "can_usb_device.hpp"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>
#include <thread>
#include <chrono>

using namespace can_usb_driver;

// 打印CAN消息
void printCanMessage(const CanUsbDevice* dev, const CanMessage& msg) {
    std::lock_guard<std::mutex> lock(CanUsbDevice::coutMutex_);
    std::cout << "Received: Port=" << (int)msg.canPort
              << ", IDType=" << (msg.canIdType == CanId_extended ? "Extended" : "Classic")
              << ", ID=0x" << std::hex << std::setw(8) << std::setfill('0') << msg.id
              << ", Data=[";
    for (size_t i = 0; i < msg.data.size(); ++i) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)msg.data[i];
        if (i < msg.data.size() - 1) std::cout << " ";
    }
    std::cout << "]" << std::dec << std::endl;
}



int main() {
    // 初始化设备
    CanUsbDevice device("/dev/usb2can1", "USB2CAN");
    if (!device.open()) {
        std::cerr << "Failed to open device" << std::endl;
        return 1;
    }

    // 设置接收回调
    device.setReceiveCallback(printCanMessage);
    device.startReceiveThread();

    CanMessage msgSend;
    msgSend.canIdType = CanId_classic;
    msgSend.canPort = CanPort_1;
    msgSend.id = 0x22;
    msgSend.data = {0x12, 0x23, 0x34, 0x45, 0x12, 0x23, 0x34, 0x45};
    for(int i=0;i<100000;i++)
    {
        device.sendCanMessage(msgSend);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    while (true)
    {
        /* code */
    }
    
    
    device.stopReceiveThread();
    device.close();
    return 0;
}