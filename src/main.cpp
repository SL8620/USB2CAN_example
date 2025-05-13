#include "CanUsbDevice.h"
#include <csignal>
#include <atomic>
#include <iostream>
#include <thread>
#include <chrono>
#include "CanUsbDevice.h"  // 引入库的头文件

std::atomic<bool> running(true);

// 信号处理函数：捕获 Ctrl+C
void signalHandler(int signum) 
{
    std::cout << "\nInterrupt signal (" << signum << ") received. Stopping...\n";
    running = false;
}

int main() {
    // 注册信号处理
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    CanUsbDevice dev0("/dev/ttyACM0", "dev0");
    CanUsbDevice dev1("/dev/ttyACM1", "dev1");

    dev0.setReceiveCallback([](const CanUsbDevice* self, const CanMessage& msg)
    {
        std::lock_guard<std::mutex> lock(self->coutMutex_);
        std::cout << "Device : " << self->devName_;
        std::cout << "  Received CAN msg: port=" << (int)msg.canPort
                    << " id=" << std::hex << msg.id << std::dec
                    << " data=[";
        for (auto b : msg.data) std::cout << " " << std::hex << (int)b;
        std::cout << " ]\n";
    });

    dev1.setReceiveCallback([](const CanUsbDevice* self, const CanMessage& msg)
    {
        std::lock_guard<std::mutex> lock(self->coutMutex_);
        std::cout << "Device : " << self->devName_;
        std::cout << "  Received CAN msg: port=" << (int)msg.canPort
                    << " id=" << std::hex << msg.id << std::dec
                    << " data=[";
        for (auto b : msg.data) std::cout << " " << std::hex << (int)b;
        std::cout << " ]\n";
    });

    if (!dev1.open()) {
        std::cerr << "❌ Failed to open CAN USB device\n";
        return 1;
    }

    if (!dev0.open()) {
        std::cerr << "❌ Failed to open CAN USB device\n";
        return 1;
    }

    dev0.startReceiveThread();
    dev1.startReceiveThread();
    std::cout << "✅ Device opened and receive thread started\n";

    while(running)
    {
        CanMessage msgSend;
        msgSend.canIdType = CanId_extended;
        msgSend.canPort = CanPort_1;
        msgSend.id = 0x2345;
        msgSend.data = {0x12,0x23,0x34,0x45};
        dev0.sendCanMessage(msgSend);
        msgSend.id = 0x1234;
        dev1.sendCanMessage(msgSend);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    dev0.stopReceiveThread();
    dev0.close();

    dev1.stopReceiveThread();
    dev1.close();

    std::cout << "✅ Device closed gracefully\n";
    return 0;
}
