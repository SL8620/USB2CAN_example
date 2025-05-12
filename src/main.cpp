/*** 
 * @Author: SL8620
 * @Date: 2025-05-06 08:28:25
 * @LastEditTime: 2025-05-12 08:25:57
 * @LastEditors: SL8620
 * @Description: 
 * @FilePath: \USB2CAN\src\main.cpp
 * @
 */
#include "CanUsbDevice.h"
#include <iostream>

int main() 
{
    CanUsbDevice dev;

    if (!dev.open()) {
        std::cerr << "Failed to open CAN USB device\n";
        return 1;
    }

    dev.startReceiveThread();

    CanMessage msg;
    msg.canPort = 1;
    msg.isExtended = false;
    msg.id = 0x123;
    msg.data = {0x11, 0x22, 0x33, 0x44};

    dev.sendCanMessage(msg);

    std::this_thread::sleep_for(std::chrono::seconds(10));
    dev.stopReceiveThread();

    return 0;
}
