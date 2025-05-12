/*** 
 * @Author: SL8620
 * @Date: 2025-05-11 21:50:34
 * @LastEditTime: 2025-05-12 09:52:28
 * @LastEditors: SL8620
 * @Description: 
 * @FilePath: \USB2CAN\include\CanUsbDevice.h
 * @
 */
#ifndef CAN_USB_DEVICE_H
#define CAN_USB_DEVICE_H

#include <thread>
#include <atomic>
#include <vector>
#include <string>
#include <mutex>

#define DEFAULT_RECV_INTERVAL_MS 2  // 接收线程默认运行间隔（可宏定义控制）

struct CanMessage {
    uint8_t canPort;
    bool isExtended;
    uint32_t id;
    std::vector<uint8_t> data;
};

class CanUsbDevice {
public:
    CanUsbDevice(const std::string& devicePath = "");
    ~CanUsbDevice();

    bool open();
    void close();

    bool sendCanMessage(const CanMessage& msg);
    void startReceiveThread();
    void stopReceiveThread();

private:
    void receiveLoop();
    bool parseBuffer(std::vector<uint8_t>& buffer, CanMessage& msg);

    std::string devicePath_;
    int fd_;
    std::thread recvThread_;
    std::atomic<bool> running_;
    std::mutex ioMutex_;
};

#endif
