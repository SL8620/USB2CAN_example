#ifndef CAN_USB_DEVICE_H
#define CAN_USB_DEVICE_H

#include <thread>
#include <atomic>
#include <vector>
#include <string>
#include <mutex>
#include <string>
#include <functional>

#define DEFAULT_RECV_INTERVAL_MS 1  // 接收线程默认运行间隔（可宏定义控制）

#define CanPort_1      0x01
#define CanPort_2      0x02
#define CanId_classic  0x00
#define CanId_extended 0x01

class CanUsbDevice;

struct CanMessage 
{
    uint8_t canPort;
    uint8_t canIdType;
    uint32_t id;
    std::vector<uint8_t> data;
};

using CanMessageCallback = std::function<void(const CanUsbDevice*,const CanMessage&)>;

class CanUsbDevice 
{
public:
    std::string devName_;
    static std::mutex coutMutex_;
    CanUsbDevice(const std::string& devicePath = "", const std::string& devName_= "");
    ~CanUsbDevice();

    bool open();
    void close();

    bool sendCanMessage(const CanMessage& msg);
    void startReceiveThread();
    void stopReceiveThread();
    void setReceiveCallback(CanMessageCallback cb); 

private:
    void receiveLoop();
    bool parseBuffer(std::vector<uint8_t>& buffer, CanMessage& msg);

    std::string devicePath_;
    int fd_;
    std::thread recvThread_;
    std::atomic<bool> running_;
    std::mutex ioMutex_;
    CanMessageCallback receiveCallback_;
};

#endif
