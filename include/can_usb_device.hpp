#pragma once

#include <thread>
#include <atomic>
#include <vector>
#include <string>
#include <mutex>
#include <functional>
#include <condition_variable>
#include <array>
#include "ring_buffer.hpp"

#define CanPort_1      0x01
#define CanPort_2      0x02

#define CanId_classic  0x00
#define CanId_extended 0x01

namespace can_usb_driver 
{
    struct CanMessage 
    {
        uint8_t canPort;
        uint8_t canIdType;
        uint32_t id;
        std::vector<uint8_t> data;
    };

    class CanUsbDevice 
    {
    public:
        using CanMessageCallback = std::function<void(const CanUsbDevice*, const CanMessage&)>;

        std::string devName_;
        static std::mutex coutMutex_;
        
        CanUsbDevice(const std::string& devicePath = "", const std::string& devName = "");
        ~CanUsbDevice();

        bool open();
        void close();

        bool sendCanMessage(const CanMessage& msg);
        void startReceiveThread();
        void stopReceiveThread();
        void setReceiveCallback(CanMessageCallback cb);
        
        uint64_t getFramesSent() const { return framesSent_; }
        uint64_t getFramesReceived() const { return framesReceived_; }
        uint64_t getFramesInvalid() const { return framesInvalid_; }

    private:
        struct TxQueue 
        {
            std::mutex mtx;
            std::vector<std::vector<uint8_t>> queue;
        };
        
        void txThreadFunc(int interval_us);
        void receiveLoop();
        ssize_t writeAll(const uint8_t* data, size_t len);
        bool tryParseOneFrame(RingBuffer<16384>& rb, CanMessage& msg);

        std::string devicePath_;
        int fd_ = -1;
        std::thread recvThread_;
        std::thread tx_thread_;
        CanMessageCallback receiveCallback_;
        
        std::atomic<uint64_t> framesSent_{0};
        std::atomic<uint64_t> framesReceived_{0};
        std::atomic<uint64_t> framesInvalid_{0};
        
        std::array<TxQueue, 3> tx_queues_;
        std::condition_variable tx_cv_;
        std::mutex tx_cv_mtx_;
        std::atomic<bool> runningTx_{false};
        std::atomic<bool> runningRx_{false};
    };
} // namespace can_usb_driver