/*** 
 * @Author: SL8620
 * @Date: 2025-04-30 16:09:55
 * @LastEditTime: 2025-04-30 16:12:58
 * @LastEditors: SL8620
 * @Description: 
 * @FilePath: \USB2CAN\include\usb2can.h
 * @
 */

 #ifndef USB2CAN_H
#define USB2CAN_H

#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include <queue>
#include <cstdint>
#include <vector>

struct CanFrame 
{
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];
    uint16_t crc; // optional
};

class USB2CAN 
{
public:
    USB2CAN(const std::string& port = "/dev/ttyACM0", int baudrate = 921600);
    ~USB2CAN();

    bool openUSB2CAN();
    void closeUSB2CAN();

    bool sendUSB2CAN(const CanFrame& frame);
    bool readUSB2CAN(CanFrame& frame);

private:
    void readThreadFunc();
    bool configurePort();

    std::string portName;
    int baudrate;
    int fd;

    std::thread readerThread;
    std::atomic<bool> running;

    std::mutex queueMutex;
    std::queue<CanFrame> recvQueue;

    // 帧解析辅助
    bool parseFrame(const std::string& line, CanFrame& frame);
    std::string frameToString(const CanFrame& frame);
    uint16_t computeCRC(const uint8_t* data, size_t len);
};

#endif // USB2CAN_H
