/*** 
 * @Author: SL8620
 * @Date: 2025-04-30 16:09:55
 * @LastEditTime: 2025-05-04 15:53:33
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

enum CanType
{
    Classic_CAN,
    Extended_CAN
};

enum CanChannel
{
    CAN_Channel1=1,
    CAN_Channel2=2
};

struct CanFrame 
{
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];

    CanType m_CANType;
    CanChannel m_CANChannel;
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
        std::string frameToString(const CanFrame& frame);
    
        std::string portName;
        int baudrate;
        int fd;
        std::atomic<bool> running;
        std::thread readerThread;
        std::mutex queueMutex;
        std::queue<CanFrame> recvQueue;
};

#endif // USB2CAN_H
