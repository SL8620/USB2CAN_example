/*** 
 * @Author: SL8620
 * @Date: 2025-04-30 16:09:37
 * @LastEditTime: 2025-05-04 15:54:22
 * @LastEditors: SL8620
 * @Description: 
 * @FilePath: \USB2CAN\src\usb2can.cpp
 * @
 */

#include "usb2can.h"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <cstring>
#include <stdexcept>

// 构造函数，初始化串口名称和波特率，准备资源
USB2CAN::USB2CAN(const std::string& port, int baudrate) : portName(port), baudrate(baudrate), fd(-1), running(false) {}

// 析构函数，确保资源释放和串口关闭
USB2CAN::~USB2CAN() 
{
    closeUSB2CAN();
}

// 打开并配置串口，启动读线程
bool USB2CAN::openUSB2CAN() 
{
    fd = open(portName.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) 
    {
        std::cerr << "无法打开设备 " << portName << ": " << strerror(errno) << std::endl;
        return false;
    }

    // 清除非阻塞标志，后续读取操作阻塞等待数据
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags >= 0) 
    {
        fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);
    }

    if (!configurePort()) 
    {
        std::cerr << "串口配置失败！" << std::endl;
        close(fd);
        return false;
    }

    running = true;
    readerThread = std::thread(&USB2CAN::readThreadFunc, this);

    std::cout << "设备打开成功：" << portName << std::endl;
    return true;
}

// 停止读取线程，关闭串口
void USB2CAN::closeUSB2CAN() 
{
    running = false;
    if (readerThread.joinable()) readerThread.join();
    if (fd >= 0) 
    {
        close(fd);
        fd = -1;
    }
}

// 配置串口参数（使用 termios），设定波特率、数据位、无校验等
bool USB2CAN::configurePort() 
{
    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) 
    {
        std::cerr << "获取串口参数失败: " << strerror(errno) << std::endl;
        return false;
    }

    // 输入模式
    tty.c_iflag = 0;

    // 输出模式
    tty.c_oflag = 0;

    // 控制模式：本地连接、启用接收、8位数据位、无校验位
    tty.c_cflag = CREAD | CLOCAL | CS8;

    // 关闭硬件流控
    tty.c_cflag &= ~CRTSCTS;

    // 屏蔽软件流控
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);

    // 本地模式（不使用 canonical 模式）
    tty.c_lflag = 0;

    // 设置读取超时
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 10; // 1 秒超时

    // 设置波特率
    cfsetispeed(&tty, B921600);
    cfsetospeed(&tty, B921600);

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cerr << "设置串口参数失败: " << strerror(errno) << std::endl;
        return false;
    }

    return true;
}

// 通过USB发送CAN报文
bool USB2CAN::sendUSB2CAN(const CanFrame& frame) 
{
    if (fd < 0) return false;

    std::vector<uint8_t> buffer;

    // 头部：0xB0 | 通道号
    uint8_t portByte = 0xB0 | static_cast<uint8_t>(frame.m_CANChannel);
    buffer.push_back(portByte);

    if (frame.m_CANType == Classic_CAN) 
    {
        // 标准 CAN: ID 1 字节
        buffer.push_back(static_cast<uint8_t>(frame.id & 0xFF));
    } 
    else if (frame.m_CANType == Extended_CAN) 
    {
        // 扩展 CAN: ID 4 字节（大端序）
        buffer.push_back((frame.id >> 24) & 0xFF);
        buffer.push_back((frame.id >> 16) & 0xFF);
        buffer.push_back((frame.id >> 8) & 0xFF);
        buffer.push_back(frame.id & 0xFF);
    } 
    else 
    {
        return false; // 未知类型
    }

    // 数据段
    for (uint8_t i = 0; i < frame.dlc; ++i) 
    {
        buffer.push_back(frame.data[i]);
    }

    // 总长度检查
    ssize_t written = write(fd, buffer.data(), buffer.size());
    return written == static_cast<ssize_t>(buffer.size());
}

// 读取函数：从接收队列中取出一个CanFrame
bool USB2CAN::readUSB2CAN(CanFrame& frame) 
{
    std::lock_guard<std::mutex> lock(queueMutex);
    if (recvQueue.empty())
    {
        return false;
    }
    frame = recvQueue.front();   // 拷贝队首
    recvQueue.pop();             // 弹出队首
    return true;
}

// 后台线程：从串口读取原始字节，处理粘包拆包，解析成CanFrame并入队
void USB2CAN::readThreadFunc() 
{
    uint8_t buffer[256];    // 数据缓存区
    size_t bufPos = 0;      // 缓存长度

    while (running) 
    {
        int n = read(fd, buffer + bufPos, sizeof(buffer) - bufPos);
        if (n > 0) 
        {
            bufPos += n;
            size_t i = 0;
            // 循环提取所有完整帧
            while (bufPos - i >= 12) 
            {
                // 查找帧头0xA5
                if (buffer[i] != 0xA5) 
                {
                    ++i;
                    continue;
                }
                // 检查帧尾0x5A
                if (buffer[i + 11] != 0x5A) 
                {
                    ++i;
                    continue;
                }

                // 构造一条有效帧
                CanFrame frame;
                frame.m_CANChannel = (buffer[i + 1] & 0x0F) == 1 ? CAN_Channel1 : CAN_Channel2;
                frame.id           = buffer[i + 2];              // 标准ID
                frame.dlc          = 8;                          // 固定8字节
                memcpy(frame.data, buffer + i + 3, 8);           // 拷贝数据
                frame.m_CANType    = Classic_CAN;

                // 入队
                {
                    std::lock_guard<std::mutex> lock(queueMutex);
                    recvQueue.push(frame);
                }
                i += 12;  // 跳过已处理帧
            }
            // 移除已处理字节
            if (i > 0) 
            {
                memmove(buffer, buffer + i, bufPos - i);
                bufPos -= i;
            }
        } 
        else 
        {
            usleep(1000);  // 无数据时稍作延时
        }
    }
}
