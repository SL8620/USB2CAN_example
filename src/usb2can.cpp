/*** 
 * @Author: SL8620
 * @Date: 2025-04-30 16:09:37
 * @LastEditTime: 2025-05-10 14:50:17
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

    // 帧头（CAN通道）
    uint8_t head = (frame.m_CANChannel == CAN_Channel1) ? 0xB1 : 0xB2;
    buffer.push_back(head);

    // AB 字节：高4位为数据长度，低1位为 ID 类型
    uint8_t ab = ((frame.dlc & 0x0F) << 4) | (frame.m_CanIdType == CanId_Extended ? 0x01 : 0x00);
    buffer.push_back(ab);

    // ID（标准1字节，扩展4字节）
    if (frame.m_CanIdType == CanId_Extended) 
    {
        buffer.push_back((frame.id >> 24) & 0xFF);
        buffer.push_back((frame.id >> 16) & 0xFF);
        buffer.push_back((frame.id >> 8) & 0xFF);
        buffer.push_back(frame.id & 0xFF);
    } 
    else 
    {
        buffer.push_back(frame.id & 0xFF);
    }

    // 数据
    for (int i = 0; i < frame.dlc; ++i)
    {
        buffer.push_back(frame.data[i]);
    }
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
    uint8_t buffer[256];
    size_t bufPos = 0;

    while (running) 
    {
        int n = read(fd, buffer + bufPos, sizeof(buffer) - bufPos);
        if (n > 0) 
        {
            bufPos += n;
            size_t i = 0;

            while (bufPos - i >= 6) // 最小长度校验
            {
                if (buffer[i] != 0xA5) 
                {
                    ++i;
                    continue;
                }

                if (i + 3 >= bufPos) break;

                uint8_t cmd = buffer[i + 1]; // 0xB0 | port
                uint8_t ab  = buffer[i + 2];
                uint8_t dlc = (ab >> 4) & 0x0F;
                bool isExt  = (ab & 0x01);
                uint8_t idLen = isExt ? 4 : 1;

                size_t frameLen = 1 + 1 + 1 + idLen + dlc + 1; // A5 + cmd + ab + id + data + 5A

                if (bufPos - i < frameLen) break; // 不完整帧

                if (buffer[i + frameLen - 1] != 0x5A) {
                    ++i;
                    continue;
                }

                // 正确帧，开始解析
                CanFrame frame;
                frame.m_CANChannel = ((cmd & 0x0F) == 1) ? CAN_Channel1 : CAN_Channel2;
                frame.dlc = dlc;
                frame.m_CanIdType = isExt ? CanId_Extended : CanId_Classic;

                if (isExt) 
                {
                    // 解析扩展 ID（4 字节）
                    frame.id = (buffer[i + 3] << 24) | (buffer[i + 4] << 16) |
                            (buffer[i + 5] << 8) | buffer[i + 6];
                    // 数据段从 id 后面的 4 字节开始
                    memcpy(frame.data, buffer + i + 7, dlc);  // id 后面是 4 字节 ID，再加上数据
                } 
                else 
                {
                    // 解析标准 ID（1 字节）
                    frame.id = buffer[i + 3];
                    // 数据段从 id 后面 1 字节开始
                    memcpy(frame.data, buffer + i + 4, dlc);  // id 后面是 1 字节 ID，再加上数据
                }

                {
                    std::lock_guard<std::mutex> lock(queueMutex);
                    recvQueue.push(frame);
                }

                i += frameLen;
            }

            if (i > 0) 
            {
                memmove(buffer, buffer + i, bufPos - i);
                bufPos -= i;
            }
        } 
        else 
        {
            usleep(1000);
        }
    }
}
