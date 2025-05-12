#include "CanUsbDevice.h"
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <dirent.h>
#include <cstring>
#include <chrono>
#include <thread>
#include <algorithm>

static std::string findDefaultDevice() 
{
    DIR* dir = opendir("/dev");
    if (!dir) return "";
    struct dirent* ent;
    while ((ent = readdir(dir)) != nullptr) 
    {
        if (strncmp(ent->d_name, "ttyACM", 6) == 0) 
        {
            closedir(dir);
            return "/dev/" + std::string(ent->d_name);
        }
    }
    closedir(dir);
    return "";
}

CanUsbDevice::CanUsbDevice(const std::string& devicePath) : devicePath_(devicePath.empty() ? findDefaultDevice() : devicePath), fd_(-1), running_(false) {}

CanUsbDevice::~CanUsbDevice() {
    stopReceiveThread();
    close();
}

bool CanUsbDevice::open() 
{
    fd_ = ::open(devicePath_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd_ < 0) 
    {
        std::cerr << "Failed to open " << devicePath_ << "\n";
        return false;
    }

    termios tty{};
    if (tcgetattr(fd_, &tty) != 0) 
    {
        std::cerr << "Failed to get termios\n";
        return false;
    }

    // 串口设置
    tty.c_iflag = 0;                             // 输入模式
    tty.c_oflag = 0;                             // 输出模式
    tty.c_cflag = CREAD | CLOCAL | CS8;          // 控制模式：8位、无校验、启用接收、本地连接
    tty.c_cflag &= ~CRTSCTS;                     // 关闭硬件流控
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);      // 禁用软件流控
    tty.c_lflag = 0;                             // 非规范模式

    cfsetospeed(&tty, B115200);                  // 输出波特率
    cfsetispeed(&tty, B115200);                  // 输入波特率

    tty.c_cc[VMIN] = 0;                          // 非阻塞读取
    tty.c_cc[VTIME] = 10;                        // 超时时间 1 秒（10 × 100ms）

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) 
    {
        std::cerr << "Failed to set termios\n";
        return false;
    }

    return true;
}


void CanUsbDevice::close() 
{
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool CanUsbDevice::sendCanMessage(const CanMessage& msg) 
{
    std::lock_guard<std::mutex> lock(ioMutex_);
    if (fd_ < 0) return false;

    std::vector<uint8_t> buf;
    buf.push_back(0xA5);
    buf.push_back(0xB0 | (msg.canPort & 0x03));
    buf.push_back((msg.data.size() & 0x0F) | (msg.isExtended ? 0x10 : 0x00));

    if (msg.isExtended) 
    {
        buf.push_back((msg.id >> 24) & 0xFF);
        buf.push_back((msg.id >> 16) & 0xFF);
        buf.push_back((msg.id >> 8) & 0xFF);
        buf.push_back(msg.id & 0xFF);
    } 
    else 
    {
        buf.push_back(msg.id & 0xFF);
    }

    buf.insert(buf.end(), msg.data.begin(), msg.data.end());
    buf.push_back(0x5A);

    return ::write(fd_, buf.data(), buf.size()) == (ssize_t)buf.size();
}

void CanUsbDevice::startReceiveThread() 
{
    running_ = true;
    recvThread_ = std::thread(&CanUsbDevice::receiveLoop, this);
}

void CanUsbDevice::stopReceiveThread() 
{
    running_ = false;
    if (recvThread_.joinable()) {
        recvThread_.join();
    }
}

void CanUsbDevice::receiveLoop() 
{
    std::vector<uint8_t> buffer;
    uint8_t temp[64];

    while (running_) 
    {
        {
            std::lock_guard<std::mutex> lock(ioMutex_);
            ssize_t n = read(fd_, temp, sizeof(temp));
            if (n > 0) 
            {
                buffer.insert(buffer.end(), temp, temp + n);
                CanMessage msg;
                while (parseBuffer(buffer, msg)) 
                {
                    std::cout << "Received CAN msg: port=" << (int)msg.canPort
                              << " id=" << std::hex << msg.id << std::dec
                              << " data=[";
                    for (auto b : msg.data) std::cout << " " << std::hex << (int)b;
                    std::cout << " ]\n";
                }
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(DEFAULT_RECV_INTERVAL_MS));
    }
}

bool CanUsbDevice::parseBuffer(std::vector<uint8_t>& buffer, CanMessage& msg) 
{
    if (buffer.size() < 4) return false;

    auto it = std::find(buffer.begin(), buffer.end(), 0xA5);
    if (it == buffer.end()) 
    {
        buffer.clear();
        return false;
    }

    if (std::distance(it, buffer.end()) < 4) return false;

    size_t start = std::distance(buffer.begin(), it);
    uint8_t ctrl = buffer[start + 1];
    uint8_t flags = buffer[start + 2];
    bool isExt = (flags & 0x10);
    uint8_t dlc = flags & 0x0F;

    size_t idLen = isExt ? 4 : 1;
    size_t totalLen = 1 + 1 + 1 + idLen + dlc + 1;

    if (start + totalLen > buffer.size()) 
    {
        return false;
    }

    if (buffer[start + totalLen - 1] != 0x5A) 
    {
        buffer.erase(buffer.begin(), buffer.begin() + start + 1);
        return false;
    }

    msg.canPort = ctrl & 0x03;
    msg.isExtended = isExt;
    msg.id = 0;
    if (isExt) 
    {
        msg.id = (buffer[start + 3] << 24) | (buffer[start + 4] << 16)
                 | (buffer[start + 5] << 8) | buffer[start + 6];
    } 
    else 
    {
        msg.id = buffer[start + 3];
    }

    size_t dataStart = start + 3 + idLen;
    msg.data.assign(buffer.begin() + dataStart, buffer.begin() + dataStart + dlc);

    buffer.erase(buffer.begin(), buffer.begin() + start + totalLen);
    return true;
}
