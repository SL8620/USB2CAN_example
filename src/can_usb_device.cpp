#include "can_usb_device.hpp"
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <dirent.h>
#include <cstring>
#include <chrono>
#include <thread>
#include <algorithm>

using namespace can_usb_driver;

std::mutex CanUsbDevice::coutMutex_;

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

CanUsbDevice::CanUsbDevice(const std::string& devicePath, const std::string& deviceName)
    : devicePath_(devicePath.empty() ? findDefaultDevice() : devicePath), 
      devName_(deviceName), 
      fd_(-1) {}

CanUsbDevice::~CanUsbDevice() 
{
    stopReceiveThread();
    close();
}

bool CanUsbDevice::open() 
{
    fd_ = ::open(devicePath_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd_ < 0) 
    {
        return false;
    }

    termios tty{};
    if (tcgetattr(fd_, &tty) != 0) 
    {
        return false;
    }

    tty.c_iflag = 0;
    tty.c_oflag = 0;
    tty.c_cflag = CREAD | CLOCAL | CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_lflag = 0;

    cfsetospeed(&tty, B921600);
    cfsetispeed(&tty, B921600);

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) 
    {
        std::lock_guard<std::mutex> lock(coutMutex_);
        std::cerr << "Failed to set termios\n";
        return false;
    }

    runningTx_ = true;
    tx_thread_ = std::thread(&CanUsbDevice::txThreadFunc, this, 100);

    return true;
}

void CanUsbDevice::close() 
{
    runningTx_ = false;
    tx_cv_.notify_all();
    if (tx_thread_.joinable()) tx_thread_.join();

    runningRx_ = false;
    if (recvThread_.joinable()) recvThread_.join();

    if (fd_ >= 0) 
    {
        ::close(fd_);
        fd_ = -1;
    }
}

void CanUsbDevice::setReceiveCallback(CanMessageCallback cb) 
{
    receiveCallback_ = std::move(cb);
}

bool CanUsbDevice::sendCanMessage(const CanMessage& msg) 
{
    int port = msg.canPort;
    if (port != CanPort_1 && port != CanPort_2) return false;

    std::vector<uint8_t> buf;
    buf.push_back(0xB0 | (msg.canPort & 0x0F));
    buf.push_back(((msg.data.size() & 0x0F) << 4) | (msg.canIdType));

    if (msg.canIdType == CanId_extended) 
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

    {
        std::lock_guard<std::mutex> lock(tx_queues_[port].mtx);
        tx_queues_[port].queue.push_back(std::move(buf));
    }

    tx_cv_.notify_one();
    framesSent_++;
    return true;
}

ssize_t CanUsbDevice::writeAll(const uint8_t* data, size_t len) 
{
    size_t off = 0;
    while (off < len) 
    {
        ssize_t n = ::write(fd_, data + off, len - off);
        if (n > 0) { off += n; continue; }
        if (n < 0 && (errno == EINTR)) continue;
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) 
        {
            std::this_thread::sleep_for(std::chrono::microseconds(100));
            continue;
        }
        return -1;
    }
    return off;
}

void CanUsbDevice::txThreadFunc(int interval_us) 
{
    while (runningTx_) 
    {
        auto loop_start = std::chrono::steady_clock::now();
        
        // 一次发送所有积压报文
        for (int port = CanPort_1; port <= CanPort_2; ++port) 
        {
            std::vector<std::vector<uint8_t>> bufs_to_send;
            {
                std::lock_guard<std::mutex> lock(tx_queues_[port].mtx);
                bufs_to_send.swap(tx_queues_[port].queue);
            }

            for (auto& buf : bufs_to_send) 
            {
                writeAll(buf.data(), buf.size());
            }
        }

        // 精确时间控制
        auto elapsed = std::chrono::steady_clock::now() - loop_start;
        if (elapsed < std::chrono::microseconds(interval_us)) 
        {
            std::this_thread::sleep_for(std::chrono::microseconds(interval_us) - elapsed);
        }
    }
}

void CanUsbDevice::startReceiveThread() 
{
    runningRx_ = true;
    recvThread_ = std::thread(&CanUsbDevice::receiveLoop, this);
}

void CanUsbDevice::stopReceiveThread() 
{
    runningRx_ = false;
    if (recvThread_.joinable()) {
        recvThread_.join();
    }
}

void CanUsbDevice::receiveLoop() 
{
    RingBuffer<16384> rb;
    uint8_t temp[512];

    while (runningRx_) {
        ssize_t n = ::read(fd_, temp, sizeof(temp));
        if (n > 0) {
            rb.push(temp, static_cast<size_t>(n));

            CanMessage msg;
            while (tryParseOneFrame(rb, msg)) {
                if (receiveCallback_) {
                    receiveCallback_(this, msg);
                    framesReceived_++;
                }
            }
        } else if (n < 0) {
            if (errno == EAGAIN || errno == EINTR) {
                continue;
            }
            {
                std::lock_guard<std::mutex> lock(coutMutex_);
                std::cerr << "Read error: " << strerror(errno) << std::endl;
            }
            break;
        }
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
    uint8_t ctrl = buffer[start + 1];       // 0xB1 / 0xB2
    uint8_t flags = buffer[start + 2];      // dlc | idType
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

    msg.canPort = ctrl & 0x0F;
    msg.canIdType = isExt ?  CanId_extended : CanId_classic;
    msg.id = 0;
    if (msg.canIdType == CanId_extended) 
    {
        msg.id = (buffer[start + 3] << 24) | (buffer[start + 4] << 16)| (buffer[start + 5] << 8) | buffer[start + 6];
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
// 使用环形缓冲解包
bool CanUsbDevice::tryParseOneFrame(RingBuffer<16384>& rb, CanMessage& msg) 
{
    if (rb.size() < 4) return false;

    auto it = rb.findFirst(0xA5);
    if (!it) {
        rb.pop(rb.size()); // 没找到帧头，丢弃全部
        return false;
    }

    size_t start = *it;
    if (rb.size() - start < 4) return false;

    uint8_t ctrl  = rb.get(start + 1);
    uint8_t flags = rb.get(start + 2);
    bool isExt    = (flags & 0x10);
    uint8_t dlc   = flags & 0x0F;

    size_t idLen   = isExt ? 4 : 1;
    size_t totalLen = 1 + 1 + 1 + idLen + dlc + 1; // A5+ctrl+flags+id+data+5A

    if (rb.size() - start < totalLen) return false;

    if (rb.get(start + totalLen - 1) != 0x5A) {
        rb.pop(start + 1); // 错帧，丢掉到下一个候选起点
        return false;
    }

    msg.canPort   = ctrl & 0x0F;
    msg.canIdType = isExt ? CanId_extended : CanId_classic;
    msg.id = 0;

    if (isExt) {
        msg.id = (rb.get(start + 3) << 24) |
                 (rb.get(start + 4) << 16) |
                 (rb.get(start + 5) << 8)  |
                  rb.get(start + 6);
    } else {
        msg.id = rb.get(start + 3);
    }

    size_t dataStart = start + 3 + idLen;
    msg.data.clear();
    msg.data.reserve(dlc);
    for (size_t i = 0; i < dlc; ++i) {
        msg.data.push_back(rb.get(dataStart + i));
    }

    rb.pop(start + totalLen); // 消费掉整帧
    return true;
}