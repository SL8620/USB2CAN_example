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
#include <sys/epoll.h>
#include <sched.h>
#include <time.h>
#include <sys/ioctl.h>
#include <sys/uio.h>

using namespace can_usb_driver;

std::mutex CanUsbDevice::coutMutex_;

static std::string findDefaultDevice() 
{
    DIR* dir = opendir("/dev");
    if (!dir) {
        std::lock_guard<std::mutex> lock(CanUsbDevice::coutMutex_);
        std::cerr << "Failed to open /dev: " << strerror(errno) << std::endl;
        return "";
    }
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
    if (devicePath_.empty()) {
        std::lock_guard<std::mutex> lock(coutMutex_);
        std::cerr << "Error: Device path is empty" << std::endl;
        return false;
    }

    fd_ = ::open(devicePath_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) 
    {
        std::lock_guard<std::mutex> lock(coutMutex_);
        std::cerr << "Failed to open " << devicePath_ << ": " << strerror(errno) << std::endl;
        return false;
    }

    termios tty{};
    if (tcgetattr(fd_, &tty) != 0) 
    {
        std::lock_guard<std::mutex> lock(coutMutex_);
        std::cerr << "Failed to get termios: " << strerror(errno) << std::endl;
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    tty.c_iflag = 0;
    tty.c_oflag = 0;
    tty.c_cflag = CREAD | CLOCAL | CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_lflag = 0;

    cfsetospeed(&tty, B2000000);
    cfsetispeed(&tty, B2000000);
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 2;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) 
    {
        std::lock_guard<std::mutex> lock(coutMutex_);
        std::cerr << "Failed to set termios: " << strerror(errno) << std::endl;
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    tcflush(fd_, TCIOFLUSH);

    try {
        runningTx_ = true;
        runningRx_ = true;
        tx_thread_ = std::thread([this] {
            try {
                struct sched_param param = { .sched_priority = 20 };
                int ret = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
                if (ret != 0) {
                    std::lock_guard<std::mutex> lock(coutMutex_);
                    std::cerr << "Failed to set SCHED_FIFO for tx_thread: " << strerror(ret) << std::endl;
                }
                txThreadFunc(100);
            } catch (const std::exception& e) {
                std::lock_guard<std::mutex> lock(coutMutex_);
                std::cerr << "Exception in tx_thread: " << e.what() << std::endl;
            }
        });
    } catch (const std::exception& e) {
        std::lock_guard<std::mutex> lock(coutMutex_);
        std::cerr << "Failed to start tx_thread: " << e.what() << std::endl;
        ::close(fd_);
        fd_ = -1;
        return false;
    }
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
    while (off < len && runningTx_) 
    {
        int epfd = epoll_create1(0);
        if (epfd < 0) {
            std::lock_guard<std::mutex> lock(coutMutex_);
            std::cerr << "Failed to create epoll: " << strerror(errno) << std::endl;
            return -1;
        }
        struct epoll_event ev = { .events = EPOLLOUT, .data = { .fd = fd_ } };
        epoll_ctl(epfd, EPOLL_CTL_ADD, fd_, &ev);
        struct epoll_event events[1];
        int nfds = epoll_wait(epfd, events, 1, 1);
        ::close(epfd);

        if (nfds > 0 && (events[0].events & EPOLLOUT)) {
            ssize_t n = ::write(fd_, data + off, len - off);
            if (n > 0) {
                off += n;
                continue;
            }
            if (n < 0 && (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)) continue;
            std::lock_guard<std::mutex> lock(coutMutex_);
            std::cerr << "Write error: " << strerror(errno) << std::endl;
            return -1;
        }
    }
    return off;
}

void CanUsbDevice::txThreadFunc(int interval_us) 
{
    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);
    while (runningTx_) 
    {
        for (int port = CanPort_1; port <= CanPort_2; ++port) 
        {
            std::vector<std::vector<uint8_t>> bufs_to_send;
            {
                std::lock_guard<std::mutex> lock(tx_queues_[port].mtx);
                bufs_to_send.swap(tx_queues_[port].queue);
            }
            std::vector<struct iovec> iov;
            for (auto& buf : bufs_to_send) {
                iov.push_back({ .iov_base = buf.data(), .iov_len = buf.size() });
            }
            if (!iov.empty()) {
                writev(fd_, iov.data(), iov.size());
            }
        }
        next.tv_nsec += interval_us * 1000;
        if (next.tv_nsec >= 1000000000) {
            next.tv_sec += 1;
            next.tv_nsec -= 1000000000;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr);
    }
}

void CanUsbDevice::startReceiveThread() 
{
    runningRx_ = true;
    try {
        recvThread_ = std::thread([this] {
            try {
                struct sched_param param = { .sched_priority = 20 };
                int ret = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
                if (ret != 0) {
                    std::lock_guard<std::mutex> lock(coutMutex_);
                    std::cerr << "Failed to set SCHED_FIFO for recvThread: " << strerror(ret) << std::endl;
                }
                receiveLoop();
            } catch (const std::exception& e) {
                std::lock_guard<std::mutex> lock(coutMutex_);
                std::cerr << "Exception in recvThread: " << e.what() << std::endl;
            }
        });
    } catch (const std::exception& e) {
        std::lock_guard<std::mutex> lock(coutMutex_);
        std::cerr << "Failed to start recvThread: " << e.what() << std::endl;
        runningRx_ = false;
    }
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
    if (fd_ < 0) {
        std::lock_guard<std::mutex> lock(coutMutex_);
        std::cerr << "Error: Invalid file descriptor in receiveLoop" << std::endl;
        return;
    }

    RingBuffer<16384> rb;
    std::vector<uint8_t> temp(512);
    int epfd = epoll_create1(0);
    if (epfd < 0) {
        std::lock_guard<std::mutex> lock(coutMutex_);
        std::cerr << "Failed to create epoll: " << strerror(errno) << std::endl;
        return;
    }
    struct epoll_event ev = { .events = EPOLLIN, .data = { .fd = fd_ } };
    if (epoll_ctl(epfd, EPOLL_CTL_ADD, fd_, &ev) < 0) {
        std::lock_guard<std::mutex> lock(coutMutex_);
        std::cerr << "Failed to add fd to epoll: " << strerror(errno) << std::endl;
        ::close(epfd);
        return;
    }
    auto last_data_time = std::chrono::steady_clock::now();

    while (runningRx_) {
        struct epoll_event events[1];
        int nfds = epoll_wait(epfd, events, 1, 1);
        if (nfds < 0) {
            std::lock_guard<std::mutex> lock(coutMutex_);
            std::cerr << "epoll_wait failed: " << strerror(errno) << std::endl;
            break;
        }
        if (nfds > 0 && (events[0].events & EPOLLIN)) {
            int available = 0;
            if (ioctl(fd_, FIONREAD, &available) < 0) {
                std::lock_guard<std::mutex> lock(coutMutex_);
                std::cerr << "ioctl FIONREAD failed: " << strerror(errno) << std::endl;
                break;
            }
            if (available > static_cast<int>(temp.size())) {
                temp.resize(std::max<size_t>(available, 1024));
            }
            ssize_t n = ::read(fd_, temp.data(), temp.size());
            if (n > 0) {
                last_data_time = std::chrono::steady_clock::now();
                rb.push(temp.data(), n);
                CanMessage msg;
                while (tryParseOneFrame(rb, msg)) {
                    if (receiveCallback_) {
                        receiveCallback_(this, msg);
                        framesReceived_++;
                    }
                }
                if (rb.getDroppedCount() > 0) {
                    std::lock_guard<std::mutex> lock(coutMutex_);
                    std::cerr << "Dropped " << rb.getDroppedCount() << " bytes in RingBuffer" << std::endl;
                }
            } else if (n < 0 && errno != EAGAIN && errno != EINTR) {
                std::lock_guard<std::mutex> lock(coutMutex_);
                std::cerr << "Read error: " << strerror(errno) << std::endl;
                break;
            }
        }
        // if (std::chrono::steady_clock::now() - last_data_time > std::chrono::milliseconds(500)) {
        //     std::lock_guard<std::mutex> lock(coutMutex_);
        //     std::cerr << "No data for 500ms, reconnecting" << std::endl;
        //     close();
        //     if (!open()) {
        //         std::cerr << "Reconnect failed" << std::endl;
        //         break;
        //     }
        //     last_data_time = std::chrono::steady_clock::now();
        // }
    }
    ::close(epfd);
}

bool CanUsbDevice::tryParseOneFrame(RingBuffer<16384>& rb, CanMessage& msg) 
{
    if (rb.size() < 4) return false;

    auto it = rb.findFirst(0xA5);
    if (!it) {
        rb.pop(rb.size());
        framesInvalid_++;
        return false;
    }

    size_t start = *it;
    if (rb.size() - start < 4) return false;

    uint8_t ctrl = rb.get(start + 1);
    uint8_t flags = rb.get(start + 2);
    bool isExt = (flags & 0x10);
    uint8_t dlc = flags & 0x0F;

    size_t idLen = isExt ? 4 : 1;
    size_t totalLen = 1 + 1 + 1 + idLen + dlc + 1;

    if (rb.size() - start < totalLen) return false;

    if (rb.get(start + totalLen - 1) != 0x5A) {
        rb.pop(start + 1);
        framesInvalid_++;
        return false;
    }

    msg.canPort = ctrl & 0x0F;
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
    msg.data.resize(dlc);
    rb.copyData(dataStart, dlc, msg.data.data());

    rb.pop(start + totalLen);
    return true;
}