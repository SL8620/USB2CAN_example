#include "can_usb_device.hpp"
#include <chrono>
#include <thread>
#include <iostream>
#include <atomic>
#include <map>
#include <iomanip>

using namespace can_usb_driver;
using namespace std::chrono_literals;

class PrecisionTimer {
    using Clock = std::chrono::high_resolution_clock;
    Clock::time_point next_;
    std::chrono::microseconds interval_;
    
public:
    PrecisionTimer(double freq_hz) {
        setFrequency(freq_hz);
        next_ = Clock::now();
    }

    void setFrequency(double freq_hz) {
        interval_ = std::chrono::microseconds(static_cast<int>(1e6 / freq_hz));
    }

    void waitNext() {
        next_ += interval_;
        std::this_thread::sleep_until(next_);
    }
};

struct TestConfig {
    double target_freq_hz = 1000.0;
    int test_duration_sec = 600;
    int frames_per_cycle = 3;
};

struct TestStats {
    std::atomic<uint64_t> sent{0};
    std::atomic<uint64_t> valid_received{0};  // 只统计有效ID
    std::map<uint8_t, uint64_t> received_ids; // 只记录1,2,3
    std::mutex map_mutex;

    void recordValidReceived(uint8_t id) {
        if (id == 1 || id == 2 || id == 3) {  // 只处理有效ID
            std::lock_guard<std::mutex> lock(map_mutex);
            received_ids[id]++;
            valid_received++;
        }
    }
};

int main() 
{
    CanUsbDevice can_dev;
    TestStats stats;
    TestConfig config;

    if (!can_dev.open()) {
        std::cerr << "Failed to open CAN device!" << std::endl;
        return 1;
    }

    // 设置接收回调（只统计ID为1,2,3的报文）
    can_dev.setReceiveCallback([&stats](const CanUsbDevice* dev, const CanMessage& msg) {
        if (!msg.data.empty()) {
            uint8_t id = msg.data[0];
            stats.recordValidReceived(id);  // 只记录有效ID
        }
    });

    can_dev.startReceiveThread();

    can_usb_driver::CanMessage msg;
    msg.canPort = CanPort_1;
    msg.canIdType = CanId_classic;
    msg.data = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    msg.id = 1;
    can_dev.sendCanMessage(msg);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    msg.id = 2;
    can_dev.sendCanMessage(msg);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    msg.id = 3;
    can_dev.sendCanMessage(msg);
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // 准备测试报文（ID对应1,2,3）
    const std::vector<uint8_t> fixed_payload = {0x7F, 0xFF, 0x7F, 0xF0, 0x00, 0x00, 0x07, 0xFF};
    const std::vector<CanMessage> test_msgs = {
        {CanPort_1, CanId_classic, 1, fixed_payload},  // ID 1
        {CanPort_1, CanId_classic, 2, fixed_payload},  // ID 2
        {CanPort_1, CanId_classic, 3, fixed_payload}   // ID 3
    };

    std::cout << "=== CAN Test ===" << std::endl;
    std::cout << "Target: " << config.target_freq_hz << " Hz\n";
    std::cout << "Duration: " << config.test_duration_sec << " sec\n";
    std::cout << "Monitoring valid IDs: 1, 2, 3\n";

    PrecisionTimer timer(config.target_freq_hz);
    auto start = std::chrono::steady_clock::now();

    // 实时监控线程
    std::atomic<bool> monitoring{true};
    std::thread monitor([&]{
        while (monitoring) {
            std::this_thread::sleep_for(1s);
            auto elapsed = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - start).count();
            double actual_freq = stats.sent / (elapsed * config.frames_per_cycle);
            std::cout << "\rActual: " << std::fixed << std::setprecision(2) 
                      << actual_freq << " Hz | "
                      << "Valid RX: " << stats.valid_received.load()
                      << "\rId 1: "<<stats.received_ids[1]<< "| Id 2: "<<stats.received_ids[2]<<"| Id 3: "<<stats.received_ids[3]
                      << std::flush;
        }
    });

    // 主发送循环
    while (std::chrono::steady_clock::now() - start < std::chrono::seconds(config.test_duration_sec)) {
        for (const auto& msg : test_msgs) {
            if (can_dev.sendCanMessage(msg)) {
                stats.sent++;
                //std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
        timer.waitNext();
    }

    monitoring = false;
    monitor.join();
    can_dev.stopReceiveThread();
    can_dev.close();

    // 打印结果
    std::cout << "\n\n=== Results ===" << std::endl;
    std::cout << "Total sent: " << stats.sent << std::endl;
    std::cout << "Valid received: " << stats.valid_received << std::endl;
    std::cout << "Success rate: " 
              << std::setprecision(2) << std::fixed
              << (stats.sent > 0 ? 100.0 * stats.valid_received / stats.sent : 0) 
              << "%" << std::endl;

    std::cout << "\nValid ID Distribution:" << std::endl;
    for (const auto& [id, count] : stats.received_ids) {
        std::cout << "ID " << static_cast<int>(id) << ": " << count << " frames\n";
    }

    return 0;
}