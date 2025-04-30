/*** 
 * @Author: SL8620
 * @Date: 2025-04-30 16:09:37
 * @LastEditTime: 2025-04-30 16:23:03
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
 
 USB2CAN::USB2CAN(const std::string& port, int baud)
     : portName(port), baudrate(baud), fd(-1), running(false) {}
 
 USB2CAN::~USB2CAN() {
     closeUSB2CAN();
 }
 
 bool USB2CAN::openUSB2CAN() {
     fd = open(portName.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
     if (fd < 0) {
         std::cerr << "Failed to open " << portName << std::endl;
         return false;
     }
 
     if (!configurePort()) {
         close(fd);
         fd = -1;
         return false;
     }
 
     running = true;
     readerThread = std::thread(&USB2CAN::readThreadFunc, this);
     return true;
 }
 
 void USB2CAN::closeUSB2CAN() {
     running = false;
     if (readerThread.joinable()) readerThread.join();
     if (fd >= 0) {
         close(fd);
         fd = -1;
     }
 }
 
 bool USB2CAN::sendUSB2CAN(const CanFrame& frame) {
     if (fd < 0) return false;
     std::string str = frameToString(frame) + "\n";
     ssize_t written = write(fd, str.c_str(), str.size());
     return written == (ssize_t)str.size();
 }
 
 bool USB2CAN::readUSB2CAN(CanFrame& frame) {
     std::lock_guard<std::mutex> lock(queueMutex);
     if (recvQueue.empty()) return false;
     frame = recvQueue.front();
     recvQueue.pop();
     return true;
 }
 
 void USB2CAN::readThreadFunc() {
     char buf[256];
     std::string line;
 
     while (running) {
         ssize_t n = read(fd, buf, sizeof(buf));
         if (n > 0) {
             for (ssize_t i = 0; i < n; ++i) {
                 if (buf[i] == '\n') {
                     CanFrame frame;
                     if (parseFrame(line, frame)) {
                         std::lock_guard<std::mutex> lock(queueMutex);
                         recvQueue.push(frame);
                     }
                     line.clear();
                 } else if (buf[i] >= 32 && buf[i] <= 126) {
                     line += buf[i];
                 }
             }
         } else {
             usleep(1000); // avoid busy loop
         }
     }
 }
 
 bool USB2CAN::configurePort() {
     struct termios tty;
     if (tcgetattr(fd, &tty) != 0) {
         std::cerr << "tcgetattr failed\n";
         return false;
     }
 
     cfsetospeed(&tty, B115200);  // 固定为 115200，这里也可根据 baudrate 改为动态设置
     cfsetispeed(&tty, B115200);
 
     tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
     tty.c_iflag &= ~(IXON | IXOFF | IXANY);
     tty.c_lflag = 0; // No signaling chars, no echo
     tty.c_oflag = 0;
     tty.c_cc[VMIN] = 1;
     tty.c_cc[VTIME] = 1;
 
     tty.c_cflag |= (CLOCAL | CREAD);
     tty.c_cflag &= ~(PARENB | PARODD);
     tty.c_cflag &= ~CSTOPB;
     tty.c_cflag &= ~CRTSCTS;
 
     if (tcsetattr(fd, TCSANOW, &tty) != 0) {
         std::cerr << "tcsetattr failed\n";
         return false;
     }
 
     return true;
 }
 
 bool USB2CAN::parseFrame(const std::string& line, CanFrame& frame) {
     std::istringstream iss(line);
     std::string token;
     std::vector<uint8_t> bytes;
 
     while (iss >> token) {
         try {
             uint8_t byte = static_cast<uint8_t>(std::stoul(token, nullptr, 16));
             bytes.push_back(byte);
         } catch (...) {
             return false;
         }
     }
 
     if (bytes.size() < 10) return false;
 
     frame.id  = (bytes[0] << 24) | (bytes[1] << 16) | (bytes[2] << 8) | bytes[3];
     frame.dlc = bytes[4];
     if (frame.dlc > 8 || bytes.size() < 5 + frame.dlc + 2) return false;
 
     for (uint8_t i = 0; i < frame.dlc; ++i) {
         frame.data[i] = bytes[5 + i];
     }
 
     frame.crc = (bytes[5 + frame.dlc] << 8) | bytes[5 + frame.dlc + 1];
     uint16_t crcCalc = computeCRC(bytes.data(), 5 + frame.dlc);
     return crcCalc == frame.crc;
 }
 
 std::string USB2CAN::frameToString(const CanFrame& frame) {
     std::ostringstream oss;
     oss << std::hex << std::setfill('0');
     oss << std::setw(2) << ((frame.id >> 24) & 0xFF) << " "
         << std::setw(2) << ((frame.id >> 16) & 0xFF) << " "
         << std::setw(2) << ((frame.id >> 8) & 0xFF) << " "
         << std::setw(2) << (frame.id & 0xFF) << " "
         << std::setw(2) << (int)frame.dlc << " ";
 
     for (uint8_t i = 0; i < frame.dlc; ++i)
         oss << std::setw(2) << (int)frame.data[i] << " ";
 
     uint16_t crc = computeCRC((const uint8_t*)frameToBytes(frame).data(), 5 + frame.dlc);
     oss << std::setw(2) << ((crc >> 8) & 0xFF) << " "
         << std::setw(2) << (crc & 0xFF);
 
     return oss.str();
 }
 
 std::vector<uint8_t> USB2CAN::frameToBytes(const CanFrame& frame) {
     std::vector<uint8_t> bytes;
     bytes.push_back((frame.id >> 24) & 0xFF);
     bytes.push_back((frame.id >> 16) & 0xFF);
     bytes.push_back((frame.id >> 8) & 0xFF);
     bytes.push_back(frame.id & 0xFF);
     bytes.push_back(frame.dlc);
     for (uint8_t i = 0; i < frame.dlc; ++i)
         bytes.push_back(frame.data[i]);
     return bytes;
 }
 
 uint16_t USB2CAN::computeCRC(const uint8_t* data, size_t len) {
     uint16_t crc = 0xFFFF;
     for (size_t i = 0; i < len; ++i) {
         crc ^= data[i];
         for (int j = 0; j < 8; ++j)
             crc = (crc >> 1) ^ (crc & 1 ? 0xA001 : 0);
     }
     return crc;
 }
 
 