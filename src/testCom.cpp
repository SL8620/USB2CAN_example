#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <errno.h>

// int main() {
//     const char* device = "/dev/ttyACM0";
//     int fd = open(device, O_RDWR | O_NONBLOCK);
//     if (fd < 0) {
//         std::cerr << "open failed: " << strerror(errno) << std::endl;
//         return 1;
//     }

//     int flags = fcntl(fd, F_GETFL, 0);
//     fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);

//     std::cout << "Device opened successfully." << std::endl;
//     close(fd);
//     return 0;
// }
