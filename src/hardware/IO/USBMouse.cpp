#include "USBMouse.h"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>

#include "logging.h"


USBMouse::USBMouse(const std::string &devicePath) {
#ifndef NOROBOT
    fd_ = open(devicePath.c_str(), O_RDONLY | O_NONBLOCK);
    if (fd_ < 0) {
        spdlog::warn("USBMouse: could not open {} ({}) — no local mouse source available",
                     devicePath, strerror(errno));
    } else {
        spdlog::debug("USBMouse: opened {}", devicePath);
    }
#endif
}

USBMouse::~USBMouse() {
    if (fd_ >= 0) close(fd_);
}

void USBMouse::updateInput() {
    if (fd_ < 0) return;

    pressEvent_ = false;

    // Drain all pending packets this cycle so a burst of motion isn't left for next time.
    unsigned char packet[3];
    while (read(fd_, packet, sizeof(packet)) == (ssize_t)sizeof(packet)) {
        bool left = packet[0] & 0x01;
        // bytes 1/2 are already plain signed 8-bit relative deltas on this interface.
        int8_t dx = static_cast<int8_t>(packet[1]);
        int8_t dy = static_cast<int8_t>(packet[2]);

        position_.x() += dx * scale;
        position_.y() += -dy * scale;  // device Y convention is inverted relative to "up"

        if (left && !buttonDown_) pressEvent_ = true;  // one-shot on press, not release
        buttonDown_ = left;
    }
}
