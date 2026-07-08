/**
 * \file LCD1602.cpp
 * \brief HD44780 16×2 LCD in 4-bit mode via PCF8574 I2C backpack.
 *
 * Public API calls (clear, setCursor, print) write only to an in-memory buffer
 * and return immediately.  A background thread started by init() compares the
 * buffer against what is currently on screen and flushes changed rows to
 * hardware at ~10 Hz, keeping the 1 kHz control loop free of I2C latency.
 *
 * All hardware I2C writes are guarded with #ifndef NOROBOT so desktop builds
 * compile and run without any real device present.
 */

#include "LCD1602.h"

#ifndef NOROBOT
#include <unistd.h>
#endif

#include <chrono>
#include <cstring>

// ─── Construction / destruction ───────────────────────────────────────────────

LCD1602::LCD1602(unsigned char addr, int bus) {
    // Initialise both buffers to spaces so the first flush writes a clean screen
    for (int r = 0; r < 2; r++) {
        memset(buffer_[r],    ' ', 16);  buffer_[r][16]    = '\0';
        memset(displayed_[r], '\0', 17); // force mismatch → full write on first flush
    }
#ifndef NOROBOT
    setDeviceAddress(addr);
    setBusId(bus);
    initDevice();
#endif
}

LCD1602::~LCD1602() {
    // Signal thread to stop and wait for it to finish its current flush
    running_ = false;
    if (lcdThread_.joinable())
        lcdThread_.join();
}


// ─── Low-level PCF8574 output (hardware layer — thread only) ─────────────────

void LCD1602::sendNibble(uint8_t nibble, uint8_t rs) {
#ifndef NOROBOT
    uint8_t bl   = backlightOn ? BL : 0;
    uint8_t data = ((nibble & 0x0F) << 4) | bl | rs;

    setRegisterAddress(data | E);
    writeToDevice(1);
    usleep(1);

    setRegisterAddress(data);
    writeToDevice(1);
    usleep(50);
#endif
}

void LCD1602::sendByte(uint8_t byte, uint8_t rs) {
    sendNibble(byte >> 4,   rs);
    sendNibble(byte & 0x0F, rs);
}


// ─── Background flush thread ──────────────────────────────────────────────────

void LCD1602::threadFunc() {
    while (running_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));  // 10 Hz

        std::lock_guard<std::mutex> lock(bufMutex_);
        if (!dirty_) continue;

        // Write only rows that changed to minimise I2C traffic
        static const uint8_t rowAddr[2] = {0x00, 0x40};
        for (int row = 0; row < 2; row++) {
            if (memcmp(buffer_[row], displayed_[row], 16) == 0) continue;
#ifndef NOROBOT
            sendCmd(0x80 | rowAddr[row]);
            for (int col = 0; col < 16; col++)
                sendData((uint8_t)buffer_[row][col]);
#endif
            memcpy(displayed_[row], buffer_[row], 16);
        }
        dirty_ = false;
    }
}


// ─── Initialisation ───────────────────────────────────────────────────────────

void LCD1602::init() {
#ifndef NOROBOT
    usleep(50000);   // >40 ms post power-on

    // Three 8-bit writes to bring the display to a known state
    sendNibble(0x3, 0);  usleep(4500);
    sendNibble(0x3, 0);  usleep(4500);
    sendNibble(0x3, 0);  usleep(150);

    // Switch to 4-bit mode
    sendNibble(0x2, 0);  usleep(100);

    sendCmd(0x28);   // 4-bit, 2-line, 5×8 font
    sendCmd(0x0C);   // Display on, cursor off, blink off
    sendCmd(0x06);   // Entry mode: increment, no shift
    sendCmd(0x01);   // Clear display
    usleep(2000);
#endif

    // Start background flush thread
    running_ = true;
    lcdThread_ = std::thread(&LCD1602::threadFunc, this);
}


// ─── Public buffer API ────────────────────────────────────────────────────────

void LCD1602::clear() {
    std::lock_guard<std::mutex> lock(bufMutex_);
    for (int r = 0; r < 2; r++) {
        memset(buffer_[r], ' ', 16);
        // Force full rewrite by invalidating the displayed snapshot
        memset(displayed_[r], '\0', 16);
    }
    dirty_ = true;
}

void LCD1602::setCursor(uint8_t col, uint8_t row) {
    // Virtual cursor — main thread only, no mutex needed
    curRow_ = row & 0x1;
    curCol_ = (col < 16) ? col : 15;
}

void LCD1602::printChar(char c) {
    std::lock_guard<std::mutex> lock(bufMutex_);
    if (curCol_ < 16) {
        buffer_[curRow_][curCol_++] = c;
        dirty_ = true;
    }
}

void LCD1602::print(const char *str) {
    std::lock_guard<std::mutex> lock(bufMutex_);
    while (*str && curCol_ < 16) {
        buffer_[curRow_][curCol_++] = *str++;
    }
    dirty_ = true;
}

void LCD1602::setBacklight(bool on) {
    std::lock_guard<std::mutex> lock(bufMutex_);
    backlightOn = on;
    // Force a flush so the backlight bit updates on the next write
    memset(displayed_, '\0', sizeof(displayed_));
    dirty_ = true;
}
