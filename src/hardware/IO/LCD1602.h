/**
 * \file LCD1602.h
 * \brief HD44780 16×2 LCD driven in 4-bit mode via PCF8574 I2C backpack.
 *
 * PCF8574 bit mapping (standard backpack wiring):
 *   P0 = RS   (0=command, 1=data)
 *   P1 = RW   (always 0 — write only)
 *   P2 = E    (enable strobe, active high)
 *   P3 = BL   (backlight, 1=on)
 *   P4-P7 = D4-D7   (4-bit data bus, high nibble sent first)
 *
 * Typical I2C address: 0x27 (A0=A1=A2=1) or 0x3F (A0=A1=A2=0).
 * Wiring: VCC → P9_7 (5V), GND → P9_2, SDA → P9_20, SCL → P9_19 (/dev/i2c-2 on this BBB).
 * Do NOT use P9_24/P9_26 — those are shared with the CAN1 bus.
 *
 * Threading model
 * ───────────────
 * The public API (setCursor, print, clear) writes only to an internal
 * 2×16 character buffer and returns immediately — no I2C blocking.
 * A background thread started by init() flushes changed rows to hardware
 * at ~10 Hz.  This keeps the control loop free of I2C latency (~5 ms per
 * full display write at 100 kHz).
 *
 * Usage:
 *   LCD1602 lcd;
 *   lcd.init();           // starts background thread
 *   lcd.clear();
 *   lcd.setCursor(0, 0);
 *   lcd.print("Hello!");  // returns immediately; thread writes to screen
 *
 * In NOROBOT builds all I2C writes are omitted; the buffer and thread still
 * work so state logic can run unmodified on a desktop.
 */

#ifndef LCD1602_H
#define LCD1602_H

#include "I2CDevice.h"
#include <atomic>
#include <cstring>
#include <mutex>
#include <stdint.h>
#include <thread>

class LCD1602 : public I2CDevice {
   public:
    LCD1602(unsigned char addr = 0x27, int bus = 2);
    ~LCD1602();

    /** Hardware init + start background flush thread. Call once after construction. */
    void init();

    /** Fill both rows with spaces and mark dirty. */
    void clear();

    /** Set write position for subsequent print() calls.  col ∈ [0,15], row ∈ [0,1]. */
    void setCursor(uint8_t col, uint8_t row);

    /** Copy str into the display buffer at the current cursor position.
     *  Returns immediately — the thread sends it to hardware within 100 ms. */
    void print(const char *str);

    /** Write a single character into the buffer. */
    void printChar(char c);

    /** Turn backlight on or off (takes effect at next thread flush). */
    void setBacklight(bool on);

   private:
    bool backlightOn = true;

    static constexpr uint8_t RS = 0x01;   //!< P0 — register select
    static constexpr uint8_t E  = 0x04;   //!< P2 — enable strobe
    static constexpr uint8_t BL = 0x08;   //!< P3 — backlight

    // ── Double-buffer state ───────────────────────────────────────────────────
    std::mutex        bufMutex_;
    std::atomic<bool> running_{false};
    bool              dirty_     = false;
    char              buffer_[2][17];      //!< Desired display content
    char              displayed_[2][17];   //!< Last content written to hardware

    // Virtual cursor (main-thread only — no mutex needed)
    uint8_t curRow_ = 0;
    uint8_t curCol_ = 0;

    // ── Background thread ─────────────────────────────────────────────────────
    std::thread lcdThread_;
    void threadFunc();

    // ── Hardware layer (called from thread only) ──────────────────────────────
    void sendNibble(uint8_t nibble, uint8_t rs);
    void sendByte(uint8_t byte, uint8_t rs);
    void sendCmd(uint8_t cmd)   { sendByte(cmd,  0); }
    void sendData(uint8_t data) { sendByte(data, RS); }

    void setDeviceAddress(unsigned char addr) override { DeviceAddress = addr; }
    void setBusId(int bus)                    override { BusId = bus; }
};

#endif  // LCD1602_H
