/**
 * \file RotaryEncoder.h
 * \brief KY-040 rotary encoder with push button, read via BeagleBone GPIO (iobb).
 *
 * Wiring (BeagleBone header pins):
 *   VCC → P9_3 (3.3 V)   GND → P9_1
 *   S1 (CLK) → P8_11  (GPIO_45)
 *   S2 (DT)  → P8_12  (GPIO_44)
 *   Key (SW) → P8_15  (GPIO1_47)
 *
 * Usage — call updateInput() every control cycle (via Robot::updateRobot).
 *   getRotation()   → -1 (CCW), 0 (no change), +1 (CW)  — valid for one cycle
 *   isPressed()     → true for one cycle on short-press release
 *   isLongPressed() → true for one cycle when held ≥ longPressThreshold seconds
 *
 * Button debouncing: the raw GPIO signal must read the same state for
 * debounceCount consecutive cycles before the debounced state updates.
 * At a 1 kHz control loop, the default value of 10 gives a 10 ms debounce
 * window — sufficient to eliminate typical contact bounce.
 *
 * In NOROBOT builds all outputs are zero / false.
 */

#ifndef ROTARYENCODER_H
#define ROTARYENCODER_H

#include "InputDevice.h"
#include <chrono>

class RotaryEncoder : public InputDevice {
   public:
    RotaryEncoder(char clk_port, char clk_pin,
                  char dt_port,  char dt_pin,
                  char sw_port,  char sw_pin);
    ~RotaryEncoder();

    void updateInput() override;

    int  getRotation()   const { return rotation; }
    bool isPressed()     const { return pressEvent; }
    bool isLongPressed() const { return longPressEvent; }

    double longPressThreshold = 2.0;  //!< Seconds held before long-press fires
    int    debounceCount      = 10;   //!< Consecutive stable samples needed (×cycle time)

   private:
    char clk_port, clk_pin;
    char dt_port,  dt_pin;
    char sw_port,  sw_pin;

    int  rotation       = 0;
    bool pressEvent     = false;
    bool longPressEvent = false;

    char lastClk = 1;   // CLK is pulled high when idle

    // Button debounce state
    bool rawLast        = false;
    int  debounceTimer  = 0;
    bool debouncedState = false;   // stable debounced button state (true = pressed)

    // Long/short press tracking (uses debounced signal)
    bool buttonDown = false;
    bool longFired  = false;
    std::chrono::steady_clock::time_point buttonDownTime;
};

#endif // ROTARYENCODER_H
