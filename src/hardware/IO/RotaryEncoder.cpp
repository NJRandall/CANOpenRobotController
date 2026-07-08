/**
 * \file RotaryEncoder.cpp
 * \brief KY-040 rotary encoder with push button, read via BeagleBone GPIO (iobb).
 *
 * In NOROBOT builds all methods are no-ops — outputs remain zero / false.
 */

#include "RotaryEncoder.h"

#ifndef NOROBOT
#include "iobb.h"
#endif

using namespace std::chrono;

RotaryEncoder::RotaryEncoder(char clk_port_, char clk_pin_,
                             char dt_port_,  char dt_pin_,
                             char sw_port_,  char sw_pin_)
    : clk_port(clk_port_), clk_pin(clk_pin_),
      dt_port(dt_port_),   dt_pin(dt_pin_),
      sw_port(sw_port_),   sw_pin(sw_pin_) {
#ifndef NOROBOT
    iolib_init();
    iolib_setdir(clk_port, clk_pin, BBBIO_DIR_IN);
    iolib_setdir(dt_port,  dt_pin,  BBBIO_DIR_IN);
    iolib_setdir(sw_port,  sw_pin,  BBBIO_DIR_IN);
#endif
}

RotaryEncoder::~RotaryEncoder() {}

void RotaryEncoder::updateInput() {
    rotation       = 0;
    pressEvent     = false;
    longPressEvent = false;

#ifndef NOROBOT
    // ── Quadrature decode: trigger on CLK falling edge ────────────────────────
    char clk = is_high(clk_port, clk_pin) ? 1 : 0;
    if (lastClk == 1 && clk == 0) {
        // DT state at the moment CLK falls encodes direction
        rotation = is_high(dt_port, dt_pin) ? -1 : 1;
    }
    lastClk = clk;

    // ── Button debounce ───────────────────────────────────────────────────────
    bool raw = is_low(sw_port, sw_pin);
    if (raw == rawLast) {
        if (debounceTimer < debounceCount) {
            debounceTimer++;
            if (debounceTimer == debounceCount)
                debouncedState = raw;
        }
    } else {
        rawLast       = raw;
        debounceTimer = 0;
    }

    // ── Long-press / short-press detection (debounced signal) ─────────────────
    bool pressed = debouncedState;
    auto now     = steady_clock::now();
    if (pressed && !buttonDown) {
        buttonDown     = true;
        longFired      = false;
        buttonDownTime = now;
    } else if (pressed && buttonDown && !longFired) {
        double held = duration_cast<microseconds>(now - buttonDownTime).count() / 1e6;
        if (held >= longPressThreshold) {
            longPressEvent = true;
            longFired      = true;
        }
    } else if (!pressed && buttonDown) {
        if (!longFired)
            pressEvent = true;
        buttonDown = false;
    }
#endif
}
