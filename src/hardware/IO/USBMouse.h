/**
 * \file USBMouse.h
 * \brief Reads a USB mouse on Linux via the legacy /dev/input/mouseN relative-motion
 * protocol (3-byte packets: button-state byte + signed dx + signed dy).
 *
 * The mouse reports relative motion only — there is no absolute position. This class
 * integrates the deltas into a running 2D position in metres, with an arbitrary origin
 * (wherever the mouse was at construction). Callers needing a reference relative to some
 * other origin (e.g. UniControllerState's X_init) capture their own baseline from
 * getPosition() and subtract it — the same pattern already used for the GUI mouse command.
 *
 * Node path and permissions: /dev/input/mouseN requires read access (root, or membership
 * of the group that owns the device node — commonly "input"); verify the actual node with
 * `ls /dev/input/` or `cat /proc/bus/input/devices` if mouse0 isn't correct on a given unit.
 */

#ifndef USBMOUSE_H
#define USBMOUSE_H

#include <Eigen/Dense>
#include <string>

#include "InputDevice.h"


/**
 * \ingroup IO
 * \brief Class inheriting InputDevice to handle a USB mouse via /dev/input/mouseN.
 *
 * Opens gracefully: if no mouse is present at construction, isConnected() returns false
 * rather than throwing, so callers can fall back to another reference source.
 */
class USBMouse : public InputDevice {
   public:
    explicit USBMouse(const std::string &devicePath = "/dev/input/mouse0");
    ~USBMouse();

    void updateInput() override;

    /** False if the device file could not be opened (no mouse present) — callers should
     *  treat this source as unavailable and fall back to another reference. */
    bool isConnected() const { return fd_ >= 0; }

    /** Integrated position [m], arbitrary origin (position at construction). */
    const Eigen::Vector2d &getPosition() const { return position_; }

    /** True for one cycle on left-button press (one-shot, like RotaryEncoder::isPressed()). */
    bool isPressed() const { return pressEvent_; }

    double scale = 0.0005;  //!< TODO: tune — metres per relative-motion count

   private:
    int fd_ = -1;
    Eigen::Vector2d position_ = Eigen::Vector2d::Zero();
    bool buttonDown_ = false;
    bool pressEvent_ = false;
};

#endif  // USBMOUSE_H
