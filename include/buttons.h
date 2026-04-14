#pragma once
#include <Arduino.h>

enum class ButtonId : uint8_t
{
    STOP = 0,

    MODE_MANUAL,
    MODE_AUTO,
    MODE_ANCHOR,

    THRUST_UP,
    THRUST_DOWN,

    STEER_LEFT,
    STEER_RIGHT,

    COUNT
};

constexpr uint32_t buttonBit(ButtonId id)
{
    return (1UL << static_cast<uint8_t>(id));
}

inline bool isButtonPressed(uint32_t mask, ButtonId id)
{
    return (mask & buttonBit(id)) != 0;
}

namespace ButtonTiming
{
    static constexpr uint32_t DEBOUNCE_MS   = 30;
    static constexpr uint32_t LONG_PRESS_MS = 1000;
}

struct ButtonOutput
{
    bool stopRequested = false;

    bool requestManual = false;
    bool requestAuto   = false;
    bool requestAnchor = false;

    bool thrustUpHeld   = false;
    bool thrustDownHeld = false;
    bool steerLeftHeld  = false;
    bool steerRightHeld = false;
};

class ButtonManager
{
public:
    void begin();
    ButtonOutput update(uint32_t rawMask, uint32_t nowMs);
    uint32_t stableMask() const { return _stableMask; }

private:
    void handleLongPress(ButtonId id, uint32_t nowMs, bool& triggerOut);

private:
    uint32_t _rawMask = 0;
    uint32_t _lastRawMask = 0;
    uint32_t _stableMask = 0;
    uint32_t _lastRawChangeMs = 0;

    uint32_t _pressStartMs[static_cast<uint8_t>(ButtonId::COUNT)] = {};
    bool _longReported[static_cast<uint8_t>(ButtonId::COUNT)] = {};
};