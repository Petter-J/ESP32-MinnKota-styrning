#include "buttons.h"

void ButtonManager::begin()
{
    _rawMask = 0;
    _lastRawMask = 0;
    _stableMask = 0;
    _lastRawChangeMs = 0;

    for (uint8_t i = 0; i < static_cast<uint8_t>(ButtonId::COUNT); ++i)
    {
        _pressStartMs[i] = 0;
        _longReported[i] = false;
    }
    _calComboStartMs = 0;
    _calComboReported = false;
}

ButtonOutput ButtonManager::update(uint32_t rawMask, uint32_t nowMs)
{
    ButtonOutput out{};

    if (rawMask != _lastRawMask)
    {
        _lastRawMask = rawMask;
        _lastRawChangeMs = nowMs;
    }

    if ((nowMs - _lastRawChangeMs) >= ButtonTiming::DEBOUNCE_MS)
    {
        _rawMask = _lastRawMask;
    }

    const uint32_t prevMask = _stableMask;
    _stableMask = _rawMask;

    const uint32_t changed = prevMask ^ _stableMask;

    for (uint8_t i = 0; i < static_cast<uint8_t>(ButtonId::COUNT); ++i)
    {
        const ButtonId id = static_cast<ButtonId>(i);
        const uint32_t bit = buttonBit(id);

        const bool wasPressed = (prevMask & bit) != 0;
        const bool isPressed  = (_stableMask & bit) != 0;

        if ((changed & bit) == 0)
            continue;

        if (!wasPressed && isPressed)
        {
            _pressStartMs[i] = nowMs;
            _longReported[i] = false;
        }
        else if (wasPressed && !isPressed)
        {
            _pressStartMs[i] = 0;
            _longReported[i] = false;
        }
    }

    if (isButtonPressed(_stableMask, ButtonId::STOP))
    {
        out.stopRequested = true;
    }

    const bool calCombo =
        isButtonPressed(_stableMask, ButtonId::STEER_LEFT) &&
        isButtonPressed(_stableMask, ButtonId::STEER_RIGHT);

    if (calCombo)
    {
        if (_calComboStartMs == 0)
        {
            _calComboStartMs = nowMs;
        }

        if (!_calComboReported &&
            (nowMs - _calComboStartMs) >= 3000)
        {
            _calComboReported = true;
            out.requestCalibration = true;
        }
    }
    else
    {
        _calComboStartMs = 0;
        _calComboReported = false;
    }

    handleLongPress(ButtonId::MODE_MANUAL, nowMs, out.requestManual);
    handleLongPress(ButtonId::MODE_AUTO,   nowMs, out.requestAuto);
    handleLongPress(ButtonId::MODE_ANCHOR, nowMs, out.requestAnchor);

    out.thrustUpHeld   = isButtonPressed(_stableMask, ButtonId::THRUST_UP);
    out.thrustDownHeld = isButtonPressed(_stableMask, ButtonId::THRUST_DOWN);
    out.steerLeftHeld  = isButtonPressed(_stableMask, ButtonId::STEER_LEFT);
    out.steerRightHeld = isButtonPressed(_stableMask, ButtonId::STEER_RIGHT);

    out.anchorHeld = isButtonPressed(_stableMask, ButtonId::MODE_ANCHOR);

    return out;
}

void ButtonManager::handleLongPress(ButtonId id, uint32_t nowMs, bool& triggerOut)
{
    const uint8_t idx = static_cast<uint8_t>(id);

    if (!isButtonPressed(_stableMask, id))
        return;

    if (_longReported[idx])
        return;

    if (_pressStartMs[idx] == 0)
        return;

    if ((nowMs - _pressStartMs[idx]) >= ButtonTiming::LONG_PRESS_MS)
    {
        _longReported[idx] = true;
        triggerOut = true;
    }
}