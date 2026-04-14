#pragma once
#include <Arduino.h>
#include "config.h"
#include "types.h"
#include "buttons.h"
#include "controller.h"

struct InputLogicConfig
{
    float manualThrustStepPct = ManualControlConfig::THRUST_STEP_PCT;

    float manualThrustMinPct  = ManualControlConfig::THRUST_START_MIN_PCT;
    float manualThrustMaxPct  = 100.0f;

    float manualSteerMinPct   = -ManualControlConfig::STEER_JOG_PCT;
    float manualSteerMaxPct   = ManualControlConfig::STEER_JOG_PCT;

    uint32_t manualRepeatMs   = ManualControlConfig::REPEAT_MS;
};

class InputLogic
{
public:
    void begin();

    void applyButtons(
        const ButtonOutput& btn,
        uint32_t nowMs,
        SystemState& sys,
        MainController& controller);

    void applySafety(
        uint32_t nowMs,
        SystemState& sys,
        MainController& controller);

private:
    void setMode(
        SystemMode newMode,
        uint32_t nowMs,
        SystemState& sys,
        MainController& controller);

    void handleStop(
        const ButtonOutput& btn,
        uint32_t nowMs,
        SystemState& sys,
        MainController& controller);

    void handleModeButtons(
        const ButtonOutput& btn,
        uint32_t nowMs,
        SystemState& sys,
        MainController& controller);

    void handleManualButtons(
        const ButtonOutput& btn,
        uint32_t nowMs,
        SystemState& sys);

    void handleAutoButtons(
        const ButtonOutput& btn,
        uint32_t nowMs,
        SystemState& sys);  

private:
    InputLogicConfig _cfg{};
    uint32_t _lastManualAdjustMs = 0;
};