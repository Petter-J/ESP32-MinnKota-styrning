#include "input_logic.h"
#include "config.h"

void InputLogic::begin()
{
    _lastManualAdjustMs = 0;
    _lastValidAutoSensorMs = 0;
    _lastValidAnchorSensorMs = 0;

    _anchorCollecting = false;
    _anchorSumLat = 0.0f;
    _anchorSumLon = 0.0f;
    _anchorCount = 0;
}

void InputLogic::applyButtons(
    const ButtonOutput &btn,
    uint32_t nowMs,
    SystemState &sys,
    MainController &controller)
{
    // 🔥 Anchor GPS sampling while holding button
    if (btn.anchorHeld && sys.sensors.gpsValid)
    {
        if (!_anchorCollecting)
        {
            _anchorCollecting = true;
            _anchorSumLat = 0.0f;
            _anchorSumLon = 0.0f;
            _anchorCount = 0;
        }

        _anchorSumLat += sys.sensors.latitudeDeg;
        _anchorSumLon += sys.sensors.longitudeDeg;
        _anchorCount++;
    }
    else
    {
        _anchorCollecting = false;
    }

    handleStop(btn, nowMs, sys, controller);

    if (sys.mode != SystemMode::STOP)
    {
        handleModeButtons(btn, nowMs, sys, controller);
    }
    else
    {
        handleModeButtons(btn, nowMs, sys, controller);
    }

    handleManualButtons(btn, nowMs, sys);
    handleAutoButtons(btn, nowMs, sys);
}

void InputLogic::applySafety(
    uint32_t nowMs,
    SystemState &sys,
    MainController &controller)
{
    // =========================
    // COMMAND TIMEOUT (link lost)
    // =========================
    const uint32_t commandAgeMs =
        (nowMs >= sys.lastCommandTimeMs)
            ? (nowMs - sys.lastCommandTimeMs)
            : 0;

    if (sys.lastCommandTimeMs > 0 &&
        commandAgeMs > SafetyConfig::COMMAND_TIMEOUT_MS)
    {
        if (sys.mode != SystemMode::STOP)
        {
            DBG_PRINTF("[SAFE] command timeout age=%lu -> STOP\n", commandAgeMs);
            setMode(SystemMode::STOP, nowMs, sys, controller);
        }
        return;
    }

    if (!SafetyConfig::ENABLE_SENSOR_MODE_SAFETY)
        return;

    const bool sensorsOk =
        sys.sensors.gpsValid &&
        sys.sensors.headingValid;

    // =========================
    // AUTO
    // =========================
    if (sys.mode == SystemMode::AUTO)
    {
        if (sensorsOk)
        {
            _lastValidAutoSensorMs = nowMs;
            return;
        }

        if (_lastValidAutoSensorMs == 0)
        {
            _lastValidAutoSensorMs = nowMs;
        }

        if (nowMs - _lastValidAutoSensorMs > SafetyConfig::SENSOR_FAIL_TIMEOUT_MS)
        {
            DBG_PRINTLN("[SAFE] AUTO sensor timeout -> STOP");
            setMode(SystemMode::STOP, nowMs, sys, controller);
        }
    }

    // =========================
    // ANCHOR
    // =========================
    else if (sys.mode == SystemMode::ANCHOR)
    {
        if (sensorsOk)
        {
            _lastValidAnchorSensorMs = nowMs;
            return;
        }

        if (_lastValidAnchorSensorMs == 0)
        {
            _lastValidAnchorSensorMs = nowMs;
        }

        if (nowMs - _lastValidAnchorSensorMs > SafetyConfig::SENSOR_FAIL_TIMEOUT_MS)
        {
            DBG_PRINTLN("[SAFE] ANCHOR sensor timeout -> STOP");
            setMode(SystemMode::STOP, nowMs, sys, controller);
        }
    }
}

void InputLogic::setMode(
    SystemMode newMode,
    uint32_t nowMs,
    SystemState &sys,
    MainController &controller)
{
    if (sys.mode == newMode)
        return;

    const SystemMode oldMode = sys.mode;

    // Reset anchor sampling när vi går till STOP
    if (newMode == SystemMode::STOP)
    {
        _anchorCollecting = false;
        _anchorSumLat = 0.0f;
        _anchorSumLon = 0.0f;
        _anchorCount = 0;
    }

    // Specialfall: AUTO -> MANUAL, ta över aktuell thrust
    if (oldMode == SystemMode::AUTO && newMode == SystemMode::MANUAL)
    {
        sys.manualThrustPct = clampf(
            sys.actuators.thrustPct,
            ManualControlConfig::THRUST_START_MIN_PCT,
            Limits::THRUST_MAX_PCT);

        sys.manualSteerPct = 0.0f;
    }

    sys.mode = newMode;
    controller.onModeChanged(newMode, sys);
    sys.lastCommandTimeMs = nowMs;

    DBG_PRINTF("[MODE] -> %s\n", modeToString(newMode));
}

void InputLogic::handleStop(
    const ButtonOutput& btn,
    uint32_t nowMs,
    SystemState& sys,
    MainController& controller)
{
    if (!btn.stopRequested)
        return;

    if (sys.mode != SystemMode::STOP)
    {
        setMode(SystemMode::STOP, nowMs, sys, controller);
    }
}

void InputLogic::handleModeButtons(
    const ButtonOutput& btn,
    uint32_t nowMs,
    SystemState& sys,
    MainController& controller)
{
    const uint8_t modePressCount =
        (btn.requestManual ? 1 : 0) +
        (btn.requestAuto   ? 1 : 0) +
        (btn.requestAnchor ? 1 : 0);

    if (modePressCount != 1)
        return;

    if (btn.requestManual)
    {
        if (sys.mode == SystemMode::MANUAL)
            setMode(SystemMode::STOP, nowMs, sys, controller);
        else
            setMode(SystemMode::MANUAL, nowMs, sys, controller);

        return;
    }

    if (btn.requestAnchor)
    {
        if (sys.mode == SystemMode::ANCHOR)
        {
            setMode(SystemMode::STOP, nowMs, sys, controller);
        }
        else
        {
            if (sys.actuators.thrustPct <= AnchorControlConfig::MAX_ENTRY_THRUST_PCT)
            {
                if (_anchorCount > 0)
                {
                    sys.anchorLatDeg = _anchorSumLat / _anchorCount;
                    sys.anchorLonDeg = _anchorSumLon / _anchorCount;
                    sys.anchorActive = true;

                    DBG_PRINTF("[ANCHOR] avg fix used (%u samples)\n", _anchorCount);
                }

                setMode(SystemMode::ANCHOR, nowMs, sys, controller);
            }
            else
            {
                DBG_PRINTF("[ANCHOR] thrust too high (%.1f) -> STOP\n", sys.actuators.thrustPct);
                setMode(SystemMode::STOP, nowMs, sys, controller);
            }
        }

        return;
    }
}

void InputLogic::handleManualButtons(
    const ButtonOutput& btn,
    uint32_t nowMs,
    SystemState& sys)
{
    if (sys.mode != SystemMode::MANUAL)
        return;

    if (nowMs - _lastManualAdjustMs < _cfg.manualRepeatMs)
        return;

    _lastManualAdjustMs = nowMs;

    const bool thrustUp   = btn.thrustUpHeld;
    const bool thrustDown = btn.thrustDownHeld;
    const bool steerLeft  = btn.steerLeftHeld;
    const bool steerRight = btn.steerRightHeld;

    if (thrustUp && !thrustDown)
    {
        if (sys.manualThrustPct < _cfg.manualThrustMinPct)
        {
            sys.manualThrustPct = _cfg.manualThrustMinPct;
        }
        else
        {
            sys.manualThrustPct = clampf(
                sys.manualThrustPct + _cfg.manualThrustStepPct,
                _cfg.manualThrustMinPct,
                _cfg.manualThrustMaxPct
            );
        }

        DBG_PRINTF("[MAN] thrust -> %.1f\n", sys.manualThrustPct);
    }
    else if (thrustDown && !thrustUp)
    {
        if (sys.manualThrustPct > _cfg.manualThrustMinPct)
        {
            sys.manualThrustPct = clampf(
                sys.manualThrustPct - _cfg.manualThrustStepPct,
                _cfg.manualThrustMinPct,
                _cfg.manualThrustMaxPct
            );

            DBG_PRINTF("[MAN] thrust -> %.1f\n", sys.manualThrustPct);
        }
    }

    if (steerLeft && !steerRight)
{
    sys.manualSteerPct = -ManualControlConfig::STEER_JOG_PCT;
}
else if (steerRight && !steerLeft)
{
    sys.manualSteerPct = ManualControlConfig::STEER_JOG_PCT;
}
else
{
    sys.manualSteerPct = 0.0f;
}
}
void InputLogic::handleAutoButtons(
    const ButtonOutput& btn,
    uint32_t nowMs,
    SystemState& sys)
{
    if (sys.mode != SystemMode::AUTO)
        return;

    if (nowMs - _lastManualAdjustMs < AutoControlConfig::REPEAT_MS)
        return;

    _lastManualAdjustMs = nowMs;

    // SPEED
    if (btn.thrustUpHeld && !btn.thrustDownHeld)
    {
        sys.targetSpeedPct = clampf(
            sys.targetSpeedPct + AutoControlConfig::SPEED_STEP_PCT,
            Limits::THRUST_MIN_PCT,
            Limits::THRUST_MAX_PCT
        );

        DBG_PRINTF("[AUTO] speed -> %.1f\n", sys.targetSpeedPct);
    }
    else if (btn.thrustDownHeld && !btn.thrustUpHeld)
    {
        sys.targetSpeedPct = clampf(
            sys.targetSpeedPct - AutoControlConfig::SPEED_STEP_PCT,
            Limits::THRUST_MIN_PCT,
            Limits::THRUST_MAX_PCT
        );

        DBG_PRINTF("[AUTO] speed -> %.1f\n", sys.targetSpeedPct);
    }

    // HEADING
    if (btn.steerLeftHeld && !btn.steerRightHeld)
    {
        sys.targetHeadingDeg = wrap360(
            sys.targetHeadingDeg - AutoControlConfig::HEADING_STEP_DEG
        );

        DBG_PRINTF("[AUTO] heading -> %.1f\n", sys.targetHeadingDeg);
    }
    else if (btn.steerRightHeld && !btn.steerLeftHeld)
    {
        sys.targetHeadingDeg = wrap360(
            sys.targetHeadingDeg + AutoControlConfig::HEADING_STEP_DEG
        );

        DBG_PRINTF("[AUTO] heading -> %.1f\n", sys.targetHeadingDeg);
    }
}
