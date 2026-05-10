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
    if (btn.anchorHeld &&
        sys.sensors.gpsValid)
    {
        if (!_anchorCollecting)
        {
            _anchorCollecting = true;
            _anchorSumLat = 0.0f;
            _anchorSumLon = 0.0f;
            _anchorCount = 0;
        }

        // Bara när GPS faktiskt har ny position
        if (sys.sensors.locationUpdated)
        {
            _anchorSumLat += sys.sensors.latitudeDeg;
            _anchorSumLon += sys.sensors.longitudeDeg;
            _anchorCount++;
        }
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

    static bool motorTiltSafe = true;

    if (MotorTiltSafetyConfig::ENABLED && sys.sensors.motorImuValid)
    {
        const bool tiltExceeded =
            fabsf(sys.sensors.motorPitchDeg) > MotorTiltSafetyConfig::STOP_TILT_DEG ||
            fabsf(sys.sensors.motorRollDeg) > MotorTiltSafetyConfig::STOP_TILT_DEG;

        const bool tiltRecovered =
            fabsf(sys.sensors.motorPitchDeg) < MotorTiltSafetyConfig::RECOVER_TILT_DEG &&
            fabsf(sys.sensors.motorRollDeg) < MotorTiltSafetyConfig::RECOVER_TILT_DEG;

        if (motorTiltSafe && tiltExceeded)
        {
            motorTiltSafe = false;
            sys.sensors.motorTiltUnsafe = true;
            setMode(SystemMode::STOP, nowMs, sys, controller);
            return;
        }

        static uint32_t tiltRecoveredStartMs = 0;

        if (!motorTiltSafe)
        {
            if (tiltRecovered)
            {
                if (tiltRecoveredStartMs == 0)
                {
                    tiltRecoveredStartMs = nowMs;
                }

                if (nowMs - tiltRecoveredStartMs >= 1000)
                {
                    motorTiltSafe = true;
                    sys.sensors.motorTiltUnsafe = false;
                }
            }
            else
            {
                tiltRecoveredStartMs = 0;
                setMode(SystemMode::STOP, nowMs, sys, controller);
                return;
            }
        }
        else
        {
            tiltRecoveredStartMs = 0;
        }
    }
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
            
            setMode(SystemMode::STOP, nowMs, sys, controller);
        }
        return;
    }

    if (!SafetyConfig::ENABLE_SENSOR_MODE_SAFETY)
        return;

    const bool autoSensorsOk =
        sys.sensors.gpsValid &&
        sys.sensors.speedValid &&
        sys.sensors.boatImuValid;

    const bool anchorSensorsOk =
        sys.sensors.gpsValid &&
        sys.sensors.motorImuValid;

    // =========================
    // AUTO
    // =========================
    if (sys.mode == SystemMode::AUTO)
    {
        if (autoSensorsOk)
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
           
            setMode(SystemMode::STOP, nowMs, sys, controller);
        }
    }

    // =========================
    // ANCHOR
    // =========================
    else if (sys.mode == SystemMode::ANCHOR)
    {
        if (anchorSensorsOk)
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

        sys.anchorActive = false;
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

    
}

void InputLogic::handleStop(
    const ButtonOutput &btn,
    uint32_t nowMs,
    SystemState &sys,
    MainController &controller)
{
    if (!btn.stopRequested)
        return;

    if (sys.mode != SystemMode::STOP)
    {
        setMode(SystemMode::STOP, nowMs, sys, controller);
    }
}

void InputLogic::handleModeButtons(
    const ButtonOutput &btn,
    uint32_t nowMs,
    SystemState &sys,
    MainController &controller)
{
    const uint8_t modePressCount =
        (btn.requestManual ? 1 : 0) +
        (btn.requestAuto ? 1 : 0) +
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

    if (btn.requestAuto)
    {
        if (sys.mode == SystemMode::AUTO)
        {
            setMode(SystemMode::STOP, nowMs, sys, controller);
            return;
        }

      //----------------TILLFÄLLIG FÖR TESTNING------------------
        if (AutoConfig::BENCH_TEST_AUTO_WITHOUT_GPS)
        {
            sys.targetHeadingDeg = sys.sensors.motorHeadingDeg;
            sys.targetSpeedMps = sys.sensors.gpsSpeedMps;

            sys.targetSpeedPct = clampf(
                (sys.targetSpeedMps / AutoConfig::MAX_SPEED_MPS) * 100.0f,
                Limits::THRUST_MIN_PCT,
                Limits::THRUST_MAX_PCT);
        }
      //----------------------------------------------------------
        else
        {
            if (!sys.sensors.gpsValid ||
                !sys.sensors.speedValid ||
                !sys.sensors.courseValid ||
                sys.sensors.gpsSpeedMps < AutoConfig::MIN_GPS_COURSE_SPEED_MPS)
            {
                return;
            }

            sys.targetHeadingDeg = sys.sensors.courseOverGroundDeg;
            sys.targetSpeedMps = sys.sensors.gpsSpeedMps;

            sys.targetSpeedPct = clampf(
                (sys.targetSpeedMps / AutoConfig::MAX_SPEED_MPS) * 100.0f,
                Limits::THRUST_MIN_PCT,
                Limits::THRUST_MAX_PCT);
        }

        setMode(SystemMode::AUTO, nowMs, sys, controller);
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
                }

                setMode(SystemMode::ANCHOR, nowMs, sys, controller);
            }
            else
            {
                
                setMode(SystemMode::STOP, nowMs, sys, controller);
            }
        }

        return;
    }
}

void InputLogic::handleManualButtons(
    const ButtonOutput &btn,
    uint32_t nowMs,
    SystemState &sys)
{
    if (sys.mode != SystemMode::MANUAL)
        return;

    if (nowMs - _lastManualAdjustMs < _cfg.manualRepeatMs)
        return;

    _lastManualAdjustMs = nowMs;

    const bool thrustUp = btn.thrustUpHeld;
    const bool thrustDown = btn.thrustDownHeld;
    const bool steerLeft = btn.steerLeftHeld;
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
                _cfg.manualThrustMaxPct);
        }

        
    }
    else if (thrustDown && !thrustUp)
    {
        if (sys.manualThrustPct > _cfg.manualThrustMinPct)
        {
            sys.manualThrustPct = clampf(
                sys.manualThrustPct - _cfg.manualThrustStepPct,
                _cfg.manualThrustMinPct,
                _cfg.manualThrustMaxPct);
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
        static uint32_t lastSteerInputMs = 0;

        if (steerLeft || steerRight)
        {
            lastSteerInputMs = nowMs;
        }

        if (nowMs - lastSteerInputMs > 75)
        {
            sys.manualSteerPct = 0.0f;
        }
    }
}
void InputLogic::handleAutoButtons(
    const ButtonOutput &btn,
    uint32_t nowMs,
    SystemState &sys)
{
    if (sys.mode != SystemMode::AUTO)
        return;

    if (nowMs - _lastManualAdjustMs < AutoControlConfig::REPEAT_MS)
        return;

    _lastManualAdjustMs = nowMs;

    static constexpr float KN_TO_MPS = 0.514444f;

    const float speedStepMps =
        AutoControlConfig::SPEED_STEP_KN * KN_TO_MPS;

    if (btn.thrustUpHeld && !btn.thrustDownHeld)
    {
        sys.targetSpeedMps = clampf(
            sys.targetSpeedMps + speedStepMps,
            0.0f,
            AutoConfig::MAX_SPEED_MPS);

        sys.targetSpeedPct = clampf(
            (sys.targetSpeedMps / AutoConfig::MAX_SPEED_MPS) * 100.0f,
            Limits::THRUST_MIN_PCT,
            Limits::THRUST_MAX_PCT);
    }

    else if (btn.thrustDownHeld && !btn.thrustUpHeld)
    {
        sys.targetSpeedMps = clampf(
            sys.targetSpeedMps - speedStepMps,
            0.0f,
            AutoConfig::MAX_SPEED_MPS);

        sys.targetSpeedPct = clampf(
            (sys.targetSpeedMps / AutoConfig::MAX_SPEED_MPS) * 100.0f,
            Limits::THRUST_MIN_PCT,
            Limits::THRUST_MAX_PCT);    
    }

    // HEADING
    if (btn.steerLeftHeld && !btn.steerRightHeld)
    {
        sys.targetHeadingDeg = wrap360(
            sys.targetHeadingDeg - AutoControlConfig::HEADING_STEP_DEG);   
    }

    else if (btn.steerRightHeld && !btn.steerLeftHeld)
    {
        sys.targetHeadingDeg = wrap360(
            sys.targetHeadingDeg + AutoControlConfig::HEADING_STEP_DEG);

        
    }
}
