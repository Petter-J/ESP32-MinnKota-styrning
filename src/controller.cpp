#include "controller.h"
#include "config.h"
#include <cstring>
#include <math.h>


void PidController::setTunings(float kp, float ki, float kd)
{
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

void PidController::setOutputLimits(float minOut, float maxOut)
{
    _outMin = minOut;
    _outMax = maxOut;
}

void PidController::reset()
{
    _integral = 0.0f;
    _prevError = 0.0f;
    _first = true;
}

float PidController::update(float error, float dtSec)
{
    if (dtSec <= 0.0f)
        return 0.0f;

    _integral += error * dtSec;

    // Anti-windup
    if (_integral * _ki > _outMax)
        _integral = _outMax / (_ki != 0.0f ? _ki : 1.0f);
    else if (_integral * _ki < _outMin)
        _integral = _outMin / (_ki != 0.0f ? _ki : 1.0f);

    float derivative = 0.0f;
    if (_first)
    {
        _first = false;
    }
    else
    {
        derivative = (error - _prevError) / dtSec;
    }

    _prevError = error;

    float out = (_kp * error) + (_ki * _integral) + (_kd * derivative);
    return clampf(out, _outMin, _outMax);
}

// ------------------------------------------------------------

void MainController::begin()
{
    _headingPid.setTunings(
        ControlDefaults::HEADING_KP,
        ControlDefaults::HEADING_KI,
        ControlDefaults::HEADING_KD);
    _headingPid.setOutputLimits(-100.0f, 100.0f);

    _speedPid.setTunings(
        ControlDefaults::SPEED_KP,
        ControlDefaults::SPEED_KI,
        ControlDefaults::SPEED_KD);
    _speedPid.setOutputLimits(0.0f, 100.0f);

    _auto.begin();  
}

void MainController::onModeChanged(SystemMode newMode, SystemState &sys)
{
    _headingPid.reset();
    _speedPid.reset();

    switch (newMode)
    {
    case SystemMode::STOP:
        sys.targetSpeedPct = 0.0f;
        sys.manualThrustPct = 0.0f;
        sys.manualSteerPct = 0.0f;
        break;

    case SystemMode::MANUAL:
        if (sys.manualThrustPct < ManualControlConfig::THRUST_START_MIN_PCT)
        {
            sys.manualThrustPct = ManualControlConfig::THRUST_START_MIN_PCT;
        }
        break;

    case SystemMode::AUTO:
        if (sys.sensors.headingValid)
        {
            sys.targetHeadingDeg = sys.sensors.headingDeg;
        }

        if (sys.sensors.speedMps >= AutoConfig::MIN_GPS_COURSE_SPEED_MPS)
        {
            sys.targetSpeedPct = sys.sensors.speedPct;
        }
        else
        {
            sys.targetSpeedPct = AutoConfig::START_THRUST_PCT;
        }
        break;

    case SystemMode::ANCHOR:
        _anchor.onEnter(sys);
        break;
    }
}

void MainController::update(float dtSec, SystemState &sys)
{
    switch (sys.mode)
    {
    case SystemMode::STOP:
        sys.actuators = computeStop(sys);
        break;

    case SystemMode::MANUAL:
        sys.actuators = computeManual(sys);
        break;

    case SystemMode::AUTO:
        sys.actuators = computeAuto(dtSec, sys);
        break;

    case SystemMode::ANCHOR:
        sys.actuators = computeAnchor(dtSec, sys);
        break;
    }
}

ActuatorCommand MainController::computeStop(const SystemState &sys)
{
    (void)sys;

    ActuatorCommand out;
    out.thrustPct = 0.0f;
    out.steerPct = 0.0f;
    return out;
}

ActuatorCommand MainController::computeManual(const SystemState &sys)
{
    ActuatorCommand out;
    out.thrustPct = clampf(sys.manualThrustPct, Limits::THRUST_MIN_PCT, Limits::THRUST_MAX_PCT);
    out.steerPct = clampf(sys.manualSteerPct, Limits::STEER_MIN_PCT, Limits::STEER_MAX_PCT);
    return out;
}

ActuatorCommand MainController::computeAuto(float dtSec, SystemState &sys)
{
    return _auto.update(dtSec, sys, _headingPid, _speedPid);
}

ActuatorCommand MainController::computeAnchor(float dtSec, SystemState &sys)
{
    return _anchor.update(dtSec, sys, _headingPid);
}

// ------------------------------------------------------------

void applyCommand(const RemoteCommand &cmd, SystemState &sys, MainController &controller, ControlSource src)
{
    if (!cmd.valid)
        return;

    sys.lastCommandTimeMs = millis();

    sys.lastCommand = cmd;
    sys.lastControlSource = src;

    const SystemMode oldMode = sys.mode;

    if (cmd.requestManual)
        sys.mode = SystemMode::MANUAL;
    if (cmd.requestAuto)
        sys.mode = SystemMode::AUTO;
    if (cmd.requestAnchor)
        sys.mode = SystemMode::ANCHOR;

    if (sys.mode != oldMode)
    {
        controller.onModeChanged(sys.mode, sys);
        DBG_PRINTF("[MODE] -> %s\n", modeToString(sys.mode));
    }

    if (cmd.hasManualThrust)
    {
        sys.manualThrustPct = clampf(cmd.manualThrustPct, Limits::THRUST_MIN_PCT, Limits::THRUST_MAX_PCT);
    }

    if (cmd.hasManualSteer)
    {
        sys.manualSteerPct = clampf(cmd.manualSteerPct, Limits::STEER_MIN_PCT, Limits::STEER_MAX_PCT);
    }

    if (cmd.hasTargetHeading)
    {
        sys.targetHeadingDeg = wrap360(cmd.targetHeadingDeg);
    }

    if (cmd.hasTargetSpeed)
    {
        sys.targetSpeedPct = clampf(cmd.targetSpeedPct, Limits::THRUST_MIN_PCT, Limits::THRUST_MAX_PCT);
        sys.targetSpeedMps = sys.sensors.gpsSpeedMps;
    }

    if (cmd.hasAnchorHere && cmd.anchorHere)
    {
        if (sys.sensors.gpsValid)
        {
            sys.anchorLatDeg = sys.sensors.latitudeDeg;
            sys.anchorLonDeg = sys.sensors.longitudeDeg;
            sys.anchorActive = true;
        }
    }
}