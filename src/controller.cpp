#include "controller.h"
#include "config.h"
#include <cstring>

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
    if (dtSec <= 0.0f) return 0.0f;

    _integral += error * dtSec;

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
        ControlDefaults::HEADING_KD
    );
    _headingPid.setOutputLimits(-100.0f, 100.0f);

    _speedPid.setTunings(
        ControlDefaults::SPEED_KP,
        ControlDefaults::SPEED_KI,
        ControlDefaults::SPEED_KD
    );
    _speedPid.setOutputLimits(0.0f, 100.0f);
}

void MainController::onModeChanged(SystemMode newMode, SystemState& sys)
{
    _headingPid.reset();
    _speedPid.reset();

    switch (newMode)
    {
        case SystemMode::STOP:
    sys.targetSpeedPct  = 0.0f;
    sys.manualThrustPct = 0.0f;
    sys.manualSteerPct  = 0.0f;
    break;

        case SystemMode::MANUAL:
    sys.manualSteerPct = 0.0f;

    if (sys.manualThrustPct < ManualControlConfig::THRUST_START_MIN_PCT)
    {
        sys.manualThrustPct = ManualControlConfig::THRUST_START_MIN_PCT;
    }
    break;

        case SystemMode::AUTO:
            sys.targetHeadingDeg = sys.sensors.headingDeg;
            sys.targetSpeedPct   = sys.sensors.speedPct;
            break;

        case SystemMode::ANCHOR:
            break;
    }
}

void MainController::update(float dtSec, SystemState& sys)
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

ActuatorCommand MainController::computeStop(const SystemState& sys)
{
    (void)sys;

    ActuatorCommand out;
    out.thrustPct = 0.0f;
    out.steerPct  = 0.0f;
    //strcpy(((SystemState&)sys).sensors.autoState, "STOP");
    return out;
}

ActuatorCommand MainController::computeManual(const SystemState& sys)
{
    ActuatorCommand out;
    out.thrustPct = clampf(sys.manualThrustPct, Limits::THRUST_MIN_PCT, Limits::THRUST_MAX_PCT);
    out.steerPct  = clampf(sys.manualSteerPct, Limits::STEER_MIN_PCT, Limits::STEER_MAX_PCT);
    //strcpy(((SystemState&)sys).sensors.autoState, "MAN");
    return out;
}

ActuatorCommand MainController::computeAuto(float dtSec, const SystemState& sys)
{
    ActuatorCommand out;
    strcpy(((SystemState&)sys).sensors.autoState, "RUN");

    // AUTO-startläge:
    // Om båten går för långsamt är GPS course opålitlig.
    // Då ger vi en fast start-thrust och använder befintlig heading som stöd
    // tills båten fått upp fart.
    if (sys.sensors.gpsSpeedMps < AutoConfig::MIN_GPS_COURSE_SPEED_MPS)
    {
        strcpy(((SystemState&)sys).sensors.autoState, "START");

        float headingError = shortestAngleErrorDeg(sys.targetHeadingDeg, sys.sensors.headingDeg);
        float steerCmd = _headingPid.update(headingError, dtSec);

        out.steerPct = clampf(steerCmd, Limits::STEER_MIN_PCT, Limits::STEER_MAX_PCT);
        out.thrustPct = clampf(
            AutoConfig::START_THRUST_PCT,
            Limits::THRUST_MIN_PCT,
            Limits::THRUST_MAX_PCT
        );

        return out;
    }

    // Normal AUTO när båten har fart nog
    float headingError = shortestAngleErrorDeg(sys.targetHeadingDeg, sys.sensors.headingDeg);
    float steerCmd = _headingPid.update(headingError, dtSec);

    float speedError = sys.targetSpeedPct - sys.sensors.speedPct;
    float thrustCmd = _speedPid.update(speedError, dtSec);

    out.steerPct  = clampf(steerCmd, Limits::STEER_MIN_PCT, Limits::STEER_MAX_PCT);
    out.thrustPct = clampf(thrustCmd, Limits::THRUST_MIN_PCT, Limits::THRUST_MAX_PCT);

    float headingAbs = fabsf(headingError);
    if (headingAbs > 45.0f) out.thrustPct *= 0.6f;
    if (headingAbs > 90.0f) out.thrustPct *= 0.35f;

    return out;
}

ActuatorCommand MainController::computeAnchor(float dtSec, const SystemState& sys)
{
    (void)dtSec;

    ActuatorCommand out;
    float headingError = shortestAngleErrorDeg(sys.targetHeadingDeg, sys.sensors.headingDeg);
    out.steerPct = clampf(headingError, Limits::STEER_MIN_PCT, Limits::STEER_MAX_PCT);
    out.thrustPct = 0.0f;
    //strcpy(((SystemState&)sys).sensors.autoState, "ANCHOR");
    return out;
}

// ------------------------------------------------------------

void applyCommand(const RemoteCommand& cmd, SystemState& sys, MainController& controller, ControlSource src)
{
    if (!cmd.valid) return;

    sys.lastCommandTimeMs = millis();

    sys.lastCommand = cmd;
    sys.lastControlSource = src;

    const SystemMode oldMode = sys.mode;

    if (cmd.requestManual) sys.mode = SystemMode::MANUAL;
    if (cmd.requestAuto)   sys.mode = SystemMode::AUTO;
    if (cmd.requestAnchor) sys.mode = SystemMode::ANCHOR;

    if (sys.mode != oldMode)
    {
        controller.onModeChanged(sys.mode, sys);
        DBG_PRINTF("[MODE] -> %s\n", modeToString(sys.mode));
    }

    if (cmd.hasManualThrust)
    {
        sys.actuators.thrustPct = clampf(cmd.manualThrustPct, Limits::THRUST_MIN_PCT, Limits::THRUST_MAX_PCT);
    }

    if (cmd.hasManualSteer)
    {
        sys.actuators.steerPct = clampf(cmd.manualSteerPct, Limits::STEER_MIN_PCT, Limits::STEER_MAX_PCT);
    }

    if (cmd.hasTargetHeading)
    {
        sys.targetHeadingDeg = wrap360(cmd.targetHeadingDeg);
    }

    if (cmd.hasTargetSpeed)
    {
        sys.targetSpeedPct = clampf(cmd.targetSpeedPct, Limits::THRUST_MIN_PCT, Limits::THRUST_MAX_PCT);
    }

    if (cmd.hasAnchorHere && cmd.anchorHere)
    {
        // placeholder for future GPS anchor position
    }
}