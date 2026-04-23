#pragma once
#include <Arduino.h>
#include "types.h"
#include "anchor_controller.h"

class PidController
{
public:
    void setTunings(float kp, float ki, float kd);
    void setOutputLimits(float minOut, float maxOut);
    void reset();
    float update(float error, float dtSec);

private:
    float _kp = 0.0f;
    float _ki = 0.0f;
    float _kd = 0.0f;

    float _integral = 0.0f;
    float _prevError = 0.0f;
    bool _first = true;

    float _outMin = -100.0f;
    float _outMax = 100.0f;
};

class MainController
{
public:
    void begin();
    void onModeChanged(SystemMode newMode, SystemState &sys);
    void update(float dtSec, SystemState &sys);

private:
    ActuatorCommand computeStop(const SystemState &sys);
    ActuatorCommand computeManual(const SystemState &sys);
    ActuatorCommand computeAuto(float dtSec, const SystemState &sys);
    ActuatorCommand computeAnchor(float dtSec, SystemState &sys);

    PidController _headingPid;
    PidController _speedPid;
    AnchorController _anchor;
};

void applyCommand(const RemoteCommand &cmd, SystemState &sys, MainController &controller, ControlSource src);