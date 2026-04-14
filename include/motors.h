#pragma once
#include <Arduino.h>
#include "types.h"

class ThrustMotor
{
public:
    void begin();
    void setPercent(float pct);
    float getPercent() const;
    void stop();

private:
    float _pct = 0.0f;
};

class SteeringMotor
{
public:
    void begin();
    void setPercent(float pct);
    float getPercent() const;
    void stop();

private:
    float _pct = 0.0f;
};

class MotorManager
{
public:
    void begin();
    void apply(const ActuatorCommand& cmd, bool motorsEnabled, float dtSec);
    void stopAll();

    float getThrustPercent() const;
    float getSteerPercent() const;

private:
    ThrustMotor _thrust;
    SteeringMotor _steer;

    // Rampad thrust + steering
    float _thrustFiltered = 0.0f;
    float _steerFiltered  = 0.0f;
};