#pragma once
#include <Arduino.h>
#include "types.h"

class BoatSimulator
{
public:
    void begin();
    void update(float dtSec, const ActuatorCommand& cmd, SensorData& sensors);

    void setHeading(float deg);
    void setSpeed(float pct);

private:
    float _headingDeg = 0.0f;
    float _speedPct   = 0.0f;
    float _posX       = 0.0f;
    float _posY       = 0.0f;
};