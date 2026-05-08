#pragma once

#include "types.h"

class PidController;

class AutoController
{
public:
    void begin();

    ActuatorCommand update(
        float dtSec,
        SystemState &sys,
        PidController &headingPid,
        PidController &speedPid);

private:
    bool _cogFilterInitialized = false;
    float _filteredCogDeg = 0.0f;
    bool _steerActive = false;

    float filterCogDeg(float rawCogDeg);
    float getAutoCourseHeadingDeg(const SystemState &sys);

    float computeDesiredSteerOffsetDeg(float courseErrorDeg) const;
    float computeActualSteerOffsetDeg(const SystemState &sys) const;
    bool updateSteerActive(float steerErrorDeg);

    float computeSteerPctFromOffset(
        const SystemState &sys,
        float desiredOffsetDeg,
        PidController &headingPid,
        float dtSec);

    float computeSpeedThrustPct(
        float targetSpeedPct,
        float currentSpeedMps,
        PidController &speedPid,
        float dtSec);
};