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
    float filterCogDeg(float rawCogDeg);
    float getAutoCourseHeadingDeg(const SystemState &sys);
};