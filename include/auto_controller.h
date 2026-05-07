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
};