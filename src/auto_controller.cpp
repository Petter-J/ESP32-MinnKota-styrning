#include "auto_controller.h"
#include "controller.h"

void AutoController::begin()
{
}

ActuatorCommand AutoController::update(
    float dtSec,
    SystemState &sys,
    PidController &headingPid,
    PidController &speedPid)
{
    (void)dtSec;
    (void)sys;
    (void)headingPid;
    (void)speedPid;

    ActuatorCommand out;
    out.thrustPct = 0.0f;
    out.steerPct = 0.0f;
    return out;
}