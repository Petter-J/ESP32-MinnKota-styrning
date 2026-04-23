#pragma once

#include "types.h"

class PidController;

class AnchorController
{
public:
    void onEnter(SystemState &sys);
    ActuatorCommand update(float dtSec, SystemState &sys, PidController &headingPid);

private:
    static float degToRad(float deg);
    static float radToDeg(float rad);
    static float distanceMeters(double lat1Deg, double lon1Deg, double lat2Deg, double lon2Deg);
    static float bearingDeg(double lat1Deg, double lon1Deg, double lat2Deg, double lon2Deg);
};