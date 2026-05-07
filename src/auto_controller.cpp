#include "auto_controller.h"
#include "controller.h"
#include "config.h"
#include <cstring>

static bool autoCanUseGpsCourse(const SystemState &sys)
{
    return sys.sensors.gpsValid &&
           sys.sensors.speedValid &&
           sys.sensors.speedMps >= AutoConfig::MIN_GPS_COURSE_SPEED_MPS;
}

static float getAutoCourseHeadingDeg(const SystemState &sys)
{
    if (AutoConfig::BENCH_TEST_AUTO_WITHOUT_GPS)
        return sys.sensors.motorHeadingDeg;

    return sys.sensors.courseOverGroundDeg;
}

static float speedPctToMps(float pct)
{
    const float clampedPct = clampf(pct, 0.0f, 100.0f);
    return (clampedPct / 100.0f) * AutoConfig::MAX_SPEED_MPS;
}

void AutoController::begin()
{
}

ActuatorCommand AutoController::update(
    float dtSec,
    SystemState &sys,
    PidController &headingPid,
    PidController &speedPid)
{
    ActuatorCommand out;
    strcpy(sys.sensors.autoState, "RUN");

    const float currentSpeedMps = sys.sensors.speedMps;

    if (!AutoConfig::BENCH_TEST_AUTO_WITHOUT_GPS &&
        !autoCanUseGpsCourse(sys))

    {
        strcpy(sys.sensors.autoState, "LOW SPD");

        sys.mode = SystemMode::MANUAL;

        out.thrustPct = clampf(
            sys.manualThrustPct,
            Limits::THRUST_MIN_PCT,
            Limits::THRUST_MAX_PCT);

        out.steerPct = 0.0f;
        return out;
    }

    const float currentHeadingDeg = getAutoCourseHeadingDeg(sys);

    float headingError =
        shortestAngleErrorDeg(sys.targetHeadingDeg, currentHeadingDeg);

    float steerCmd = headingPid.update(headingError, dtSec);

    const float targetSpeedMps = speedPctToMps(sys.targetSpeedPct);

    float speedError = targetSpeedMps - currentSpeedMps;
    float thrustCmd = speedPid.update(speedError, dtSec);

    out.steerPct =
        clampf(steerCmd, Limits::STEER_MIN_PCT, Limits::STEER_MAX_PCT);

    out.thrustPct =
        clampf(thrustCmd, Limits::THRUST_MIN_PCT, Limits::THRUST_MAX_PCT);

    return out;
}