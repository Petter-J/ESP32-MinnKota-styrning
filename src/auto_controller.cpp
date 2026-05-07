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

float AutoController::getAutoCourseHeadingDeg(const SystemState &sys)
{
    if (AutoConfig::BENCH_TEST_AUTO_WITHOUT_GPS)
        return sys.sensors.motorHeadingDeg;

    return filterCogDeg(sys.sensors.courseOverGroundDeg);
}

static float speedPctToMps(float pct)
{
    const float clampedPct = clampf(pct, 0.0f, 100.0f);
    return (clampedPct / 100.0f) * AutoConfig::MAX_SPEED_MPS;
}

float AutoController::filterCogDeg(float rawCogDeg)
{
    if (!_cogFilterInitialized)
    {
        _filteredCogDeg = rawCogDeg;
        _cogFilterInitialized = true;
        return _filteredCogDeg;
    }

    const float diffDeg =
        shortestAngleErrorDeg(rawCogDeg, _filteredCogDeg);

    if (fabs(diffDeg) <= AutoConfig::COG_MAX_JUMP_DEG)
    {
        _filteredCogDeg =
            wrap360(_filteredCogDeg + diffDeg * AutoConfig::COG_FILTER_ALPHA);
    }

    return _filteredCogDeg;
}

static ActuatorCommand makeManualFallbackCommand(SystemState &sys)
{
    sys.mode = SystemMode::MANUAL;

    ActuatorCommand out;
    out.thrustPct = clampf(
        sys.manualThrustPct,
        Limits::THRUST_MIN_PCT,
        Limits::THRUST_MAX_PCT);

    out.steerPct = 0.0f;
    return out;
}

static float computeSpeedThrustPct(
    float targetSpeedPct,
    float currentSpeedMps,
    PidController &speedPid,
    float dtSec)
{
    const float targetSpeedMps = speedPctToMps(targetSpeedPct);
    const float speedError = targetSpeedMps - currentSpeedMps;

    const float thrustCmd = speedPid.update(speedError, dtSec);

    return clampf(
        thrustCmd,
        Limits::THRUST_MIN_PCT,
        Limits::THRUST_MAX_PCT);
}

static float computeHeadingSteerPct(
    float targetHeadingDeg,
    float currentHeadingDeg,
    PidController &headingPid,
    float dtSec)
{
    const float headingError =
        shortestAngleErrorDeg(targetHeadingDeg, currentHeadingDeg);

    const float steerCmd =
        headingPid.update(headingError, dtSec);

    return clampf(
        steerCmd,
        Limits::STEER_MIN_PCT,
        Limits::STEER_MAX_PCT);
}

void AutoController::begin()
{
    _cogFilterInitialized = false;
    _filteredCogDeg = 0.0f;
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
        return makeManualFallbackCommand(sys);
    }

    const float currentHeadingDeg = getAutoCourseHeadingDeg(sys);

    out.steerPct =
        computeHeadingSteerPct(
            sys.targetHeadingDeg,
            currentHeadingDeg,
            headingPid,
            dtSec);

    out.thrustPct =
        computeSpeedThrustPct(
            sys.targetSpeedPct,
            currentSpeedMps,
            speedPid,
            dtSec);

    return out;
}