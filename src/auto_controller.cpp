#include "auto_controller.h"
#include "controller.h"
#include "config.h"

#include <cstring>
#include <cmath>

// =====================================================
// Local helpers
// =====================================================

static bool autoCanUseGpsCourse(const SystemState &sys)
{
    return sys.sensors.gpsValid &&
           sys.sensors.speedValid &&
           sys.sensors.speedMps >= AutoConfig::MIN_GPS_COURSE_SPEED_MPS;
}

static float speedPctToMps(float pct)
{
    const float clampedPct = clampf(pct, 0.0f, 100.0f);
    return (clampedPct / 100.0f) * AutoConfig::MAX_SPEED_MPS;
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

// =====================================================
// AutoController
// =====================================================

void AutoController::begin()
{
    _cogFilterInitialized = false;
    _filteredCogDeg = 0.0f;
    _steerActive = false;
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

    if (std::fabs(diffDeg) <= AutoConfig::COG_MAX_JUMP_DEG)
    {
        _filteredCogDeg =
            wrap360(_filteredCogDeg + diffDeg * AutoConfig::COG_FILTER_ALPHA);
    }

    return _filteredCogDeg;
}

float AutoController::getAutoCourseHeadingDeg(const SystemState &sys)
{
    if (AutoConfig::BENCH_TEST_AUTO_WITHOUT_GPS)
        return sys.sensors.motorHeadingDeg;

    if (sys.sensors.locationUpdated)
    {
        return filterCogDeg(sys.sensors.courseOverGroundDeg);
    }

    return _filteredCogDeg;
}

float AutoController::computeDesiredSteerOffsetDeg(float courseErrorDeg) const
{
    if (std::fabs(courseErrorDeg) < AutoConfig::STEER_ERROR_STOP_DEG)
        return 0.0f;

    float offsetDeg = courseErrorDeg;

    if (offsetDeg > 0.0f)
        offsetDeg += AutoConfig::AUTO_STEER_EXTRA_DEG;
    else
        offsetDeg -= AutoConfig::AUTO_STEER_EXTRA_DEG;

    return clampf(
        offsetDeg,
        -AutoConfig::MAX_STEER_OFFSET_DEG,
        AutoConfig::MAX_STEER_OFFSET_DEG);
}

float AutoController::computeActualSteerOffsetDeg(const SystemState &sys) const
{
    return shortestAngleErrorDeg(
        sys.sensors.motorHeadingDeg,
        sys.sensors.boatHeadingDeg);
}

bool AutoController::updateSteerActive(float steerErrorDeg)
{
    const float absError = std::fabs(steerErrorDeg);

    if (_steerActive)
    {
        if (absError <= AutoConfig::STEER_ERROR_STOP_DEG)
            _steerActive = false;
    }
    else
    {
        if (absError >= AutoConfig::STEER_ERROR_START_DEG)
            _steerActive = true;
    }

    return _steerActive;
}

float AutoController::computeSteerPctFromOffset(
    const SystemState &sys,
    float desiredOffsetDeg,
    PidController &headingPid,
    float dtSec)
{
    const float actualOffsetDeg = computeActualSteerOffsetDeg(sys);
    const float steerErrorDeg = desiredOffsetDeg - actualOffsetDeg;

    if (!updateSteerActive(steerErrorDeg))
        return 0.0f;

    const float steerCmd =
        headingPid.update(steerErrorDeg, dtSec);

    return clampf(
        steerCmd,
        Limits::STEER_MIN_PCT,
        Limits::STEER_MAX_PCT);
}

float AutoController::computeSpeedThrustPct(
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

ActuatorCommand AutoController::update(
    float dtSec,
    SystemState &sys,
    PidController &headingPid,
    PidController &speedPid)
{
    strcpy(sys.sensors.autoState, "RUN");

    const float currentSpeedMps = sys.sensors.speedMps;

    if (!AutoConfig::BENCH_TEST_AUTO_WITHOUT_GPS &&
        !autoCanUseGpsCourse(sys))
    {
        strcpy(sys.sensors.autoState, "LOW SPD");
        return makeManualFallbackCommand(sys);
    }

    if (!AutoConfig::BENCH_TEST_AUTO_WITHOUT_GPS &&
        !sys.sensors.boatImuValid)
    {
        strcpy(sys.sensors.autoState, "NO BH");
        return makeManualFallbackCommand(sys);
    }

    const float currentCourseDeg = getAutoCourseHeadingDeg(sys);

    const float courseErrorDeg =
        shortestAngleErrorDeg(sys.targetHeadingDeg, currentCourseDeg);

    const float desiredOffsetDeg =
        computeDesiredSteerOffsetDeg(courseErrorDeg);

    ActuatorCommand out;

    out.steerPct =
        computeSteerPctFromOffset(
            sys,
            desiredOffsetDeg,
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