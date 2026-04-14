#include "simulator.h"
#include "config.h"

void BoatSimulator::begin()
{
    _headingDeg = 0.0f;
    _speedPct   = 0.0f;
    _posX       = 0.0f;
    _posY       = 0.0f;
}

void BoatSimulator::update(float dtSec, const ActuatorCommand& cmd, SensorData& sensors)
{
    const float steer  = clampf(cmd.steerPct, -100.0f, 100.0f) / 100.0f;
    const float thrust = clampf(cmd.thrustPct, 0.0f, 100.0f) / 100.0f;

    const float turnRate = steer * SimConfig::MAX_TURN_RATE_DEG_PER_SEC;
    _headingDeg = wrap360(_headingDeg + turnRate * dtSec);

    const float targetSpeedPct = thrust * 100.0f;
    _speedPct += (targetSpeedPct - _speedPct) * clampf(SimConfig::SPEED_RESPONSE * dtSec, 0.0f, 1.0f);
    _speedPct = clampf(_speedPct, 0.0f, 100.0f);

    const float speedMps   = (_speedPct / 100.0f) * SimConfig::MAX_VIRTUAL_SPEED_MPS;
    const float headingRad = _headingDeg * DEG_TO_RAD;

    _posX += cosf(headingRad) * speedMps * dtSec;
    _posY += sinf(headingRad) * speedMps * dtSec;

    sensors.headingDeg   = _headingDeg;
    sensors.speedPct     = _speedPct;
    sensors.posX         = _posX;
    sensors.posY         = _posY;
    sensors.headingValid = true;
    sensors.speedValid   = true;
    sensors.gpsValid     = false;
}

void BoatSimulator::setHeading(float deg)
{
    _headingDeg = wrap360(deg);
}

void BoatSimulator::setSpeed(float pct)
{
    _speedPct = clampf(pct, 0.0f, 100.0f);
}