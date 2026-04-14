#include "motors.h"
#include "config.h"

// ============================================================
// ThrustMotor
// ============================================================

void ThrustMotor::begin()
{
    pinMode(PinConfig::THRUST_EN, OUTPUT);
    digitalWrite(PinConfig::THRUST_EN, HIGH); // active LOW => OFF vid boot

    ledcSetup(PwmConfig::THRUST_CHANNEL, PwmConfig::THRUST_FREQ_HZ, PwmConfig::RESOLUTION_BITS);
    ledcAttachPin(PinConfig::THRUST_PWM, PwmConfig::THRUST_CHANNEL);

    ledcWrite(PwmConfig::THRUST_CHANNEL, 0);
    _pct = 0.0f;
}

void ThrustMotor::setPercent(float pct)
{
    _pct = clampf(pct, Limits::THRUST_MIN_PCT, Limits::THRUST_MAX_PCT);

    float scaledPct = _pct;

    if (scaledPct > 0.1f)
    {
        scaledPct = MotorConfig::THRUST_MIN_START_PCT +
                    (scaledPct / 100.0f) * (100.0f - MotorConfig::THRUST_MIN_START_PCT);
    }
    else
    {
        scaledPct = 0.0f;
    }

    const int duty = (int)((scaledPct / 100.0f) * PwmConfig::MAX_DUTY);

    if (_pct > 0.1f)
    {
        ledcWrite(PwmConfig::THRUST_CHANNEL, duty);
        digitalWrite(PinConfig::THRUST_EN, LOW);   // ON
    }
    else
    {
        digitalWrite(PinConfig::THRUST_EN, HIGH);  // OFF
        ledcWrite(PwmConfig::THRUST_CHANNEL, 0);
    }
}

float ThrustMotor::getPercent() const
{
    return _pct;
}

void ThrustMotor::stop()
{
    _pct = 0.0f;
    digitalWrite(PinConfig::THRUST_EN, HIGH); // OFF
    ledcWrite(PwmConfig::THRUST_CHANNEL, 0);
}

// ============================================================
// SteeringMotor
// ============================================================

void SteeringMotor::begin()
{
    pinMode(PinConfig::STEER_DIR, OUTPUT);
    digitalWrite(PinConfig::STEER_DIR, LOW);

    ledcSetup(PwmConfig::STEER_CHANNEL, PwmConfig::STEER_FREQ_HZ, PwmConfig::RESOLUTION_BITS);
    ledcAttachPin(PinConfig::STEER_PWM, PwmConfig::STEER_CHANNEL);

    ledcWrite(PwmConfig::STEER_CHANNEL, 0);
    _pct = 0.0f;
}

void SteeringMotor::setPercent(float pct)
{
    _pct = clampf(pct, Limits::STEER_MIN_PCT, Limits::STEER_MAX_PCT);

    const bool dir = (_pct >= 0.0f);
    digitalWrite(PinConfig::STEER_DIR, dir ? HIGH : LOW);

    float mag = fabsf(_pct);
    float scaledMag = mag;

    if (scaledMag > 0.1f)
    {
        scaledMag = MotorConfig::STEER_MIN_START_PCT +
                    (scaledMag / 100.0f) * (100.0f - MotorConfig::STEER_MIN_START_PCT);
    }
    else
    {
        scaledMag = 0.0f;
    }

    const int duty = (int)((scaledMag / 100.0f) * PwmConfig::MAX_DUTY);
    ledcWrite(PwmConfig::STEER_CHANNEL, duty);
}

float SteeringMotor::getPercent() const
{
    return _pct;
}

void SteeringMotor::stop()
{
    _pct = 0.0f;
    ledcWrite(PwmConfig::STEER_CHANNEL, 0);
}

// ============================================================
// MotorManager
// ============================================================

void MotorManager::begin()
{
    _thrust.begin();
    _steer.begin();

    _thrustFiltered = 0.0f;
    _steerFiltered  = 0.0f;
}

void MotorManager::apply(const ActuatorCommand& cmd, bool motorsEnabled, float dtSec)
{
    if (!motorsEnabled)
    {
        stopAll();
        return;
    }

    if (dtSec <= 0.0f)
    {
        return;
    }

    // ----------------------------
    // Thrust ramp
    // ----------------------------
    const float thrustRatePerSec =
        (RampConfig::THRUST_RAMP_TIME_MS > 0.0f)
            ? (100.0f / (RampConfig::THRUST_RAMP_TIME_MS / 1000.0f))
            : 1000.0f;

    const float thrustMaxStep = thrustRatePerSec * dtSec;
    float thrustDiff = cmd.thrustPct - _thrustFiltered;

    if (thrustDiff > thrustMaxStep) thrustDiff = thrustMaxStep;
    if (thrustDiff < -thrustMaxStep) thrustDiff = -thrustMaxStep;

    _thrustFiltered += thrustDiff;
    _thrustFiltered = clampf(_thrustFiltered, Limits::THRUST_MIN_PCT, Limits::THRUST_MAX_PCT);

    _thrust.setPercent(_thrustFiltered);

    // ----------------------------
    // Steering ramp
    // ----------------------------
    const float steerRatePerSec =
        (RampConfig::STEER_RAMP_TIME_MS > 0.0f)
            ? (100.0f / (RampConfig::STEER_RAMP_TIME_MS / 1000.0f))
            : 1000.0f;

    const float steerMaxStep = steerRatePerSec * dtSec;
    float steerDiff = cmd.steerPct - _steerFiltered;

    if (steerDiff > steerMaxStep) steerDiff = steerMaxStep;
    if (steerDiff < -steerMaxStep) steerDiff = -steerMaxStep;

    _steerFiltered += steerDiff;
    _steerFiltered = clampf(_steerFiltered, Limits::STEER_MIN_PCT, Limits::STEER_MAX_PCT);

    _steer.setPercent(_steerFiltered);
}

void MotorManager::stopAll()
{
    _thrust.stop();
    _steer.stop();

    _thrustFiltered = 0.0f;
    _steerFiltered  = 0.0f;
}

float MotorManager::getThrustPercent() const
{
    return _thrust.getPercent();
}

float MotorManager::getSteerPercent() const
{
    return _steer.getPercent();
}