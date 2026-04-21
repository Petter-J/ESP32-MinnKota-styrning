#pragma once
#include <Arduino.h>
#include "remote_protocol.h"



enum class SystemMode : uint8_t
{
    STOP = 0,
    MANUAL,
    AUTO,
    ANCHOR
};

enum class ControlSource : uint8_t
{
    NONE = 0,
    SERIAL_SIM,
    REMOTE_ESPNOW
};

struct RemoteCommand
{
    bool valid = false;

    bool requestManual = false;
    bool requestAuto   = false;
    bool requestAnchor = false;

    bool hasManualThrust  = false;
    bool hasManualSteer   = false;
    bool hasTargetHeading = false;
    bool hasTargetSpeed   = false;
    bool hasAnchorHere    = false;

    float manualThrustPct = 0.0f;   // 0..100
    float manualSteerPct  = 0.0f;   // -100..100

    float targetHeadingDeg = 0.0f;  // 0..360
    float targetSpeedPct   = 0.0f;  // 0..100

    bool anchorHere = false;

    uint32_t buttonMask = 0;

    uint32_t timestampMs = 0;
};


struct SensorData
{
    float headingDeg = 0.0f;
    float speedPct   = 0.0f;

    float posX = 0.0f;
    float posY = 0.0f;

    // GPS / navigation
    double latitudeDeg  = 0.0;
    double longitudeDeg = 0.0;

    float gpsSpeedMps = 0.0f;
    float courseOverGroundDeg = 0.0f;

    int satellites = 0;

    bool headingValid = true;
    bool speedValid   = true;
    bool gpsValid     = false;

    char headingSource[5] = "NONE";
    char autoState[10] = "NONE";
};

struct ActuatorCommand
{
    float thrustPct = 0.0f;  // 0..100
    float steerPct  = 0.0f;  // -100..100
};

struct SystemState
{
    SystemMode mode = SystemMode::MANUAL;
    ControlSource lastControlSource = ControlSource::NONE;

    RemoteCommand lastCommand;
    SensorData sensors;
    ActuatorCommand actuators;

    float targetHeadingDeg = 0.0f;
    float targetSpeedPct   = 0.0f;

    float manualThrustPct = 0.0f;
    float manualSteerPct  = 0.0f;

    bool motorsEnabled = true;
    bool simulatorEnabled = true;

    uint32_t lastCommandTimeMs = 0;
    uint32_t sensorFailStartMs = 0;
};

// ---- Utility functions ----
inline float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

inline float wrap360(float deg)
{
    while (deg < 0.0f) deg += 360.0f;
    while (deg >= 360.0f) deg -= 360.0f;
    return deg;
}

inline float shortestAngleErrorDeg(float targetDeg, float currentDeg)
{
    float err = wrap360(targetDeg) - wrap360(currentDeg);
    while (err > 180.0f) err -= 360.0f;
    while (err < -180.0f) err += 360.0f;
    return err;
}

inline const char* modeToString(SystemMode mode)
{
    switch (mode)
    {
        case SystemMode::STOP:   return "STOP";
        case SystemMode::MANUAL: return "MANUAL";
        case SystemMode::AUTO:   return "AUTO";
        case SystemMode::ANCHOR: return "ANCHOR";
        default:                 return "UNKNOWN";
    }
}