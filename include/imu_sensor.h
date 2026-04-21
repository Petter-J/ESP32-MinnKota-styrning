#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include "config.h"
#include "types.h"

struct ImuHeading
{
    bool valid = false;
    float headingDeg = 0.0f;
    uint8_t accuracy = 0;
};

class ImuSensor
{
public:
    bool begin();
    void update(ImuHeading& out);

private:
    bool enableReports();
    float correctHeading(float raw);

private:
    Adafruit_BNO08x _bno08x{-1};
    sh2_SensorValue_t _sensorValue{};

    float _headingDeg = 0.0f;
    bool _valid = false;

    uint8_t _imuFailCount = 0;
    static constexpr uint8_t IMU_FAIL_LIMIT = 10;
};