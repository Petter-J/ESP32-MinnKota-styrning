#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include "config.h"
#include "types.h"

struct HeadingCorrectionPoint
{
    float raw;
    float corr;
};

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
    bool begin(int sdaPin, int sclPin, uint32_t freqHz, float headingOffsetDeg);

    void update(ImuHeading &out);

    void setHeadingOffset(float offsetDeg);
    void setCorrectionTable(const HeadingCorrectionPoint *table, uint8_t count);

private:
    bool enableReports();
    float correctHeading(float raw);

private:
    Adafruit_BNO08x _bno08x{-1};
    sh2_SensorValue_t _sensorValue{};

    float _headingDeg = 0.0f;
    bool _valid = false;

    float _headingOffsetDeg = 0.0f;
    const HeadingCorrectionPoint *_correctionTable = nullptr;
    uint8_t _correctionCount = 0;

    uint8_t _imuFailCount = 0;
    static constexpr uint8_t IMU_FAIL_LIMIT = 10;
};