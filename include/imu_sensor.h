#pragma once
#include <Arduino.h>
#include <HardwareSerial.h>
#include <Adafruit_BNO08x_RVC.h>
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
    float pitchDeg = 0.0f;
    float rollDeg = 0.0f;
    uint8_t accuracy = 0;
};

class ImuSensor
{
public:
    explicit ImuSensor(int uartNum = 2);

    bool begin();
    bool begin(int rxPin, int txPin, uint32_t baud, float headingOffsetDeg);

    void update(ImuHeading &out);

    void setHeadingOffset(float offsetDeg);
    void setCorrectionTable(const HeadingCorrectionPoint *table, uint8_t count);

private:
    float correctHeading(float raw);

private:
    HardwareSerial _serial;
    Adafruit_BNO08x_RVC _rvc;

    float _headingDeg = 0.0f;
    float _pitchDeg = 0.0f;
    float _rollDeg = 0.0f;
    bool _valid = false;

    float _headingOffsetDeg = 0.0f;
    const HeadingCorrectionPoint *_correctionTable = nullptr;
    uint8_t _correctionCount = 0;

    uint8_t _imuFailCount = 0;
    static constexpr uint8_t IMU_FAIL_LIMIT = 10;
};