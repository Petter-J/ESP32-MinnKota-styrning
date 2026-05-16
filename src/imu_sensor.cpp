#include "imu_sensor.h"
#include "heading_calibration.h"

ImuSensor::ImuSensor(int uartNum)
    : _serial(uartNum)
{
}

bool ImuSensor::begin(int rxPin, int txPin, uint32_t baud, float headingOffsetDeg)
{
    _headingDeg = 0.0f;
    _pitchDeg = 0.0f;
    _rollDeg = 0.0f;
    _valid = false;
    _imuFailCount = 0;
    _headingOffsetDeg = headingOffsetDeg;

    _serial.begin(baud, SERIAL_8N1, rxPin, txPin);

    if (!_rvc.begin(&_serial))
    {
        Serial.println("[IMU] BNO085 RVC not found on UART");
        return false;
    }

    Serial.println("[IMU] BNO085 RVC started on UART");
    return true;
}

bool ImuSensor::begin()
{
    return begin(
        CompassConfig::RX_PIN,
        CompassConfig::TX_PIN,
        CompassConfig::BAUD,
        CompassConfig::M_HEADING_OFFSET_DEG);
}

void ImuSensor::setHeadingOffset(float offsetDeg)
{
    _headingOffsetDeg = offsetDeg;
}

void ImuSensor::setCorrectionTable(const HeadingCorrectionPoint *table, uint8_t count)
{
    _correctionTable = table;
    _correctionCount = count;
}

float ImuSensor::correctHeading(float raw)
{
    return HeadingCalibration::apply(
        raw,
        _correctionTable,
        _correctionCount);
}

void ImuSensor::update(ImuHeading &out)
{
    BNO08x_RVC_Data rvcData;

    if (!_rvc.read(&rvcData))
    {
        _imuFailCount++;
        if (_imuFailCount >= IMU_FAIL_LIMIT)
        {
            _valid = false;
        }

        out.headingDeg = _headingDeg;
        out.pitchDeg = _pitchDeg;
        out.rollDeg = _rollDeg;
        out.valid = _valid;
        out.accuracy = _valid ? 3 : 0;
        return;
    }

    _imuFailCount = 0;

    float headingDeg = rvcData.yaw;
    headingDeg += _headingOffsetDeg;
    headingDeg = wrap360(headingDeg);
    headingDeg = correctHeading(headingDeg);

    _headingDeg = headingDeg;
    _pitchDeg = rvcData.pitch;
    _rollDeg = rvcData.roll;
    _valid = true;

    out.headingDeg = _headingDeg;
    out.pitchDeg = _pitchDeg;
    out.rollDeg = _rollDeg;
    out.valid = true;
    out.accuracy = 3;

    static uint32_t lastImuPrintMs = 0;
    const uint32_t now = millis();

    if (now - lastImuPrintMs >= 1000)
    {
        lastImuPrintMs = now;
        Serial.printf(
            "[IMU] hdg=%.1f pitch=%.1f roll=%.1f\n",
            _headingDeg,
            _pitchDeg,
            _rollDeg);
    }
}