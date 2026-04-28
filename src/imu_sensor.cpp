#include "imu_sensor.h"

bool ImuSensor::begin(int sdaPin, int sclPin, uint32_t freqHz, float headingOffsetDeg)
{
    Wire.begin(sdaPin, sclPin);
    Wire.setClock(freqHz);

    _headingDeg = 0.0f;
    _valid = false;
    _imuFailCount = 0;
    _headingOffsetDeg = headingOffsetDeg;

    if (!_bno08x.begin_I2C())
    {
        Serial.println("[NAV] BNO085 not found on I2C");
        return false;
    }

    Serial.println("[NAV] BNO085 found");

    if (enableReports())
    {
        Serial.println("[NAV] IMU reports enabled");
        return true;
    }

    Serial.println("[NAV] Failed to enable IMU reports");
    return false;
}

bool ImuSensor::begin()
{
    Wire.begin(CompassConfig::SDA_PIN, CompassConfig::SCL_PIN);
    Wire.setClock(CompassConfig::FREQ_HZ);

    _headingDeg = 0.0f;
    // Default single-IMU config (kan senare ersättas med boat/motor-specifik config)
    _headingOffsetDeg = CompassConfig::HEADING_OFFSET_DEG;
    _valid = false;
    _imuFailCount = 0;

    if (!_bno08x.begin_I2C())
    {
        Serial.println("[NAV] BNO085 not found on I2C");
        return false;
    }

    Serial.println("[NAV] BNO085 found");

    if (enableReports())
    {
        Serial.println("[NAV] IMU reports enabled");
        return true;
    }

    Serial.println("[NAV] Failed to enable IMU reports");
    return false;
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

bool ImuSensor::enableReports()
{
    if (!_bno08x.enableReport(SH2_GEOMAGNETIC_ROTATION_VECTOR))
    {
        Serial.println("[IMU] Could not enable geomagnetic rotation vector");
        return false;
    }

    return true;
}

float ImuSensor::correctHeading(float raw)
{
    static const HeadingCorrectionPoint defaultTable[] =
        {
            {0, 0},
            {37, 45},
            {83, 90},
            {128, 135},
            {163, 180},
            {205, 225},
            {275, 270},
            {325, 315},
            {360, 360}};

    const HeadingCorrectionPoint *table = _correctionTable;
    uint8_t count = _correctionCount;

    if (table == nullptr || count < 2)
    {
        table = defaultTable;
        count = sizeof(defaultTable) / sizeof(defaultTable[0]);
    }

    for (uint8_t i = 0; i < count - 1; i++)
    {
        const float r0 = table[i].raw;
        const float r1 = table[i + 1].raw;

        if (raw >= r0 && raw <= r1)
        {
            const float t = (raw - r0) / (r1 - r0);
            const float c0 = table[i].corr;
            const float c1 = table[i + 1].corr;

            return wrap360(c0 + t * (c1 - c0));
        }
    }

    return raw;
}

void ImuSensor::update(ImuHeading& out)
{
    if (_bno08x.wasReset())
    {
        Serial.println("[IMU] Sensor reset detected, re-enabling reports");
        if (!enableReports())
        {
            _valid = false;
            out.valid = false;
            return;
        }
    }

    bool gotRotationVector = false;

    while (_bno08x.getSensorEvent(&_sensorValue))
    {
        if (_sensorValue.sensorId == SH2_GEOMAGNETIC_ROTATION_VECTOR)
        {
            gotRotationVector = true;

            const float qi = _sensorValue.un.rotationVector.i;
            const float qj = _sensorValue.un.rotationVector.j;
            const float qk = _sensorValue.un.rotationVector.k;
            const float qr = _sensorValue.un.rotationVector.real;

            float yawRad = atan2f(
                2.0f * (qr * qk + qi * qj),
                1.0f - 2.0f * (qj * qj + qk * qk)
            );

            float headingDeg = yawRad * 180.0f / PI;
            headingDeg += _headingOffsetDeg;
            headingDeg = wrap360(headingDeg);
            headingDeg = correctHeading(headingDeg);

            _headingDeg = headingDeg;
            _valid = true;

            out.headingDeg = _headingDeg;
            out.valid = true;
            out.accuracy = _sensorValue.status;
        }
    }

    if (!gotRotationVector)
    {
        _imuFailCount++;
        if (_imuFailCount >= IMU_FAIL_LIMIT)
        {
            _valid = false;
        }

        out.headingDeg = _headingDeg;
        out.valid = _valid;
        out.accuracy = _sensorValue.status;
        return;
    }

    _imuFailCount = 0;

    static uint32_t lastImuPrintMs = 0;
    const uint32_t now = millis();

    if (now - lastImuPrintMs >= 1000)
    {
        lastImuPrintMs = now;
        Serial.printf("[IMU] hdg=%.1f acc=%u\n", _headingDeg, _sensorValue.status);
    }

    out.headingDeg = _headingDeg;
    out.valid = _valid;
    out.accuracy = _sensorValue.status;
}