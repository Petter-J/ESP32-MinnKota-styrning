#include "navigation.h"
#include <cstring>
#include <Arduino.h>
#include <HardwareSerial.h>
#include <TinyGPSPlus.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include "config.h"

static HardwareSerial GPSSerial(1);
static TinyGPSPlus gps;

// ------------------------------------------------------------
// BNO085
// ------------------------------------------------------------
static Adafruit_BNO08x bno08x(-1);   // ingen reset-pin använd
static sh2_SensorValue_t sensorValue;

static float gImuHeadingDeg = 0.0f;
static bool gImuValid = false;

static uint8_t imuFailCount = 0;
static constexpr uint8_t IMU_FAIL_LIMIT = 10;

// ------------------------------------------------------------
// Helpers
// ------------------------------------------------------------
static float smoothAngleDeg(float currentDeg, float targetDeg, float alpha)
{
    const float err = shortestAngleErrorDeg(targetDeg, currentDeg);
    return wrap360(currentDeg + err * alpha);
}

static bool enableImuReports()
{
    // Rotation vector = fusionerad orientering
    if (!bno08x.enableReport(SH2_ROTATION_VECTOR))
    {
        Serial.println("[IMU] Could not enable rotation vector");
        return false;
    }

    return true;
}

static bool updateImuHeading()
{
    if (bno08x.wasReset())
    {
        Serial.println("[IMU] Sensor reset detected, re-enabling reports");
        if (!enableImuReports())
        {
            gImuValid = false;
            return false;
        }
    }

    bool gotRotationVector = false;

    while (bno08x.getSensorEvent(&sensorValue))
    {
        if (sensorValue.sensorId == SH2_ROTATION_VECTOR)
        {
            gotRotationVector = true;

            const float qi = sensorValue.un.rotationVector.i;
            const float qj = sensorValue.un.rotationVector.j;
            const float qk = sensorValue.un.rotationVector.k;
            const float qr = sensorValue.un.rotationVector.real;

            // Quaternion -> yaw
            float yawRad = atan2f(
                2.0f * (qr * qk + qi * qj),
                1.0f - 2.0f * (qj * qj + qk * qk)
            );

            float headingDeg = yawRad * 180.0f / PI;
            headingDeg += CompassConfig::HEADING_OFFSET_DEG;
            headingDeg = wrap360(headingDeg);

            gImuHeadingDeg = headingDeg;
            gImuValid = true;
        }
    }

    if (!gotRotationVector)
    {
        imuFailCount++;
        if (imuFailCount >= IMU_FAIL_LIMIT)
        {
            gImuValid = false;
        }
        return false;
    }

    imuFailCount = 0;

    static uint32_t lastImuPrintMs = 0;
    const uint32_t now = millis();

    if (now - lastImuPrintMs >= 1000)
    {
        lastImuPrintMs = now;
        Serial.printf("[IMU] hdg=%.1f acc=%u\n", gImuHeadingDeg, sensorValue.status);
    }

    return true;
}

// ------------------------------------------------------------
// Navigation
// ------------------------------------------------------------
bool Navigation::begin()
{
    GPSSerial.begin(
        GpsConfig::BAUD,
        SERIAL_8N1,
        GpsConfig::RX_PIN,
        GpsConfig::TX_PIN
    );

    Wire.begin(CompassConfig::SDA_PIN, CompassConfig::SCL_PIN);
    Wire.setClock(CompassConfig::FREQ_HZ);

    Serial.println("[NAV] GPS UART started");

    gImuHeadingDeg = 0.0f;
    gImuValid = false;
    imuFailCount = 0;

    if (!bno08x.begin_I2C())
    {
        Serial.println("[NAV] BNO085 not found on I2C");
    }
    else
    {
        Serial.println("[NAV] BNO085 found");

        if (enableImuReports())
        {
            Serial.println("[NAV] IMU reports enabled");
        }
        else
        {
            Serial.println("[NAV] Failed to enable IMU reports");
        }
    }

    return true;
}

void Navigation::update(SensorData& sensors)
{
    // --------------------------------------------------------
    // 1. Read GPS stream
    // --------------------------------------------------------
    while (GPSSerial.available())
    {
        gps.encode((char)GPSSerial.read());
    }

    // --------------------------------------------------------
    // 2. Update IMU
    // --------------------------------------------------------
    updateImuHeading();

    // --------------------------------------------------------
    // 3. Fill GPS-related sensor fields
    // --------------------------------------------------------
    strcpy(sensors.headingSource, "NONE");

    sensors.gpsValid = gps.location.isValid();
    sensors.speedValid = gps.speed.isValid();

    if (gps.location.isValid())
    {
        sensors.latitudeDeg = gps.location.lat();
        sensors.longitudeDeg = gps.location.lng();
    }

    if (gps.satellites.isValid())
    {
        sensors.satellites = gps.satellites.value();
    }
    else
    {
        sensors.satellites = 0;
    }

    if (gps.speed.isValid())
    {
        sensors.gpsSpeedMps = gps.speed.mps();
    }
    else
    {
        sensors.gpsSpeedMps = 0.0f;
    }

    if (gps.course.isValid())
    {
        sensors.courseOverGroundDeg = gps.course.deg();
    }
    else
    {
        sensors.courseOverGroundDeg = 0.0f;
    }

    // --------------------------------------------------------
    // 4. Map GPS speed -> speedPct
    // --------------------------------------------------------
    const float maxSpeedMps = 2.5f;
    float pct = (sensors.gpsSpeedMps / maxSpeedMps) * 100.0f;

    if (pct < 0.0f) pct = 0.0f;
    if (pct > 100.0f) pct = 100.0f;

    sensors.speedPct = pct;

    // --------------------------------------------------------
    // 5. Select heading source
    //    - GPS course when moving fast enough
    //    - otherwise IMU
    // --------------------------------------------------------
    const float minHeadingSpeedMps = 0.8f;

    if (gps.course.isValid() &&
        gps.speed.isValid() &&
        gps.speed.mps() >= minHeadingSpeedMps)
    {
        sensors.headingDeg = sensors.courseOverGroundDeg;
        sensors.headingValid = true;
        strcpy(sensors.headingSource, "GPS");
    }
    else if (gImuValid)
    {
        sensors.headingDeg = gImuHeadingDeg;
        sensors.headingValid = true;
        strcpy(sensors.headingSource, "IMU");
    }
    else
    {
        sensors.headingDeg = 0.0f;
        sensors.headingValid = false;
        strcpy(sensors.headingSource, "NONE");
    }
}