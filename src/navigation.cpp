#include "navigation.h"

#include <Arduino.h>
#include <HardwareSerial.h>
#include <TinyGPSPlus.h>
#include <Wire.h>

#include "config.h"

static HardwareSerial GPSSerial(1);
static TinyGPSPlus gps;

static float gCompassHeadingDeg = 0.0f;
static bool gCompassValid = false;

static constexpr float COMPASS_OFFSET_X = -296.0f;
static constexpr float COMPASS_OFFSET_Y = -590.0f;

static constexpr float COMPASS_SCALE_X = 1.0f;
static constexpr float COMPASS_SCALE_Y = 1.230f;

static int16_t magMinX = 32767;
static int16_t magMaxX = -32768;
static int16_t magMinY = 32767;
static int16_t magMaxY = -32768;

// ------------------------------------------------------------
// Helpers
// ------------------------------------------------------------


static float smoothAngleDeg(float currentDeg, float targetDeg, float alpha)
{
    const float err = shortestAngleErrorDeg(targetDeg, currentDeg);
    return wrap360(currentDeg + err * alpha);
}

static void writeCompassRegister(uint8_t reg, uint8_t value)
{
    Wire.beginTransmission(0x0D); // QMC5883 address
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

static bool readCompassRaw(int16_t& x, int16_t& y, int16_t& z)
{
    Wire.beginTransmission(0x0D);
    Wire.write(0x00); // data register start

    if (Wire.endTransmission(false) != 0)
    {
        return false;
    }

    const uint8_t bytesRequested = 6;
    const uint8_t bytesRead = Wire.requestFrom((uint8_t)0x0D, bytesRequested);

    if (bytesRead != bytesRequested)
    {
        return false;
    }

    x = (int16_t)(Wire.read() | (Wire.read() << 8));
    y = (int16_t)(Wire.read() | (Wire.read() << 8));
    z = (int16_t)(Wire.read() | (Wire.read() << 8));

    return true;
}

static bool updateCompassHeading()
{
    int16_t x = 0;
    int16_t y = 0;
    int16_t z = 0;

    if (!readCompassRaw(x, y, z))
    {
        return false;
    }

    if (x < magMinX) magMinX = x;
    if (x > magMaxX) magMaxX = x;
  
    if (y < magMinY) magMinY = y;
    if (y > magMaxY) magMaxY = y;

    // Enkel första version:
    // heading från X/Y, offset från config
    const float xCal = ((float)x - COMPASS_OFFSET_X) * COMPASS_SCALE_X;
    const float yCal = ((float)y - COMPASS_OFFSET_Y) * COMPASS_SCALE_Y;

    float headingDeg = atan2f(yCal, xCal) * 180.0f / PI;
headingDeg -= 90.0f;
headingDeg = wrap360(headingDeg);

    headingDeg += CompassConfig::HEADING_OFFSET_DEG;
    headingDeg = wrap360(headingDeg);

    gCompassHeadingDeg = headingDeg;
    gCompassValid = true;

    static uint32_t lastCompassPrintMs = 0;
    const uint32_t now = millis();
    if (now - lastCompassPrintMs >= 1000)
    {
        lastCompassPrintMs = now;

   // Serial.printf(
    //"[CAL] minX=%d maxX=%d minY=%d maxY=%d\n",
    //magMinX, magMaxX, magMinY, magMaxY
    //);

    Serial.printf("[TEST] x=%.1f y=%.1f hdg=%.1f\n", xCal, yCal, gCompassHeadingDeg);

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

    // QMC5883 init
    writeCompassRegister(0x0B, 0x01); // reset
    writeCompassRegister(0x09, 0x1D); // continuous mode, 200Hz ODR, 8G, OSR 512
    writeCompassRegister(0x0A, 0x00); // normal set/reset

    Serial.println("[NAV] GPS UART started");
    Serial.println("[NAV] Compass I2C started");

    gCompassHeadingDeg = 0.0f;
    gCompassValid = false;

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
    // 2. Update compass
    // --------------------------------------------------------
    updateCompassHeading();

    // --------------------------------------------------------
    // 3. Fill GPS-related sensor fields
    // --------------------------------------------------------
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
    //    - otherwise compass
    // --------------------------------------------------------
    const float minHeadingSpeedMps = 0.8f;

    if (gps.course.isValid() &&
        gps.speed.isValid() &&
        gps.speed.mps() >= minHeadingSpeedMps)
    {
        sensors.headingDeg = sensors.courseOverGroundDeg;
        sensors.headingValid = true;
    }
    else if (gCompassValid)
    {
        sensors.headingDeg = gCompassHeadingDeg;
        sensors.headingValid = true;
    }
    else
    {
        sensors.headingDeg = 0.0f;
        sensors.headingValid = false;
    }
}