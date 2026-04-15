#include "navigation.h"
#include <cstring>
#include <Arduino.h>
#include <HardwareSerial.h>
#include <TinyGPSPlus.h>
#include <Wire.h>
#include "config.h"

static HardwareSerial GPSSerial(1);
static TinyGPSPlus gps;

static float gCompassHeadingDeg = 0.0f;
static bool gCompassValid = false;

static uint8_t compassFailCount = 0;
static constexpr uint8_t COMPASS_FAIL_LIMIT = 5;

static constexpr float COMPASS_OFFSET_X = -27.5f;
static constexpr float COMPASS_OFFSET_Y = -550.5f;

static constexpr float COMPASS_GAIN_X = 1.164f;
static constexpr float COMPASS_GAIN_Y = 0.876f;

static constexpr float COMPASS_CROSS_XY = 0.05f;
static constexpr float COMPASS_CROSS_YX = 0.05f;

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

static float correctHeading45(float h)
{
    h = wrap360(h);

    // Uppmätt heading -> verklig heading
    static constexpr float meas[9] = {
    0.0f,    // verklig 0
    38.5f,   // verklig 45
    82.3f,   // verklig 90
    119.0f,  // verklig 135
    153.5f,  // verklig 180
    213.0f,  // verklig 225
    289.2f,  // verklig 270
    329.1f,  // verklig 315
    360.0f   // verklig 360
};
    static constexpr float real[9] = {
        0.0f,
        45.0f,
        90.0f,
        135.0f,
        180.0f,
        225.0f,
        270.0f,
        315.0f,
        360.0f
    };

    float hw = h;
    if (hw < meas[0])
    {
        hw += 360.0f;
    }

    for (int i = 0; i < 8; ++i)
    {
        if (hw >= meas[i] && hw <= meas[i + 1])
        {
            const float span = meas[i + 1] - meas[i];
            if (span <= 0.001f)
            {
                return wrap360(real[i]);
            }

            const float t = (hw - meas[i]) / span;
            const float corrected = real[i] + t * (real[i + 1] - real[i]);
            return wrap360(corrected);
        }
    }

    return wrap360(h);
}

static bool updateCompassHeading()
{
    int16_t x = 0;
    int16_t y = 0;
    int16_t z = 0;

    if (!readCompassRaw(x, y, z))
{
    compassFailCount++;

    if (compassFailCount >= COMPASS_FAIL_LIMIT)
    {
        gCompassValid = false;
    }

    return false; // behåll senaste heading
}

    compassFailCount = 0;

    if (x < magMinX) magMinX = x;
    if (x > magMaxX) magMaxX = x;

    if (y < magMinY) magMinY = y;
    if (y > magMaxY) magMaxY = y;

    // --- Offset ---
    const float x0 = (float)x - COMPASS_OFFSET_X;
    const float y0 = (float)y - COMPASS_OFFSET_Y;

    // --- Soft iron correction matrix ---
    const float xCal = COMPASS_GAIN_X * x0 + COMPASS_CROSS_XY * y0;
    const float yCal = COMPASS_CROSS_YX * x0 + COMPASS_GAIN_Y * y0;

    float headingDeg = atan2f(yCal, xCal) * 180.0f / PI;
    headingDeg += CompassConfig::HEADING_OFFSET_DEG;
    headingDeg = wrap360(headingDeg);

    //  Lägg in correction här
    const float correctedHeading = correctHeading45(headingDeg);

    //  Lägg in smoothing här
    if (!gCompassValid)
  {
    gCompassHeadingDeg = correctedHeading;
    gCompassValid = true;
  }
   else
  {
    const float alpha = 0.25f;
    gCompassHeadingDeg = smoothAngleDeg(gCompassHeadingDeg, correctedHeading, alpha);
  }

    static uint32_t lastCompassPrintMs = 0;
    const uint32_t now = millis();

    if (now - lastCompassPrintMs >= 1000)
    {
        lastCompassPrintMs = now;

        //Serial.printf("[CAL] minX=%d maxX=%d minY=%d maxY=%d\n",magMinX, magMaxX, magMinY, magMaxY);

        Serial.printf("[COMPASS] hdg=%.1f\n", gCompassHeadingDeg);

        //Serial.printf("[RAW] hdg=%.1f\n", headingDeg);
            
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
    //    - otherwise compass
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
else if (gCompassValid)
{
    sensors.headingDeg = gCompassHeadingDeg;
    sensors.headingValid = true;
    strcpy(sensors.headingSource, "CMP");
}
else
{
    sensors.headingDeg = 0.0f;
    sensors.headingValid = false;
    strcpy(sensors.headingSource, "NONE");
}
}