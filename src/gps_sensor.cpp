#include "gps_sensor.h"

bool GpsSensor::begin()
{
    _serial.begin(GpsConfig::BAUD, SERIAL_8N1, GpsConfig::RX_PIN, GpsConfig::TX_PIN);
    Serial.println("[GPS] UART started");
    return true;
}

void GpsSensor::update(GpsFix &out)
{
    while (_serial.available())
    {
        char c = _serial.read();
        _gps.encode(c);
    }

    out.locationValid = _gps.location.isValid();
    out.speedValid = _gps.speed.isValid();
    out.courseValid = false;

    if (out.locationValid)
    {
        out.latDeg = _gps.location.lat();
        out.lonDeg = _gps.location.lng();
    }

    if (out.speedValid)
    {
        out.speedMps = _gps.speed.mps();
    }
    else
    {
        out.speedMps = 0.0f;
    }

    if (_gps.course.isValid() &&
        out.speedValid &&
        out.speedMps >= AutoConfig::MIN_GPS_COURSE_SPEED_MPS)
    {
        const float course = _gps.course.deg();

        if (course >= 0.0f && course <= 360.0f)
        {
            out.courseDeg = course;
            out.courseValid = true;
        }
        else
        {
            out.courseDeg = 0.0f;
            out.courseValid = false;
        }
    }
    else
    {
        out.courseDeg = 0.0f;
        out.courseValid = false;
    }

    if (_gps.satellites.isValid())
    {
        out.satellites = _gps.satellites.value();
    }
    else
    {
        out.satellites = 0;
    }
}