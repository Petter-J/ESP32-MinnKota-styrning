#include "gps_sensor.h"

bool GpsSensor::begin()
{
    _serial.begin(GpsConfig::BAUD, SERIAL_8N1, GpsConfig::RX_PIN, GpsConfig::TX_PIN);

    _gsvSatsInView.begin(_gps, "GPGSV", 3);

    return true;
}

void GpsSensor::update(GpsFix &out)
{
    while (_serial.available())
    {
        char c = _serial.read();
        _gps.encode(c);
    }

    out.locationValid =
        _gps.location.isValid() &&
        _gps.location.age() < 2000;
    out.speedValid =
        _gps.speed.isValid() &&
        _gps.speed.age() < 2000;
    out.courseValid = false;

    if (out.locationValid)
    {
        out.latDeg = _gps.location.lat();
        out.lonDeg = _gps.location.lng();
        out.locationUpdated = _gps.location.isUpdated();
    }
    else
    {
        out.locationUpdated = false;
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
        _gps.course.age() < 2000 &&
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

    if (_gps.satellites.isValid() && _gps.satellites.age() < 2000)
    {
        out.satellites = _gps.satellites.value();
    }
    else
    {
        out.satellites = 0;
    }

    if (_gsvSatsInView.isValid())
    {
        out.satellitesInView = (uint8_t)atoi(_gsvSatsInView.value());
    }
    else
    {
        out.satellitesInView = 0;
    }
}