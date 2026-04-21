#include "gps_sensor.h"

bool GpsSensor::begin()
{
    _serial.begin(GpsConfig::BAUD, SERIAL_8N1, GpsConfig::RX_PIN, GpsConfig::TX_PIN);
    Serial.println("[GPS] UART started");
    return true;
}

void GpsSensor::update(GpsFix& out)
{
    while (_serial.available())
    {
        char c = _serial.read();
        _gps.encode(c);
    }

    out.locationValid = _gps.location.isValid();
    out.speedValid = _gps.speed.isValid();
    out.courseValid = _gps.course.isValid();

    if (out.locationValid)
    {
        out.latDeg = _gps.location.lat();
        out.lonDeg = _gps.location.lng();
    }

    if (out.speedValid)
    {
        out.speedMps = _gps.speed.mps();
    }

    if (out.courseValid)
    {
        out.courseDeg = _gps.course.deg();
    }

    out.satellites = _gps.satellites.value();
}