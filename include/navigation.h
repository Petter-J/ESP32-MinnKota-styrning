#pragma once

#include "types.h"
#include "gps_sensor.h"
#include "imu_sensor.h"

class Navigation {
public:
    bool begin();
    void update(SensorData& sensors);

private:
    GpsSensor _gps;
    ImuSensor _imu;
};