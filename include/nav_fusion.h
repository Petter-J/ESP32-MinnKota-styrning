#pragma once

#include "types.h"
#include "gps_sensor.h"
#include "imu_sensor.h"

class NavFusion
{
public:
    void update(const GpsFix &gps, const ImuHeading &imu, SensorData &out);
};