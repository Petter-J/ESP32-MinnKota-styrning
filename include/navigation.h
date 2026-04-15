#pragma once

#include "types.h"

class Navigation {
public:
    bool begin();
    void update(SensorData& sensors);
};