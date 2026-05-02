#pragma once
#include <Arduino.h>



// ============================================================
// MANUAL CONTROL
// ============================================================
namespace ManualControlConfig
{
    static constexpr float THRUST_START_MIN_PCT = 20.0f;
    static constexpr float THRUST_STEP_PCT = 5.0f;
    static constexpr uint32_t REPEAT_MS = 120;

    static constexpr float STEER_JOG_PCT = 100.0f; // max pwm
}

// ============================================================
// AUTO CONTROL
// ============================================================
namespace AutoControlConfig
{
    static constexpr float SPEED_STEP_PCT = 5.0f;
    static constexpr float HEADING_STEP_DEG = 5.0f;
    static constexpr uint32_t REPEAT_MS = 250;
}

// ============================================================
// ANCHOR CONTROL
// ============================================================
namespace AnchorControlConfig
{
    static constexpr float MAX_ENTRY_THRUST_PCT = 30.0f;
}

// ============================================================
// BUTTONS
// ============================================================

namespace ButtonPins
{
    // ESP32-S3 valid GPIOs (internal pull-ups, active LOW to GND)
    static constexpr int STOP         = 8;
    static constexpr int MODE_MANUAL  = 13;
    static constexpr int MODE_AUTO    = 12;
    static constexpr int MODE_ANCHOR  = 11;

    static constexpr int THRUST_UP    = 10;
    static constexpr int THRUST_DOWN  = 9;

    static constexpr int STEER_LEFT   = 6;
    static constexpr int STEER_RIGHT  = 5;
}


// ============================================================
// PIN CONFIG
// ============================================================
namespace PinConfig
{
    static constexpr int THRUST_PWM = 15;   
    static constexpr int THRUST_EN  = 16;

    static constexpr int STEER_DIR  = 17;
    static constexpr int STEER_PWM  = 18;

}

// ============================================================
// GPS / IMU CONFIG
// ============================================================
namespace GpsConfig
{
    // UART för GPS
    static constexpr int RX_PIN = 39;
    static constexpr int TX_PIN = 38;
    static constexpr uint32_t BAUD = 115200;
}

namespace CompassConfig
{
    // I2C för BNO085
    static constexpr int SDA_PIN = 3;
    static constexpr int SCL_PIN = 4;
    static constexpr uint32_t FREQ_HZ = 100000;

    static constexpr float HEADING_OFFSET_DEG = 130.0f;
}

// ============================================================
// RAMP DEFAULTS
// ============================================================
namespace RampConfig
{
    static constexpr float THRUST_RAMP_TIME_MS = 600.0f;
    static constexpr float STEER_RAMP_TIME_MS = 400.0f;
}

// ============================================================
// MOTOR CONFIG
// ============================================================
namespace MotorConfig
{
    static constexpr float THRUST_MIN_START_PCT = 15.0f;
    static constexpr float STEER_MIN_START_PCT = 10.0f;
}

// ============================================================
// AUTO CONFIG
// ============================================================
namespace AutoConfig
{
    static constexpr float MIN_GPS_COURSE_SPEED_MPS = 0.5f;
    static constexpr float START_THRUST_PCT = 20.0f;
    static constexpr float MAX_SPEED_MPS = 2.5f;
}

// ============================================================
// PWM / LEDC CONFIG
// ============================================================
namespace PwmConfig
{
    static constexpr int RESOLUTION_BITS = 8; // 0..255
    static constexpr int MAX_DUTY = (1 << RESOLUTION_BITS) - 1;

    static constexpr int THRUST_CHANNEL = 0;
    static constexpr int STEER_CHANNEL = 1;

    static constexpr int THRUST_FREQ_HZ = 10000;
    static constexpr int STEER_FREQ_HZ = 16000;
}

// ============================================================
// TIMING
// ============================================================
namespace TimingConfig
{
    static constexpr uint32_t MAIN_LOOP_INTERVAL_MS = 20;
    static constexpr uint32_t CONTROL_INTERVAL_MS = 20;
    static constexpr uint32_t SIM_INTERVAL_MS = 20;
    static constexpr uint32_t PRINT_INTERVAL_MS = 500;
    static constexpr uint32_t HEARTBEAT_INTERVAL_MS = 500;
    static constexpr uint32_t FAILSAFE_TIMEOUT_MS = 1000;
}

// ============================================================
// LIMITS
// ============================================================
namespace Limits
{
    static constexpr float THRUST_MIN_PCT = 0.0f;
    static constexpr float THRUST_MAX_PCT = 100.0f;

    static constexpr float STEER_MIN_PCT = -100.0f;
    static constexpr float STEER_MAX_PCT = 100.0f;
}

// ============================================================
// CONTROL DEFAULTS
// ============================================================
namespace ControlDefaults
{
    static constexpr float HEADING_KP = 1.2f;
    static constexpr float HEADING_KI = 0.0f;
    static constexpr float HEADING_KD = 0.08f;

    static constexpr float SPEED_KP = 1.0f;
    static constexpr float SPEED_KI = 0.0f;
    static constexpr float SPEED_KD = 0.0f;
}

// ============================================================
// SIMULATOR DEFAULTS
// ============================================================
namespace SimConfig
{
    static constexpr float MAX_TURN_RATE_DEG_PER_SEC = 100.0f;
    static constexpr float SPEED_RESPONSE = 1.5f;
    static constexpr float MAX_VIRTUAL_SPEED_MPS = 1.5f;
}

// ============================================================
// SAFETY
// ============================================================
namespace SafetyConfig
{
    static constexpr bool ENABLE_SENSOR_MODE_SAFETY = false;

    static constexpr uint32_t SENSOR_FAIL_TIMEOUT_MS = 2000;
    static constexpr uint32_t COMMAND_TIMEOUT_MS = 3000;
}

// ============================================================
// DEBUG
// ============================================================
#define ENABLE_SERIAL_DEBUG 1

#if ENABLE_SERIAL_DEBUG
#define DBG_PRINT(x) Serial.print(x)
#define DBG_PRINTLN(x) Serial.println(x)
#define DBG_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
#define DBG_PRINT(x)
#define DBG_PRINTLN(x)
#define DBG_PRINTF(...)
#endif
