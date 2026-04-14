#pragma once
#include <Arduino.h>


// ============================================================
// MANUAL CONTROL
// ============================================================
namespace ManualControlConfig
{
    static constexpr float THRUST_START_MIN_PCT = 20.0f;
    static constexpr float THRUST_STEP_PCT      = 5.0f;
    static constexpr uint32_t REPEAT_MS         = 120;

    static constexpr float STEER_JOG_PCT        = 100.0f;
}

// ============================================================
// AUTO CONTROL
// ============================================================
namespace AutoControlConfig
{
    static constexpr float SPEED_STEP_PCT   = 5.0f;
    static constexpr float HEADING_STEP_DEG = 5.0f;
    static constexpr uint32_t REPEAT_MS     = 250;
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
    static constexpr int STOP         = 26;
    static constexpr int MODE_MANUAL  = 25;
    static constexpr int MODE_AUTO    = 34;
    static constexpr int MODE_ANCHOR  = 39;

    static constexpr int THRUST_UP    = 36;
    static constexpr int THRUST_DOWN  = 35;

    static constexpr int STEER_LEFT   = 18;
    static constexpr int STEER_RIGHT  = 23;
}

// ============================================================
// PIN CONFIG
// ============================================================
namespace PinConfig
{
    static constexpr int THRUST_PWM = 33;   // DFR1036 PWM input
    static constexpr int THRUST_EN  = 27;   // Enable to motor board (active LOW)

    static constexpr int STEER_DIR  = 14;   // Cytron MD13S DIR
    static constexpr int STEER_PWM  = 32;   // Cytron MD13S PWM

    static constexpr int STATUS_LED = 13;
}

// ============================================================
// GPS / COMPASS CONFIG 
// ============================================================    
namespace GpsConfig
{
    // UART för GPS
    static constexpr int RX_PIN = 16;   // ESP32 RX tar emot från GPS TX
    static constexpr int TX_PIN = 17;   // ESP32 TX skickar till GPS RX
    static constexpr uint32_t BAUD = 115200;
}

namespace CompassConfig
{
    // I2C för QMC5883
    static constexpr int SDA_PIN = 21;
    static constexpr int SCL_PIN = 22;
    static constexpr uint32_t FREQ_HZ = 100000;

    // Enligt modulens orientering
    static constexpr float HEADING_OFFSET_DEG = 180.0f;
}

// ============================================================
// RAMP DEFAULTS
// ============================================================
namespace RampConfig
{
    static constexpr float THRUST_RAMP_TIME_MS = 600.0f;
    static constexpr float STEER_RAMP_TIME_MS  = 400.0f;
}
// ============================================================
// MOTOR CONFIG 
// ============================================================
namespace MotorConfig
{
    static constexpr float THRUST_MIN_START_PCT = 15.0f;
    static constexpr float STEER_MIN_START_PCT  = 10.0f;
}


// ============================================================
// PWM / LEDC CONFIG
// ============================================================
namespace PwmConfig
{
    static constexpr int RESOLUTION_BITS = 8;   // 0..255
    static constexpr int MAX_DUTY = (1 << RESOLUTION_BITS) - 1;

    static constexpr int THRUST_CHANNEL = 0;
    static constexpr int STEER_CHANNEL  = 1;

    static constexpr int THRUST_FREQ_HZ = 10000;
    static constexpr int STEER_FREQ_HZ  = 16000;
}

// ============================================================
// TIMING
// ============================================================
namespace TimingConfig
{
    static constexpr uint32_t MAIN_LOOP_INTERVAL_MS = 20;
    static constexpr uint32_t CONTROL_INTERVAL_MS   = 20;
    static constexpr uint32_t SIM_INTERVAL_MS       = 20;
    static constexpr uint32_t PRINT_INTERVAL_MS     = 500;
    static constexpr uint32_t HEARTBEAT_INTERVAL_MS = 500;
    static constexpr uint32_t FAILSAFE_TIMEOUT_MS   = 1000;
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