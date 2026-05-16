#pragma once
#include <Arduino.h>

// ============================================================
// MANUAL CONTROL
// ============================================================
namespace ManualControlConfig
{
    static constexpr float THRUST_START_MIN_PCT = 20.0f;
    static constexpr float THRUST_STEP_PCT = 5.0f;
    static constexpr uint32_t REPEAT_MS = 100;

    static constexpr float STEER_JOG_PCT = 60.0f; // max pwm
}

// ============================================================
// AUTO CONTROL
// ============================================================
namespace AutoControlConfig
{
    static constexpr float SPEED_STEP_PCT = 5.0f;
    static constexpr float HEADING_STEP_DEG = 5.0f;
    static constexpr uint32_t REPEAT_MS = 250;
    static constexpr float SPEED_STEP_KN = 0.1f;

}

// ============================================================
// ANCHOR CONTROL
// ============================================================
namespace AnchorControlConfig
{
    static constexpr float MAX_ENTRY_THRUST_PCT = 30.0f;
}

namespace AnchorConfig
{
    // --------------------------------------------------------
    // Radius logic
    // --------------------------------------------------------
    static constexpr float STOP_RADIUS_M = 1.0f;        // Motor OFF innanför denna
    static constexpr float START_RADIUS_M = 2.0f;       // Normal ON
    static constexpr float LEARN_START_RADIUS_M = 2.5f; // Learning ON

    // --------------------------------------------------------
    // Steering
    // --------------------------------------------------------
    static constexpr float HEADING_DEADBAND_DEG = 3.0f;

    // --------------------------------------------------------
    // Base thrust
    // --------------------------------------------------------
    static constexpr float START_THRUST_PCT = 5.0f;
    static constexpr float MIN_THRUST_PCT = 2.0f;
    static constexpr float MAX_THRUST_PCT = 25.0f;

    // Distans där full thrust används
    static constexpr float FULL_THRUST_DIST_M = 8.0f;

    // --------------------------------------------------------
    // Learning mode
    // --------------------------------------------------------
    // Antal driftmätningar innan mode väljs
    static constexpr uint8_t DRIFT_LEARN_SAMPLES = 5;

    // Önskad tid från STOP_RADIUS -> LEARN_START_RADIUS
    static constexpr uint32_t TARGET_DRIFT_TIME_MS = 10000;

    // Dödzon så thrust inte ändras för små variationer
    static constexpr uint32_t DRIFT_TIME_DEADBAND_MS = 2000;

    // --------------------------------------------------------
    // Return thrust learning
    // --------------------------------------------------------
    // Önskad tid från START_RADIUS -> STOP_RADIUS
    static constexpr uint32_t TARGET_RETURN_TIME_MS = 12000;

    // Dödzon för return thrust-justering
    static constexpr uint32_t RETURN_TIME_DEADBAND_MS = 2000;

    // Hur mycket thrust justeras per steg
    static constexpr float THRUST_ADJUST_STEP_PCT = 1.0f;

    // --------------------------------------------------------
    // Maintenance mode
    // --------------------------------------------------------
    // % av learned thrust i maintenance-zon
    static constexpr float MAINTENANCE_FACTOR = 0.35f;

    // Begränsningar för maintenance thrust
    static constexpr float MIN_MAINTENANCE_THRUST_PCT = 2.0f;
    static constexpr float MAX_MAINTENANCE_THRUST_PCT = 10.0f;

    // --------------------------------------------------------
    // Drift line maintenance
    // --------------------------------------------------------
    static constexpr float BASE_THRUST_ADJUST_STEP_PCT = 1.0f;
    static constexpr float CORRECTION_THRUST_OFFSET_PCT = 5.0f;
    static constexpr uint32_t BASE_THRUST_ADJUST_INTERVAL_MS = 5000;

    // Positiv riktning = samma riktning som båten brukar driva ut i learning
    static constexpr float DRIFT_BACK_ZONE_M = 1.5f;

    // Negativ sida om ankarpunkten, används som "stopplinje"
    static constexpr float DRIFT_FRONT_STOP_LINE_M = -0.5f;
}
// ============================================================
// BUTTONS
// ============================================================

namespace ButtonPins
{
    // ESP32-S3 valid GPIOs (internal pull-ups, active LOW to GND)
    static constexpr int STOP = 8;
    static constexpr int MODE_MANUAL = -1;
    static constexpr int MODE_AUTO = -1;
    static constexpr int MODE_ANCHOR = -1;

    static constexpr int THRUST_UP = -1;
    static constexpr int THRUST_DOWN = -1;

    static constexpr int STEER_LEFT = -1;
    static constexpr int STEER_RIGHT = -1;
}

// ============================================================
// PIN CONFIG
// ============================================================
namespace PinConfig
{
    static constexpr int THRUST_PWM = 15;
    static constexpr int THRUST_EN = 16;

    static constexpr int STEER_DIR = 17;
    static constexpr int STEER_PWM = 18;

}

// ============================================================
// GPS / IMU CONFIG
// ============================================================
namespace GpsConfig
{
    // UART för GPS
    static constexpr int RX_PIN = 38;
    static constexpr int TX_PIN = 39;
    static constexpr uint32_t BAUD = 115200;
}

namespace CompassConfig
{
    // UART-RVC för MOTOR BNO085
    static constexpr int RX_PIN = 3; // välj rätt pin på main
    static constexpr int TX_PIN = 4; // eller riktig TX-pin om du använder den
    static constexpr uint32_t BAUD = 115200;

    static constexpr float M_HEADING_OFFSET_DEG = 46.0f;
    static constexpr uint32_t MOTOR_HEADING_HOLD_MS = 400;

}

namespace BoatCompassConfig
{
    // UART-RVC för BOAT BNO085
    static constexpr int RX_PIN = 3; // ESP RX <- BNO TX
    static constexpr int TX_PIN = 4; // oftast ej använd i RVC
    static constexpr uint32_t BAUD = 115200;

    static constexpr float B_HEADING_OFFSET_DEG = 0.0f;
    static constexpr uint32_t BOAT_HEADING_HOLD_MS = 400;
}

// ============================================================
// RAMP DEFAULTS
// ============================================================
namespace RampConfig
{
    static constexpr float THRUST_RAMP_TIME_MS = 400.0f;
    static constexpr float STEER_RAMP_TIME_MS = 200.0f;
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
    static constexpr float MIN_GPS_COURSE_SPEED_MPS = 0.3f;
    static constexpr float START_THRUST_PCT = 20.0f;
    static constexpr float MAX_SPEED_MPS = 2.5f;
    static constexpr bool BENCH_TEST_AUTO_WITHOUT_GPS = false;
    static constexpr float COG_FILTER_ALPHA = 0.2f;
    static constexpr float COG_MAX_JUMP_DEG = 25.0f;
    static constexpr float MAX_STEER_OFFSET_DEG = 15.0f;
    static constexpr float STEER_ERROR_START_DEG = 0.8f;
    static constexpr float STEER_ERROR_STOP_DEG = 0.4f;
    static constexpr float AUTO_STEER_EXTRA_DEG = 2.0f;
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
    // Main ESP
    static constexpr uint32_t MAIN_LOOP_INTERVAL_MS = 10;

    // Remote1 (boat IMU + buttons)
    static constexpr uint32_t REMOTE1_LOOP_INTERVAL_MS = 10;

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

namespace MotorTiltSafetyConfig
{
    static constexpr bool ENABLED = true;

    static constexpr float STOP_TILT_DEG = 60.0f;
    static constexpr float RECOVER_TILT_DEG = 30.0f;
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
