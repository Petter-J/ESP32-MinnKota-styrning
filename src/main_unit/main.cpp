#include <Arduino.h>
#include <WiFi.h>
#include "config.h"
#include "types.h"
#include "motors.h"
#include "controller.h"
#include "simulator.h"
#include "remote_espnow.h"
#include "buttons.h"
#include "input_logic.h"


// ============================================================
// Globals
// ============================================================
static bool gDisplayLinkAlive = false;
static uint32_t gStatusCounter = 0;
static bool useSimulator = true;  // sätt false när GPS/kompass är inkopplat
static SystemState gSys;
static MotorManager gMotors;
static MainController gController;
static BoatSimulator gSimulator;
static RemoteEspNow gRemote;
static ButtonManager gButtons;
static InputLogic gInputLogic;

// ============================================================
// Local button read
// ============================================================
static uint32_t readLocalButtons()
{
    uint32_t mask = 0;

    if (digitalRead(ButtonPins::STOP) == LOW)
        mask |= buttonBit(ButtonId::STOP);

    if (digitalRead(ButtonPins::MODE_MANUAL) == LOW)
        mask |= buttonBit(ButtonId::MODE_MANUAL);

    if (digitalRead(ButtonPins::MODE_AUTO) == LOW)
        mask |= buttonBit(ButtonId::MODE_AUTO);

    if (digitalRead(ButtonPins::MODE_ANCHOR) == LOW)
        mask |= buttonBit(ButtonId::MODE_ANCHOR);

    if (digitalRead(ButtonPins::THRUST_UP) == LOW)
        mask |= buttonBit(ButtonId::THRUST_UP);

    if (digitalRead(ButtonPins::THRUST_DOWN) == LOW)
        mask |= buttonBit(ButtonId::THRUST_DOWN);

    if (digitalRead(ButtonPins::STEER_LEFT) == LOW)
        mask |= buttonBit(ButtonId::STEER_LEFT);

    if (digitalRead(ButtonPins::STEER_RIGHT) == LOW)
        mask |= buttonBit(ButtonId::STEER_RIGHT);

    return mask;
}

// ============================================================
// Helpers
// ============================================================
static void printTelemetry(const SystemState& sys)
{
    DBG_PRINTF(
        "[TEL] mode=%s hdg=%.1f tgtH=%.1f tgtS=%.1f manT=%.1f manS=%.1f actT=%.1f actS=%.1f pos=(%.2f,%.2f)\n",
        modeToString(sys.mode),
        sys.sensors.headingDeg,
        sys.targetHeadingDeg,
        sys.targetSpeedPct,
        sys.manualThrustPct,
        sys.manualSteerPct,
        sys.actuators.thrustPct,
        sys.actuators.steerPct,
        sys.sensors.posX,
        sys.sensors.posY
    );
}

// ============================================================
// Setup
// ============================================================
void setup()
{
    Serial.begin(115200);
    delay(300);

    Serial.println(WiFi.macAddress());

    pinMode(PinConfig::STATUS_LED, OUTPUT);
    digitalWrite(PinConfig::STATUS_LED, LOW);

    pinMode(ButtonPins::STOP, INPUT);
    pinMode(ButtonPins::MODE_MANUAL, INPUT);
    pinMode(ButtonPins::MODE_AUTO, INPUT);
    pinMode(ButtonPins::MODE_ANCHOR, INPUT);

    pinMode(ButtonPins::THRUST_UP, INPUT);
    pinMode(ButtonPins::THRUST_DOWN, INPUT);

    pinMode(ButtonPins::STEER_LEFT, INPUT);
    pinMode(ButtonPins::STEER_RIGHT, INPUT);

    DBG_PRINTLN("");
    DBG_PRINTLN("=======================================");
    DBG_PRINTLN("ESP32 Trolling Motor Controller - Boot");
    DBG_PRINTLN("=======================================");

    gMotors.begin();
    gController.begin();
    gSimulator.begin();
    gRemote.begin();
    gButtons.begin();
    gInputLogic.begin();

    gSys.mode = SystemMode::STOP;
    gSys.motorsEnabled = true;
    gSys.simulatorEnabled = true;
    gSys.targetHeadingDeg = 0.0f;
    gSys.targetSpeedPct = 0.0f;
    gSys.manualThrustPct = 0.0f;
    gSys.manualSteerPct = 0.0f;
    gSys.actuators = {};
    gSys.lastCommandTimeMs = millis();

    DBG_PRINTLN("Buttons active, serial control removed.");
}

// ============================================================
// Main loop
// ============================================================
void loop()
{
    static uint32_t lastMainMs = 0;
    static uint32_t lastControlMs = 0;
    static uint32_t lastSimMs = 0;
    static uint32_t lastPrintMs = 0;
    static uint32_t lastHeartbeatMs = 0;
    static bool ledState = false;
    static uint32_t remoteMask = 0;

    const uint32_t now = millis();

    // Heartbeat LED
    if (now - lastHeartbeatMs >= TimingConfig::HEARTBEAT_INTERVAL_MS)
    {
        lastHeartbeatMs = now;
        ledState = !ledState;
        digitalWrite(PinConfig::STATUS_LED, ledState ? HIGH : LOW);
    }

    // Main loop pacing
    if (now - lastMainMs < TimingConfig::MAIN_LOOP_INTERVAL_MS)
    {
        return;
    }
    lastMainMs = now;

    // 1. Read local buttons
const uint32_t localMask = readLocalButtons();

// 2. Read remote buttons if new data exists
uint32_t newRemoteMask = 0;
if (gRemote.update(newRemoteMask))
{
    remoteMask = newRemoteMask;
}

// timeout
if (!gRemote.isAlive(now))
{
    remoteMask = 0;
}

// kombinera
const uint32_t effectiveMask = localMask | remoteMask;

gSys.lastCommand.buttonMask = effectiveMask;
gSys.lastCommand.valid = true;
gSys.lastCommand.timestampMs = now;

// 3. Interpret buttons centrally
const ButtonOutput btn = gButtons.update(effectiveMask, now);

// 4. Apply input policy
gInputLogic.applyButtons(btn, now, gSys, gController);

// 5. Apply mode-dependent safety
gInputLogic.applySafety(now, gSys, gController);

// 6. Control update
if (now - lastControlMs >= TimingConfig::CONTROL_INTERVAL_MS)
{
    const float dtSec = (now - lastControlMs) / 1000.0f;
    lastControlMs = now;

    gController.update(dtSec, gSys);
    gMotors.apply(gSys.actuators, gSys.motorsEnabled, dtSec);
}

// 6.5 Send status to remote
StatusPacket pkt;
pkt.mode = (uint8_t)gSys.mode;
pkt.manualThrustPct = (uint8_t)roundf(gSys.manualThrustPct);
pkt.targetSpeedPct = (uint8_t)roundf(gSys.targetSpeedPct);

pkt.headingDeg10 = (uint16_t)roundf(gSys.sensors.headingDeg * 10.0f);
pkt.targetHeadingDeg10 = (uint16_t)roundf(gSys.targetHeadingDeg * 10.0f);

pkt.satellites = (uint8_t)gSys.sensors.satellites;

pkt.flags = 0;
if (gSys.sensors.gpsValid)
{
    pkt.flags |= STATUS_FLAG_GPS_VALID;
}

pkt.counter = (uint8_t)gStatusCounter++;

gRemote.sendStatus(pkt);

    // 7. Sensor update
if (useSimulator && now - lastSimMs >= TimingConfig::SIM_INTERVAL_MS)
{
    const float dtSec = (now - lastSimMs) / 1000.0f;
    lastSimMs = now;

    gSimulator.update(dtSec, gSys.actuators, gSys.sensors);
}
else if (!useSimulator)
{
    // Här kommer GPS + kompass senare
}



    // 8. Telemetry
    if (now - lastPrintMs >= TimingConfig::PRINT_INTERVAL_MS)
    {
        lastPrintMs = now;
        printTelemetry(gSys);
    }
}