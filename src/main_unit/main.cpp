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
#include "navigation.h"
#include "ota_update.h"

// ============================================================
// Globals
// ============================================================
static bool gDisplayLinkAlive = false;
static uint32_t gStatusCounter = 0;
static bool useSimulator = false; // sätt false när GPS/kompass är inkopplat
static SystemState gSys;
static MotorManager gMotors;
static MainController gController;
static BoatSimulator gSimulator;
static RemoteEspNow gRemote;
static ButtonManager gButtons;
static InputLogic gInputLogic;
static Navigation gNavigation;

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
static void printTelemetry(const SystemState &sys)
{
    DBG_PRINTF(
        "[TEL] mode=%s auto=%s hdg=%.1f hdgSrc=%s hdgValid=%d gpsValid=%d spdValid=%d sats=%u lat=%.6f lon=%.6f  gpsSpd=%.2f speedMps=%.2f cog=%.1f spdPct=%.1f tgtH=%.1f tgtS=%.1f actT=%.1f actS=%.1f\n",
        modeToString(sys.mode),
        sys.sensors.autoState,
        sys.sensors.headingDeg,
        sys.sensors.headingSource,
        sys.sensors.headingValid ? 1 : 0,
        sys.sensors.gpsValid ? 1 : 0,
        sys.sensors.speedValid ? 1 : 0,
        sys.sensors.satellites,
        sys.sensors.latitudeDeg,
        sys.sensors.longitudeDeg,
        sys.sensors.gpsSpeedMps,
        sys.sensors.speedMps, // 🔹 här
        sys.sensors.courseOverGroundDeg,
        sys.sensors.speedPct,
        sys.targetHeadingDeg,
        sys.targetSpeedPct,
        sys.actuators.thrustPct,
        sys.actuators.steerPct);
}
// ============================================================
// Setup
// ============================================================
void setup()
{
    Serial.begin(115200);
delay(1500);

for (int i = 0; i < 5; i++)
{
    Serial.print("MAIN MAC: ");
    Serial.println(WiFi.macAddress());
    delay(500);
}

    pinMode(ButtonPins::STOP, INPUT_PULLUP);
    pinMode(ButtonPins::MODE_MANUAL, INPUT_PULLUP);
    pinMode(ButtonPins::MODE_AUTO, INPUT_PULLUP);
    pinMode(ButtonPins::MODE_ANCHOR, INPUT_PULLUP);

    pinMode(ButtonPins::THRUST_UP, INPUT_PULLUP);
    pinMode(ButtonPins::THRUST_DOWN, INPUT_PULLUP);

    pinMode(ButtonPins::STEER_LEFT, INPUT_PULLUP);
    pinMode(ButtonPins::STEER_RIGHT, INPUT_PULLUP);

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
    gNavigation.begin();
    ota_begin();

    gSys.mode = SystemMode::STOP;
    gSys.motorsEnabled = true;
    gSys.simulatorEnabled = false;
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

    const uint32_t now = millis();

    ota_handle();

        // Main loop pacing
    if (now - lastMainMs < TimingConfig::MAIN_LOOP_INTERVAL_MS)
    {
        return;
    }
    lastMainMs = now;

    // 0. Update sensors first
    gNavigation.update(gSys.sensors);

    // 1. Read local buttons
    const uint32_t localMask = readLocalButtons();

    // 2. Read ALL remotes (combined inside RemoteEspNow)
    const uint32_t remoteMask = gRemote.getCombinedMask(now);

    const uint32_t lastRx = gRemote.lastRxTimeMs();

    const uint32_t rxAge =
        (lastRx > 0 && now >= lastRx)
            ? (now - lastRx)
            : 999999;

    if (lastRx > 0 && rxAge < 500)
    {
        gSys.lastCommandTimeMs = now;
    }

    // link/command heartbeat time
   // gSys.lastCommandTimeMs = gRemote.lastRxTimeMs();

    

    // 3. Combine all inputs
    const uint32_t effectiveMask = localMask | remoteMask;

    if (effectiveMask != 0)
    {
        DBG_PRINTF("[BTN] local=0x%08lx remote=0x%08lx effective=0x%08lx\n",
                   localMask, remoteMask, effectiveMask);
    }

    // 4. Store command
    gSys.lastCommand.buttonMask = effectiveMask;
    gSys.lastCommand.valid = true;
    gSys.lastCommand.timestampMs = now;

    // 5. Interpret buttons
    const ButtonOutput btn = gButtons.update(effectiveMask, now);

    // 6. Apply input policy
    gInputLogic.applyButtons(btn, now, gSys, gController);

    // 7. Apply safety
    gInputLogic.applySafety(now, gSys, gController);

    // 8. Control update
    if (now - lastControlMs >= TimingConfig::CONTROL_INTERVAL_MS)
    {
        const float dtSec = (now - lastControlMs) / 1000.0f;
        lastControlMs = now;

        gController.update(dtSec, gSys);
        gMotors.apply(gSys.actuators, gSys.motorsEnabled, dtSec);
    }

    // 9. Send status to remotes
    StatusPacket pkt;
    pkt.mode = (uint8_t)gSys.mode;
    pkt.manualThrustPct = (uint8_t)roundf(gSys.manualThrustPct);
    pkt.targetSpeedPct = (uint8_t)roundf(gSys.targetSpeedPct);

    pkt.headingDeg10 = (uint16_t)roundf(gSys.sensors.headingDeg * 10.0f);
    pkt.targetHeadingDeg10 = (uint16_t)roundf(gSys.targetHeadingDeg * 10.0f);

    pkt.satellites = (uint8_t)gSys.sensors.satellites;

    // 🔥 STEER baserat på faktisk motorstyrning

    if (gSys.actuators.steerPct < -1.0f)
    {
        pkt.steerState = -1;
    }
    else if (gSys.actuators.steerPct > 1.0f)
    {
        pkt.steerState = 1;
    }
    else
    {
        pkt.steerState = 0;
    }

    pkt.flags = 0;
    if (gSys.sensors.gpsValid)
    {
        pkt.flags |= STATUS_FLAG_GPS_VALID;
    }

    pkt.counter = (uint8_t)gStatusCounter++;

    // Skicka status med olika takt per remote (lätt att ändra senare)
    static uint32_t lastStatusR1Ms = 0;
    static uint32_t lastStatusR2Ms = 0;

    if (now - lastStatusR1Ms >= 50) // Remote1: 10 Hz
    {
        lastStatusR1Ms = now;

        StatusPacket pkt1 = pkt; // framtid: anpassa pkt1 för remote1
        gRemote.sendStatusRemote1(pkt1);
    }

    if (now - lastStatusR2Ms >= 50) // Remote2: 20 Hz
    {
        lastStatusR2Ms = now;

        StatusPacket pkt2 = pkt; // framtid: anpassa pkt2 för remote2
        gRemote.sendStatusRemote2(pkt2);
    }

    // 10. Sensor update
    if (useSimulator && now - lastSimMs >= TimingConfig::SIM_INTERVAL_MS)
    {
        const float dtSec = (now - lastSimMs) / 1000.0f;
        lastSimMs = now;

        gSimulator.update(dtSec, gSys.actuators, gSys.sensors);
    }
    else if (!useSimulator)
    {
       // gNavigation.update(gSys.sensors);
    }

    // 11. Telemetry
    if (now - lastPrintMs >= TimingConfig::PRINT_INTERVAL_MS)
    {
        lastPrintMs = now;
        printTelemetry(gSys);
    }
}