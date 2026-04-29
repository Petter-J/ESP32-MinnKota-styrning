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
#include "calibration_manager.h"

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
static CalibrationManager gCalibration;
static uint32_t gCalibrationCommandId = 1;

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

static void startCalibrationClockwise()
{
    gCalibrationCommandId++;

    gCalibration.startClockwise(gCalibrationCommandId);

    CalStartSweepPacket pkt;
    pkt.phase = static_cast<uint8_t>(RemoteCalPhase::Clockwise);
    pkt.bucketCount = HEADING_CAL_BUCKET_COUNT;
    pkt.bucketWindowDeg = HEADING_CAL_BUCKET_WINDOW_DEG;
    pkt.minSamplesPerBucket = HEADING_CAL_MIN_SAMPLES_PER_BUCKET;
    pkt.commandId = gCalibrationCommandId;

    // Byt detta mot din befintliga send-funktion om du har en wrapper
    gRemote.sendCalibrationPacket(
        reinterpret_cast<const uint8_t *>(&pkt),
        sizeof(pkt));

    Serial.println("[CAL] Started clockwise sweep");
}

static void startCalibrationCounterClockwise()
{
    gCalibrationCommandId++;

    gCalibration.startCounterClockwise(gCalibrationCommandId);

    CalStartSweepPacket pkt;
    pkt.phase = static_cast<uint8_t>(RemoteCalPhase::CounterClockwise);
    pkt.bucketCount = HEADING_CAL_BUCKET_COUNT;
    pkt.bucketWindowDeg = HEADING_CAL_BUCKET_WINDOW_DEG;
    pkt.minSamplesPerBucket = HEADING_CAL_MIN_SAMPLES_PER_BUCKET;
    pkt.commandId = gCalibrationCommandId;

    gRemote.sendCalibrationPacket(
        reinterpret_cast<const uint8_t *>(&pkt),
        sizeof(pkt));

    Serial.println("[CAL] Started counter-clockwise sweep");
}

static void sendBoatLutToRemote()
{
    const HeadingCalPoint *points = gCalibration.finalPoints();

    for (uint8_t i = 0; i < HEADING_CAL_BUCKET_COUNT; i++)
    {
        const HeadingCalPoint &p = points[i];

        CalSaveBoatLutPointPacket pkt;
        pkt.lutIndex = i;
        pkt.rawDeg = p.boatRawDeg;
        pkt.corrDeg = p.gpsDeg;
        pkt.valid = p.valid;
        pkt.commandId = gCalibrationCommandId;

        gRemote.sendCalibrationPacket(
            reinterpret_cast<const uint8_t *>(&pkt),
            sizeof(pkt));

        delay(10);

        Serial.printf("[CAL] Sent boat LUT %u raw=%.1f corr=%.1f valid=%d\n",
                      i,
                      pkt.rawDeg,
                      pkt.corrDeg,
                      pkt.valid ? 1 : 0);
    }

    CalEndSweepPacket endPkt;
    endPkt.saveToFlash = false;
    endPkt.commandId = gCalibrationCommandId;

    gRemote.sendCalibrationPacket(
        reinterpret_cast<const uint8_t *>(&endPkt),
        sizeof(endPkt));

    Serial.println("[CAL] Boat LUT sent to remote");
}

// ============================================================
// Helpers
// ============================================================
static void printTelemetry(const SystemState &sys)
{
    DBG_PRINTF(
        "[TEL] mode=%s auto=%s hdg=%.1f hdgSrc=%s hdgValid=%d boat=%.1f boatV=%d motor=%.1f motorV=%d mAng=%.1f gpsValid=%d spdValid=%d sats=%u lat=%.6f lon=%.6f gpsSpd=%.2f speedMps=%.2f cog=%.1f spdPct=%.1f tgtH=%.1f tgtS=%.1f actT=%.1f actS=%.1f\n",
        modeToString(sys.mode),
        sys.sensors.autoState,
        sys.sensors.headingDeg,
        sys.sensors.headingSource,
        sys.sensors.headingValid ? 1 : 0,
        sys.sensors.boatHeadingDeg,
        sys.sensors.boatImuValid ? 1 : 0,
        sys.sensors.motorHeadingDeg,
        sys.sensors.motorImuValid ? 1 : 0,
        sys.sensors.motorAngleDeg,
        sys.sensors.gpsValid ? 1 : 0,
        sys.sensors.speedValid ? 1 : 0,
        sys.sensors.satellites,
        sys.sensors.latitudeDeg,
        sys.sensors.longitudeDeg,
        sys.sensors.gpsSpeedMps,
        sys.sensors.speedMps, 
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
    gCalibration.begin();
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

    float remoteBoatHeadingDeg = 0.0f;

    if (gRemote.getBoatHeading(remoteBoatHeadingDeg, now))
    {
        gSys.sensors.boatHeadingDeg = remoteBoatHeadingDeg;
        gSys.sensors.boatImuValid = true;
    }
    else
    {
        gSys.sensors.boatImuValid = false;
    }

        // Main loop pacing
    if (now - lastMainMs < TimingConfig::MAIN_LOOP_INTERVAL_MS)
    {
        return;
    }
    lastMainMs = now;

    // 0. Update sensors first
    gNavigation.update(gSys.sensors);

    // 0.5 Calibration sweep update
    gCalibration.update(
        gSys.sensors.courseOverGroundDeg,
        gSys.sensors.gpsSpeedMps,
        gSys.sensors.motorHeadingDeg);

    if (gCalibration.hasPendingBucketSample())
    {
        CalBucketSamplePacket calPkt =
            gCalibration.takePendingBucketSample();

        gRemote.sendCalibrationPacket(
            reinterpret_cast<const uint8_t *>(&calPkt),
            sizeof(calPkt));
    }

    CalBoatBucketResultPacket boatPkt;
    if (gRemote.getBoatCalibrationResult(boatPkt))
    {
        gCalibration.handleBoatBucketResult(boatPkt);
    }

    static bool cwDoneHandled = false;
    static bool finalLutSent = false;

    if (gCalibration.isClockwiseComplete() && !cwDoneHandled)
    {
        cwDoneHandled = true;

        Serial.println("[CAL] CW complete. Start CCW sweep.");
        startCalibrationCounterClockwise();
    }

    if (gCalibration.isComplete() && !finalLutSent)
    {
        finalLutSent = true;

        Serial.println("[CAL] Calibration complete. Sending boat LUT.");
        sendBoatLutToRemote();
    }

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

    if (btn.stopRequested && gCalibration.active())
    {
        Serial.println("[CAL] Cancelled by STOP");

        gCalibration.stop();

        CalEndSweepPacket endPkt;
        endPkt.saveToFlash = false;
        endPkt.commandId = gCalibrationCommandId;

        gRemote.sendCalibrationPacket(
            reinterpret_cast<const uint8_t *>(&endPkt),
            sizeof(endPkt));

        cwDoneHandled = false;
        finalLutSent = false;
    }

    if (btn.requestCalibration && !gCalibration.active())
    {
        cwDoneHandled = false;
        finalLutSent = false;

        Serial.println("[CAL] ButtonManager start");
        startCalibrationClockwise();
    }

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
    pkt.gpsSpeedCmps = (uint16_t)roundf(gSys.sensors.gpsSpeedMps * 100.0f);
    pkt.gpsCogDeg10 = (uint16_t)roundf(gSys.sensors.courseOverGroundDeg * 10.0f);

    if (gSys.sensors.boatImuValid)
    {
        pkt.headingDeg10 = (uint16_t)roundf(gSys.sensors.boatHeadingDeg * 10.0f);
    }
    else
    {
        pkt.headingDeg10 = (uint16_t)roundf(gSys.sensors.headingDeg * 10.0f);
    }

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

    pkt.calFlags = 0;

    if (gCalibration.active())
    {
        pkt.calFlags |= STATUS_CAL_FLAG_ACTIVE;
    }

    if (gCalibration.isComplete())
    {
        pkt.calFlags |= STATUS_CAL_FLAG_COMPLETE;
    }

    pkt.calBucketMask = gCalibration.mainBucketsValidMask();

    if (gCalibration.active())
    {
        if (!gCalibration.isClockwiseComplete())
        {
            pkt.calPhase = static_cast<uint8_t>(RemoteCalPhase::Clockwise);
        }
        else
        {
            pkt.calPhase = static_cast<uint8_t>(RemoteCalPhase::CounterClockwise);
        }
    }
    else
    {
        pkt.calPhase = static_cast<uint8_t>(RemoteCalPhase::None);
    }

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


    // 11. Telemetry
    if (now - lastPrintMs >= TimingConfig::PRINT_INTERVAL_MS)
    {
        lastPrintMs = now;
        printTelemetry(gSys);
    }
}