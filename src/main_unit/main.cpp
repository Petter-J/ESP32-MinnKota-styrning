#include <Arduino.h>
#include <WiFi.h>
#include "config.h"
#include "types.h"
#include "motors.h"
#include "controller.h"
#include "remote_espnow.h"
#include "buttons.h"
#include "input_logic.h"
#include "navigation.h"
#include "ota_update.h"
#include "calibration_manager.h"
#include <cstring>

// ============================================================
// Globals
// ============================================================
static bool gDisplayLinkAlive = false;
static uint32_t gStatusCounter = 0;
static bool useSimulator = false; // sätt false när GPS/kompass är inkopplat
static SystemState gSys;
static MotorManager gMotors;
static MainController gController;
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

    if (ButtonPins::STOP >= 0 && digitalRead(ButtonPins::STOP) == LOW)
        mask |= buttonBit(ButtonId::STOP);

    if (ButtonPins::MODE_MANUAL >= 0 && digitalRead(ButtonPins::MODE_MANUAL) == LOW)
        mask |= buttonBit(ButtonId::MODE_MANUAL);

    if (ButtonPins::MODE_AUTO >= 0 && digitalRead(ButtonPins::MODE_AUTO) == LOW)
        mask |= buttonBit(ButtonId::MODE_AUTO);

    if (ButtonPins::MODE_ANCHOR >= 0 && digitalRead(ButtonPins::MODE_ANCHOR) == LOW)
        mask |= buttonBit(ButtonId::MODE_ANCHOR);

    if (ButtonPins::THRUST_UP >= 0 && digitalRead(ButtonPins::THRUST_UP) == LOW)
        mask |= buttonBit(ButtonId::THRUST_UP);

    if (ButtonPins::THRUST_DOWN >= 0 && digitalRead(ButtonPins::THRUST_DOWN) == LOW)
        mask |= buttonBit(ButtonId::THRUST_DOWN);

    if (ButtonPins::STEER_LEFT >= 0 && digitalRead(ButtonPins::STEER_LEFT) == LOW)
        mask |= buttonBit(ButtonId::STEER_LEFT);

    if (ButtonPins::STEER_RIGHT >= 0 && digitalRead(ButtonPins::STEER_RIGHT) == LOW)
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
    delay(500);

    if (ButtonPins::STOP >= 0)
        pinMode(ButtonPins::STOP, INPUT_PULLUP);

    const uint32_t bootStartMs = millis();
    bool forceOta = false;

    while (millis() - bootStartMs < 5000)
    {
        if (ButtonPins::STOP >= 0 && digitalRead(ButtonPins::STOP) == LOW)
        {
            forceOta = true;
            break;
        }

        delay(10);
    }

    if (forceOta)
    {

        ota_begin();

        while (true)
        {
            ota_handle();
            delay(10);
        }
    }

    if (ButtonPins::MODE_MANUAL >= 0)
        pinMode(ButtonPins::MODE_MANUAL, INPUT_PULLUP);

    if (ButtonPins::MODE_AUTO >= 0)
        pinMode(ButtonPins::MODE_AUTO, INPUT_PULLUP);

    if (ButtonPins::MODE_ANCHOR >= 0)
        pinMode(ButtonPins::MODE_ANCHOR, INPUT_PULLUP);

    if (ButtonPins::THRUST_UP >= 0)
        pinMode(ButtonPins::THRUST_UP, INPUT_PULLUP);

    if (ButtonPins::THRUST_DOWN >= 0)
        pinMode(ButtonPins::THRUST_DOWN, INPUT_PULLUP);

    if (ButtonPins::STEER_LEFT >= 0)
        pinMode(ButtonPins::STEER_LEFT, INPUT_PULLUP);

    if (ButtonPins::STEER_RIGHT >= 0)
        pinMode(ButtonPins::STEER_RIGHT, INPUT_PULLUP);

    DBG_PRINTLN("");
    DBG_PRINTLN("=======================================");
    DBG_PRINTLN("ESP32 Trolling Motor Controller - Boot");
    DBG_PRINTLN("=======================================");

    // init safe state först
    gSys.mode = SystemMode::STOP;
    gSys.motorsEnabled = true;
    gSys.targetHeadingDeg = 0.0f;
    gSys.targetSpeedPct = 0.0f;
    gSys.manualThrustPct = 0.0f;
    gSys.manualSteerPct = 0.0f;
    gSys.actuators = {};
    gSys.lastCommandTimeMs = millis();

    // sedan begin
    gMotors.begin();
    gController.begin();
    gRemote.begin();
    gButtons.begin();
    gInputLogic.begin();
    gNavigation.begin();
    gCalibration.begin();

    DBG_PRINTLN("Buttons active, serial control removed.");
}

// ============================================================
// Main loop
// ============================================================
void loop()
{
    static uint32_t lastMainMs = 0;
    static uint32_t lastControlMs = 0;
    static uint32_t lastPrintMs = 0;

    const uint32_t now = millis();

    // Main loop pacing
    if (now - lastMainMs < TimingConfig::MAIN_LOOP_INTERVAL_MS)
    {
        return;
    }
    lastMainMs = now;

    // 1. Read local buttons
    const uint32_t localMask = readLocalButtons();

    // Emergency OTA: local STOP held 5 sec
    static uint32_t earlyOtaStopHoldStartMs = 0;
    static bool earlyOtaTriggered = false;

    const bool earlyLocalStopHeld =
        (localMask & buttonBit(ButtonId::STOP)) != 0;

    if (earlyLocalStopHeld)
    {
        if (earlyOtaStopHoldStartMs == 0)
        {
            earlyOtaStopHoldStartMs = now;
        }

        if (!earlyOtaTriggered && (now - earlyOtaStopHoldStartMs) >= 5000)
        {
            earlyOtaTriggered = true;
            ota_begin();
        }
    }
    else
    {
        earlyOtaStopHoldStartMs = 0;
        earlyOtaTriggered = false;
    }

    ota_handle();

    // 2. Read remotes
    const uint32_t rawRemoteMask = gRemote.getCombinedMask(now);

    static uint32_t remoteMaskFiltered = 0;
    static uint32_t lastRemoteNonZeroMs = 0;

    if (rawRemoteMask != 0)
    {
        remoteMaskFiltered = rawRemoteMask;
        lastRemoteNonZeroMs = now;
    }
    else
    {
        if (now - lastRemoteNonZeroMs > 30)
        {
            remoteMaskFiltered = 0;
        }
    }

    const uint32_t remoteMask = remoteMaskFiltered;

    const uint32_t lastRx = gRemote.lastRxTimeMs();

    const uint32_t rxAge =
        (lastRx > 0)
            ? (now - lastRx)
            : 999999;

    if (lastRx > 0 && rxAge < 1000)
    {
        gSys.lastCommandTimeMs = now;
    }

    // 3. Combine inputs
    const uint32_t effectiveMask = localMask | remoteMask;

    gSys.lastCommand.buttonMask = effectiveMask;
    gSys.lastCommand.valid = true;
    gSys.lastCommand.timestampMs = now;


    // 4. Boat heading from Remote1
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

    // 5. Navigation: GPS + local MH + fusion
    gNavigation.update(gSys.sensors);

    // 6. Interpret buttons AFTER fresh sensors
    const ButtonOutput btn = gButtons.update(effectiveMask, now);

    // 7. Apply input policy AFTER navigation
    gInputLogic.applyButtons(btn, now, gSys, gController);

    // . Calibration sweep update
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

    // 9. Safety AFTER fresh sensors
    gInputLogic.applySafety(now, gSys, gController);

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

    // 10. Control update
    if (now - lastControlMs >= TimingConfig::CONTROL_INTERVAL_MS)
    {
        const float dtSec = (now - lastControlMs) / 1000.0f;
        lastControlMs = now;

        gController.update(dtSec, gSys);
        gMotors.apply(gSys.actuators, gSys.motorsEnabled, dtSec);
    }

    // 11. Send status to remotes
    StatusPacket pkt;
    pkt.mode = (uint8_t)gSys.mode;

    pkt.motorTiltUnsafe = gSys.sensors.motorTiltUnsafe ? 1 : 0;

    pkt.manualThrustPct = (uint8_t)roundf(gSys.manualThrustPct);
    pkt.targetSpeedPct = (uint8_t)roundf(gSys.targetSpeedPct);
    pkt.gpsSpeedCmps = (uint16_t)roundf(gSys.sensors.gpsSpeedMps * 100.0f);
    pkt.targetSpeedCmps = (uint16_t)roundf(gSys.targetSpeedMps * 100.0f);
    pkt.gpsCogDeg10 = (uint16_t)roundf(gSys.sensors.courseOverGroundDeg * 10.0f);

    pkt.boatHeadingDeg10 =
        (uint16_t)roundf(gSys.sensors.boatHeadingDeg * 10.0f);

    if (gSys.sensors.motorImuValid)
    {
        pkt.motorHeadingDeg10 =
            (uint16_t)roundf(gSys.sensors.motorHeadingDeg * 10.0f);
    }
    else
    {
        pkt.motorHeadingDeg10 = 0;
    }

    pkt.targetHeadingDeg10 =
        (uint16_t)roundf(gSys.targetHeadingDeg * 10.0f);

    pkt.satellites = (uint8_t)gSys.sensors.satellites;
    pkt.satellitesInView = (uint8_t)gSys.sensors.satellitesInView;

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

    if (ota_is_active())
    {
        pkt.flags |= STATUS_FLAG_OTA_ACTIVE;
    }

    if (gSys.mode != SystemMode::ANCHOR)
    {
        pkt.counter = 0;
    }
    else if (strcmp(gSys.sensors.autoState, "A_WAIT") == 0)
        pkt.counter = 1;
    else if (strcmp(gSys.sensors.autoState, "A_GPSAVG") == 0)
        pkt.counter = 2;
    else if (strcmp(gSys.sensors.autoState, "L_HOLD") == 0)
        pkt.counter = 3;
    else if (strcmp(gSys.sensors.autoState, "L_DRIFT") == 0)
        pkt.counter = 4;
    else if (strcmp(gSys.sensors.autoState, "LEARN_RET") == 0)
        pkt.counter = 5;
    else if (strcmp(gSys.sensors.autoState, "HOLD") == 0)
        pkt.counter = 6;
    else if (strcmp(gSys.sensors.autoState, "DRIFT") == 0)
        pkt.counter = 7;
    else if (strcmp(gSys.sensors.autoState, "RETURN") == 0)
        pkt.counter = 8;
    else if (strcmp(gSys.sensors.autoState, "M_HOLD") == 0)
        pkt.counter = 9;
    else if (strcmp(gSys.sensors.autoState, "MAINTAIN") == 0)
        pkt.counter = 10;
    else if (strcmp(gSys.sensors.autoState, "M_RETURN") == 0)
        pkt.counter = 11;
    else
        pkt.counter = 0;

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

    static uint32_t lastStatusR1Ms = 0;
    static uint32_t lastStatusR2Ms = 0;

    if (now - lastStatusR1Ms >= 50)
    {
        lastStatusR1Ms = now;

        StatusPacket pkt1 = pkt;
        gRemote.sendStatusRemote1(pkt1);
    }

    if (now - lastStatusR2Ms >= 70)
    {
        lastStatusR2Ms = now;

        StatusPacket pkt2 = pkt;
        gRemote.sendStatusRemote2(pkt2);
    }

    // 12. Telemetry
    if (now - lastPrintMs >= TimingConfig::PRINT_INTERVAL_MS)
    {
        lastPrintMs = now;
        printTelemetry(gSys);
    }
}