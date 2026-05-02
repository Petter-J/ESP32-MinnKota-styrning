#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <cstring>
#include "imu_sensor.h"
#include "remote_protocol.h"
#include "display.h"
#include "remote_calibration.h"

// ============================================================
// BUTTON IDS
// ============================================================
enum class ButtonId : uint8_t
{
    STOP = 0,
    MODE_MANUAL,
    MODE_AUTO,
    MODE_ANCHOR,
    THRUST_UP,
    THRUST_DOWN,
    STEER_LEFT,
    STEER_RIGHT
};

constexpr uint32_t buttonBit(ButtonId id)
{
    return (1UL << static_cast<uint8_t>(id));
}

// ============================================================
// PIN CONFIG
// ============================================================
namespace RemoteButtonPins
{
    static constexpr int STOP         = 18;
    static constexpr int MODE_MANUAL  = 17;
    static constexpr int MODE_AUTO    = 16;
    static constexpr int MODE_ANCHOR  = 15;

    static constexpr int THRUST_UP    = 5;
    static constexpr int THRUST_DOWN  = 6;

    static constexpr int STEER_LEFT   = 9;
    static constexpr int STEER_RIGHT  = 10;
}

// ============================================================
// RECEIVER MAC (ändra vid behov)
// ============================================================
static uint8_t RECEIVER_MAC[6] = {0xB4, 0x3A, 0x45, 0xB9, 0xE1, 0x6C};

// ============================================================
// STATUS
// ============================================================
static StatusPacket gStatus;
static bool gHasStatus = false;
static uint32_t gLastStatusMs = 0;

static ImuSensor gBoatImu;
static ImuHeading gBoatHeading;
static bool gBoatImuStarted = false;

static RemoteCalibration gRemoteCalibration;

// ============================================================
// BUTTON READ
// ============================================================
static uint32_t readButtons()
{
    uint32_t mask = 0;

    if (digitalRead(RemoteButtonPins::STOP) == LOW)
        mask |= buttonBit(ButtonId::STOP);

    if (digitalRead(RemoteButtonPins::MODE_MANUAL) == LOW)
        mask |= buttonBit(ButtonId::MODE_MANUAL);

    if (digitalRead(RemoteButtonPins::MODE_AUTO) == LOW)
        mask |= buttonBit(ButtonId::MODE_AUTO);

    if (digitalRead(RemoteButtonPins::MODE_ANCHOR) == LOW)
        mask |= buttonBit(ButtonId::MODE_ANCHOR);

    if (digitalRead(RemoteButtonPins::THRUST_UP) == LOW)
        mask |= buttonBit(ButtonId::THRUST_UP);

    if (digitalRead(RemoteButtonPins::THRUST_DOWN) == LOW)
        mask |= buttonBit(ButtonId::THRUST_DOWN);

    if (digitalRead(RemoteButtonPins::STEER_LEFT) == LOW)
        mask |= buttonBit(ButtonId::STEER_LEFT);

    if (digitalRead(RemoteButtonPins::STEER_RIGHT) == LOW)
        mask |= buttonBit(ButtonId::STEER_RIGHT);

    return mask;
}

// ============================================================
// CALLBACKS
// ============================================================
void onSent(const uint8_t*, esp_now_send_status_t) {}

void onRecv(const uint8_t *, const uint8_t *data, int len)
{
    if (!data || len <= 0)
        return;

    // StatusPacket från Main
    if (len == (int)sizeof(StatusPacket))
    {
        memcpy(&gStatus, data, sizeof(StatusPacket));
        gHasStatus = true;
        gLastStatusMs = millis();
        return;
    }

    // Kalibreringspaket från Main
    const uint8_t msgType = data[0];

    if (msgType == static_cast<uint8_t>(RemoteMsgType::CalStartSweep) &&
        len == (int)sizeof(CalStartSweepPacket))
    {
        CalStartSweepPacket packet;
        memcpy(&packet, data, sizeof(packet));
        gRemoteCalibration.startSweep(packet);
        return;
    }

    if (msgType == static_cast<uint8_t>(RemoteMsgType::CalBucketSample) &&
        len == (int)sizeof(CalBucketSamplePacket))
    {
        CalBucketSamplePacket packet;
        memcpy(&packet, data, sizeof(packet));

        if (gBoatHeading.valid)
        {
            gRemoteCalibration.addBucketSample(packet, gBoatHeading.headingDeg);
        }

        return;
    }

    if (msgType == static_cast<uint8_t>(RemoteMsgType::CalSaveBoatLutPoint) &&
        len == (int)sizeof(CalSaveBoatLutPointPacket))
    {
        CalSaveBoatLutPointPacket packet;
        memcpy(&packet, data, sizeof(packet));
        gRemoteCalibration.saveBoatLutPoint(packet);
        return;
    }

    if (msgType == static_cast<uint8_t>(RemoteMsgType::CalEndSweep) &&
        len == (int)sizeof(CalEndSweepPacket))
    {
        CalEndSweepPacket packet;
        memcpy(&packet, data, sizeof(packet));
        gRemoteCalibration.endSweep(packet);
        return;
    }
}

// ============================================================
// SETUP
// ============================================================
void setup()
{
    Serial.begin(115200);
    delay(1500);

    //  Visa MAC-adress (viktigt för pairing)
    WiFi.mode(WIFI_STA);
    Serial.print("REMOTE MAC: ");
    Serial.println(WiFi.macAddress());

    // Pins
    pinMode(RemoteButtonPins::STOP, INPUT_PULLUP);
    pinMode(RemoteButtonPins::MODE_MANUAL, INPUT_PULLUP);
    pinMode(RemoteButtonPins::MODE_AUTO, INPUT_PULLUP);
    pinMode(RemoteButtonPins::MODE_ANCHOR, INPUT_PULLUP);

    pinMode(RemoteButtonPins::THRUST_UP, INPUT_PULLUP);
    pinMode(RemoteButtonPins::THRUST_DOWN, INPUT_PULLUP);

    pinMode(RemoteButtonPins::STEER_LEFT, INPUT_PULLUP);
    pinMode(RemoteButtonPins::STEER_RIGHT, INPUT_PULLUP);

    // Display
    display_begin();

    gBoatImuStarted = gBoatImu.begin(3, 4, 100000, 0.0f);

    if (gBoatImuStarted)
    {
        Serial.println("[REMOTE] Boat IMU started");
    }
    else
    {
        Serial.println("[REMOTE] Boat IMU not found");
    }

    // ESP-NOW
    if (esp_now_init() != ESP_OK)
    {
        Serial.println("ESP-NOW init failed");
        return;
    }

    esp_now_register_send_cb(onSent);
    esp_now_register_recv_cb(onRecv);

    esp_now_peer_info_t peer{};
    memcpy(peer.peer_addr, RECEIVER_MAC, 6);
    peer.channel = 0;
    peer.encrypt = false;

    esp_now_add_peer(&peer);

    gRemoteCalibration.begin();
}

// ============================================================
// LOOP
// ============================================================
void loop()
{
    static uint32_t lastSendMs = 0;
    static uint32_t lastSentMask = 0;

    const uint32_t now = millis();
    const uint32_t buttonMask = readButtons();

    if (gBoatImuStarted)
    {
        gBoatImu.update(gBoatHeading);
    }

    if (gRemoteCalibration.hasPendingBucketResult())
    {
        CalBoatBucketResultPacket result =
            gRemoteCalibration.takePendingBucketResult();

        esp_now_send(RECEIVER_MAC,
                     reinterpret_cast<const uint8_t *>(&result),
                     sizeof(result));
    }

    static uint32_t lastBoatPrintMs = 0;
    if (now - lastBoatPrintMs >= 1000)
    {
        lastBoatPrintMs = now;
        Serial.printf("[BOAT IMU] valid=%d hdg=%.1f acc=%u\n",
                      gBoatHeading.valid ? 1 : 0,
                      gBoatHeading.headingDeg,
                      gBoatHeading.accuracy);
    }

    // Skicka knappar
    
    const bool changed = (buttonMask != lastSentMask);
    const bool heartbeat = (now - lastSendMs >= 50);

    if (changed || heartbeat)
    {
        lastSendMs = now;
        lastSentMask = buttonMask;

        RemotePacket pkt;
        pkt.buttonMask = buttonMask;

        if (gBoatHeading.valid)
        {
            pkt.boatHeadingDeg10 =
                (uint16_t)roundf(gBoatHeading.headingDeg * 10.0f);

            pkt.boatFlags |= REMOTE_FLAG_BOAT_IMU_VALID;
        }
        else
        {
            pkt.boatHeadingDeg10 = 0;
            pkt.boatFlags = 0;
        }

        esp_now_send(RECEIVER_MAC,
                     reinterpret_cast<const uint8_t *>(&pkt),
                     sizeof(pkt));
    }

    // Link status
    const bool linkAlive = gHasStatus && ((now - gLastStatusMs) < 1000);

    // Uppdatera display
    static uint32_t lastDisplayMs = 0;

    if (now - lastDisplayMs >= 200)
    {
        lastDisplayMs = now;

        display_update(
            gStatus,
            gHasStatus,
            buttonMask,
            linkAlive,
            (gStatus.calFlags & STATUS_CAL_FLAG_ACTIVE) != 0,
            (gStatus.calFlags & STATUS_CAL_FLAG_COMPLETE) != 0,
            gStatus.calBucketMask,
            gStatus.calPhase);
        }

    
}