#include <Arduino.h>
#include <Wire.h>
#include <math.h>

static constexpr uint8_t QMC5883_ADDR = 0x0D;
static constexpr int SDA_PIN = 21;
static constexpr int SCL_PIN = 22;
static constexpr uint32_t I2C_FREQ_HZ = 100000;

// Din nuvarande hard/soft-iron kalibrering
static constexpr float COMPASS_OFFSET_X = -27.5f;
static constexpr float COMPASS_OFFSET_Y = -550.5f;
static constexpr float COMPASS_GAIN_X   = 1.164f;
static constexpr float COMPASS_GAIN_Y   = 0.876f;
static constexpr float COMPASS_CROSS_XY = 0.05f;
static constexpr float COMPASS_CROSS_YX = 0.05f;

// Bara för referens i utskrift, används inte i heading i detta test
static constexpr float HEADING_OFFSET_DEG = 134.0f;

static float wrap360(float deg) {
    while (deg < 0.0f) deg += 360.0f;
    while (deg >= 360.0f) deg -= 360.0f;
    return deg;
}

static void writeReg(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(QMC5883_ADDR);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

static bool readRawXYZ(int16_t &x, int16_t &y, int16_t &z) {
    Wire.beginTransmission(QMC5883_ADDR);
    Wire.write(0x00);
    if (Wire.endTransmission(false) != 0) return false;

    if (Wire.requestFrom(QMC5883_ADDR, (uint8_t)6) != 6) return false;

    x = (int16_t)(Wire.read() | (Wire.read() << 8));
    y = (int16_t)(Wire.read() | (Wire.read() << 8));
    z = (int16_t)(Wire.read() | (Wire.read() << 8));
    return true;
}

static bool initQMC5883() {
    writeReg(0x0B, 0x01); // reset
    delay(20);
    writeReg(0x09, 0x1D); // continuous, 200Hz, 8G, OSR 512
    writeReg(0x0A, 0x00);
    delay(20);
    return true;
}

struct SampleStats {
    double rawX = 0;
    double rawY = 0;
    double rawZ = 0;
    double xCal = 0;
    double yCal = 0;
    double hRawSin = 0;
    double hRawCos = 0;
    double hCalSin = 0;
    double hCalCos = 0;
    double magXY = 0;
    uint32_t count = 0;
};

static void addSample(SampleStats &s, int16_t rawX, int16_t rawY, int16_t rawZ) {
    float x0 = (float)rawX - COMPASS_OFFSET_X;
    float y0 = (float)rawY - COMPASS_OFFSET_Y;

    float xCal = COMPASS_GAIN_X * x0 + COMPASS_CROSS_XY * y0;
    float yCal = COMPASS_CROSS_YX * x0 + COMPASS_GAIN_Y * y0;

    float hRaw = wrap360(atan2f((float)rawY, (float)rawX) * 180.0f / PI);
    float hCal = wrap360(atan2f(yCal, xCal) * 180.0f / PI);
    float mxy  = sqrtf(xCal * xCal + yCal * yCal);

    s.rawX += rawX;
    s.rawY += rawY;
    s.rawZ += rawZ;
    s.xCal += xCal;
    s.yCal += yCal;
    s.magXY += mxy;

    float hRawRad = hRaw * PI / 180.0f;
    float hCalRad = hCal * PI / 180.0f;

    s.hRawSin += sinf(hRawRad);
    s.hRawCos += cosf(hRawRad);
    s.hCalSin += sinf(hCalRad);
    s.hCalCos += cosf(hCalRad);

    s.count++;
}

static float avgAngleDeg(double s, double c) {
    return wrap360(atan2((float)s, (float)c) * 180.0f / PI);
}

static void printStepResult(const SampleStats &s, int stepIndex) {
    if (s.count == 0) {
        Serial.printf("STEP %d: no samples\n", stepIndex);
        return;
    }

    float rawX = s.rawX / s.count;
    float rawY = s.rawY / s.count;
    float rawZ = s.rawZ / s.count;
    float xCal = s.xCal / s.count;
    float yCal = s.yCal / s.count;
    float hRaw = avgAngleDeg(s.hRawSin, s.hRawCos);
    float hCal = avgAngleDeg(s.hCalSin, s.hCalCos);
    float mxy  = s.magXY / s.count;

    Serial.printf(
        "STEP %d (%3d deg) | N=%lu | "
        "RAWavg x=%7.1f y=%7.1f z=%7.1f | "
        "CALavg x=%8.2f y=%8.2f | "
        "Hraw=%7.2f Hcal=%7.2f | "
        "Mxy=%8.2f\n",
        stepIndex,
        stepIndex * 45,
        (unsigned long)s.count,
        rawX, rawY, rawZ,
        xCal, yCal,
        hRaw, hCal,
        mxy
    );
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.println("=== QMC5883 45-degree jig test ===");
    Serial.println("Hall sensorn plant.");
    Serial.println("Placera i 45-graderslagen.");
    Serial.println("Varje steg matas i serial monitor med Enter.");
    Serial.println("Koden samlar i 3 sekunder och skriver medelvarde.");
    Serial.println();

    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(I2C_FREQ_HZ);

    if (!initQMC5883()) {
        Serial.println("QMC5883 init failed");
        while (true) delay(1000);
    }

    Serial.println("Skriv Enter for steg 0.");
}

void loop() {
    static int stepIndex = 0;

    if (Serial.available()) {
        while (Serial.available()) Serial.read();

        Serial.printf("\n--- Measuring STEP %d (%d deg) for 3 seconds ---\n",
                      stepIndex, stepIndex * 45);

        SampleStats stats;
        uint32_t startMs = millis();

        while (millis() - startMs < 3000) {
            int16_t rawX, rawY, rawZ;
            if (readRawXYZ(rawX, rawY, rawZ)) {
                addSample(stats, rawX, rawY, rawZ);
            }
            delay(20);
        }

        printStepResult(stats, stepIndex);

        stepIndex = (stepIndex + 1) % 8;
        Serial.printf("Rotera till STEP %d (%d deg) och tryck Enter.\n",
                      stepIndex, stepIndex * 45);
    }
}