#include "heading_calibration.h"

static float wrap360Local(float deg)
{
    while (deg >= 360.0f)
        deg -= 360.0f;
    while (deg < 0.0f)
        deg += 360.0f;
    return deg;
}

float HeadingCalibration::apply(
    float rawDeg,
    const HeadingCorrectionPoint *table,
    uint8_t count)
{
    rawDeg = wrap360Local(rawDeg);

    if (table == nullptr || count < 2)
    {
        return rawDeg;
    }

    for (uint8_t i = 0; i < count - 1; i++)
    {
        const float r0 = table[i].raw;
        const float r1 = table[i + 1].raw;

        if (rawDeg >= r0 && rawDeg <= r1)
        {
            const float span = r1 - r0;
            if (span <= 0.01f)
                return rawDeg;

            const float t = (rawDeg - r0) / span;

            const float c0 = table[i].corr;
            const float c1 = table[i + 1].corr;

            float diff = c1 - c0;

            if (diff > 180.0f)
                diff -= 360.0f;
            if (diff < -180.0f)
                diff += 360.0f;

            return wrap360Local(c0 + t * diff);
        }
    }

    return rawDeg;
}

void AngleAverage::reset()
{
    _sumSin = 0.0;
    _sumCos = 0.0;
    _count = 0;
}

void AngleAverage::add(float deg)
{
    if (!isfinite(deg))
        return;

    const double rad = deg * DEG_TO_RAD;
    _sumSin += sin(rad);
    _sumCos += cos(rad);
    _count++;
}

bool AngleAverage::valid(uint16_t minSamples) const
{
    return _count >= minSamples;
}

float AngleAverage::averageDeg() const
{
    if (_count == 0)
        return NAN;

    return wrap360Local(atan2(_sumSin, _sumCos) * RAD_TO_DEG);
}

uint16_t AngleAverage::count() const
{
    return _count;
}

float HeadingCalibration::bucketCenterDeg(uint8_t bucketIndex)
{
    return wrap360Local(bucketIndex * HEADING_CAL_BUCKET_STEP_DEG);
}

int8_t HeadingCalibration::bucketForGpsHeading(
    float gpsHeadingDeg,
    float windowDeg)
{
    if (!isfinite(gpsHeadingDeg))
        return -1;

    gpsHeadingDeg = wrap360Local(gpsHeadingDeg);

    for (uint8_t i = 0; i < HEADING_CAL_BUCKET_COUNT; i++)
    {
        const float center = bucketCenterDeg(i);
        const float error = fabs(gpsHeadingDeg - center);

        float circularError = error;
        if (circularError > 180.0f)
        {
            circularError = 360.0f - circularError;
        }

        if (circularError <= windowDeg)
        {
            return static_cast<int8_t>(i);
        }
    }

    return -1;
}