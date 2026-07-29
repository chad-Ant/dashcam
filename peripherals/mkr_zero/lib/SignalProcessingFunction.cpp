#include <string.h>   // memset

#include "SignalProcessingFunctions.h"

// ─── CircularMovingAverage ────────────────────────────────────────────────────

CircularMovingAverage::CircularMovingAverage(const FilterWSize wSize):
    windowSize(wSize),
    sinBuffer(new float[wSize]),
    cosBuffer(new float[wSize]),
    bufferIndex(0),
    reciprocalDivisor(NAN),
    sumSin(0.0f),
    sumCos(0.0f),
    outputValid(false){
        // IEEE 754 zero is all-bits-zero, so memset is a valid float clear here
        // (same assumption SimpleMovingAverage documents).
        memset(sinBuffer, 0, windowSize * sizeof(float));
        memset(cosBuffer, 0, windowSize * sizeof(float));
        switch (windowSize){
            case SIZE_8:
                reciprocalDivisor = 0.125f;
                break;
            case SIZE_16:
                reciprocalDivisor = 0.0625f;
                break;
            case SIZE_32:
                reciprocalDivisor = 0.03125f;
                break;
            default:
                reciprocalDivisor = NAN;
                break;
        }
    }

CircularMovingAverage::~CircularMovingAverage(){
    delete[] sinBuffer;
    delete[] cosBuffer;
}

bool CircularMovingAverage::calculate(float inputDeg, float &outputDeg){
    if (isnan(reciprocalDivisor)){
        return false;
    }
    // Skip rather than fold in: the sums are incremental, so a single NAN would
    // make every subsequent output NAN until reset().
    if (isnan(inputDeg)){
        return outputValid;
    }

    const float rad = wrapAngle360(inputDeg) * DEG_TO_RAD_F;
    const float s   = sinf(rad);
    const float c   = cosf(rad);

    sumSin += s - sinBuffer[bufferIndex];
    sumCos += c - cosBuffer[bufferIndex];
    sinBuffer[bufferIndex] = s;
    cosBuffer[bufferIndex] = c;
    bufferIndex = (bufferIndex + 1) & (windowSize - 1); // power-of-two wrap

    if (!outputValid && bufferIndex == 0){
        outputValid = true;
    }
    if (!outputValid){
        return false;   // warming up: leave outputDeg untouched
    }

    // atan2f is defined for every quadrant and needs no division.  When the
    // samples cancel out both sums approach zero and atan2f(0,0) yields 0 —
    // a legal but meaningless answer, which is precisely what
    // resultantLength() exists to expose.
    outputDeg = wrapAngle360(atan2f(sumSin, sumCos) * RAD_TO_DEG_F);
    return true;
}

float CircularMovingAverage::resultantLength() const{
    if (!outputValid || isnan(reciprocalDivisor)){
        return NAN;
    }
    return sqrtf(sumSin * sumSin + sumCos * sumCos) * reciprocalDivisor;
}

void CircularMovingAverage::reset(){
    memset(sinBuffer, 0, windowSize * sizeof(float));
    memset(cosBuffer, 0, windowSize * sizeof(float));
    bufferIndex = 0;
    sumSin      = 0.0f;
    sumCos      = 0.0f;
    outputValid = false;
}

// ─── AccelerationEstimator ────────────────────────────────────────────────────

AccelerationEstimator::AccelerationEstimator(const FilterWSize wSize,
                                             float maxAbsMs2, float jerkMs3):
    speedFilter(wSize),
    accel(0.0f),
    prevSmoothed(0.0f),
    prevMs(0),
    havePrev(false),
    valid(false),
    maxAbs(maxAbsMs2),
    jerk(jerkMs3){
}

void AccelerationEstimator::reset(){
    speedFilter.reset();
    accel        = 0.0f;
    prevSmoothed = 0.0f;
    prevMs       = 0;
    havePrev     = false;
    valid        = false;
}

bool AccelerationEstimator::update(float speedKmh, uint32_t sampleMs){
    // NAN means the ECU never answered the SPEED PID.  Feeding it into the
    // moving average would poison every subsequent output (the running sum
    // becomes NAN and never recovers), so drop the whole chain instead.
    if (isnan(speedKmh)){
        reset();
        return false;
    }

    float smoothed = 0.0f;
    float sample   = speedKmh;              // calculate() takes a non-const reference
    const bool warm = speedFilter.calculate(sample, smoothed);
    if (!warm){
        // Still filling the window: remember nothing, because differentiating a
        // partially-filled average would read as a large false acceleration.
        havePrev = false;
        valid    = false;
        return false;
    }

    if (!havePrev){
        prevSmoothed = smoothed;
        prevMs       = sampleMs;
        havePrev     = true;
        return false;                        // need two smoothed samples to differentiate
    }

    // Unsigned subtraction is millis()-rollover safe.
    const uint32_t elapsedMs = sampleMs - prevMs;
    if (elapsedMs == 0){
        return valid;                        // same millisecond; nothing new to compute
    }
    if (elapsedMs > ACCEL_MAX_GAP_MS){
        // A long gap means samples were missed (bus dropout, blocked loop).
        // Differentiating across it would manufacture a huge acceleration, so
        // restart the derivative from this sample.
        prevSmoothed = smoothed;
        prevMs       = sampleMs;
        valid        = false;
        return false;
    }

    const float dt  = (float)elapsedMs * 0.001f;
    float       raw = (smoothed - prevSmoothed) * KMH_TO_MS / dt;

    // Clamp the raw derivative BEFORE slew limiting so one impossible sample is
    // discarded outright rather than dragging the output toward it.
    saturate(raw, maxAbs, -maxAbs);

    // Jerk limit scaled by dt, so the constraint is real acceleration change per
    // second rather than per call.
    rateLimit(raw, accel, jerk * dt);
    saturate(accel, maxAbs, -maxAbs);

    prevSmoothed = smoothed;
    prevMs       = sampleMs;
    valid        = true;
    return true;
}
