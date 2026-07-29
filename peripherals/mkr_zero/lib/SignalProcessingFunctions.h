#ifndef SIGPROC_FUNCTIONS
#define SIGPROC_FUNCTIONS 1

#include <Arduino.h>
#include <math.h>
#include "MathFunctions.h"   // swapCustom() used by saturate(); divThreshold/fastReciprocal used by PIDControls

/** Window sizes supported by @c SimpleMovingAverage.  Must be powers of two. */
enum FilterWSize{
    SIZE_8  = 8,  ///< 8-sample window.
    SIZE_16 = 16, ///< 16-sample window.
    SIZE_32 = 32  ///< 32-sample window.
};

/**
 * @brief Increments an 8-bit counter, or resets it to zero.
 *
 * @param[in,out] counter  Counter variable to update.
 * @param[in]     reset    If @c true, set counter to 0 and return immediately
 *                         without incrementing.
 */
inline void counterU8(uint8_t &counter, bool reset = false){
    if (reset){
        counter = 0;
        return;
    }
    counter++;
}

/**
 * @brief Increments a 16-bit counter, or resets it to zero.
 *
 * @param[in,out] counter  Counter variable to update.
 * @param[in]     reset    If @c true, set counter to 0 and return immediately.
 */
inline void counterU16(uint16_t &counter, bool reset = false){
    if (reset){
        counter = 0;
        return;
    }
    counter++;
}

/**
 * @brief Increments a 32-bit counter, or resets it to zero.
 *
 * @param[in,out] counter  Counter variable to update.
 * @param[in]     reset    If @c true, set counter to 0 and return immediately.
 */
inline void counterU32(uint32_t &counter, bool reset = false){
    if (reset){
        counter = 0;
        return;
    }
    counter++;
}

/**
 * @brief Limits the rate of change of a signal.
 *
 * Clamps the difference between @p input and the current @p output to
 * ±@p delta per call, effectively slew-rate limiting the output.
 *
 * @param[in]     input   Desired target value.
 * @param[in,out] output  Current output value; updated in-place.
 * @param[in]     delta   Maximum allowed change per call (must be ≥ 0).
 */
inline void rateLimit(float input, float &output, float delta){
    float tempNumUp = output + delta;
    float tempNumDown = output - delta;
    output = (input > tempNumUp) ? tempNumUp : ((input < tempNumDown) ? tempNumDown : input);
}

/**
 * @brief Clamps a value to the interval [@p lBound, @p uBound].
 *
 * Bounds are sorted automatically so swapped arguments are handled
 * gracefully.
 *
 * @param[in,out] num     Value to saturate.
 * @param[in]     uBound  Upper saturation limit.
 * @param[in]     lBound  Lower saturation limit.
 */
inline void saturate(float &num, float uBound, float lBound){
    if (uBound < lBound) swapCustom(uBound, lBound);
    num = num < uBound ? (num > lBound ? num : lBound) : uBound;
}

/**
 * @brief Causal simple moving average filter with a power-of-two window.
 *
 * Uses a circular buffer and a pre-computed reciprocal divisor for
 * efficient per-sample updates with no division.  The output is not
 * considered valid until the buffer has been filled once (warm-up period).
 *
 * @note Copy construction and assignment are disabled; create on the stack
 *       or heap and pass by reference.
 */
class SimpleMovingAverage{
    public:
        /**
         * @brief Constructs the filter and allocates the internal buffer.
         * @param[in] wSize  Window size; must be @c SIZE_8, @c SIZE_16, or @c SIZE_32.
         */
        SimpleMovingAverage(const FilterWSize wSize);

        /** @brief Destructor — frees the internal buffer. */
        ~SimpleMovingAverage();

        SimpleMovingAverage(const SimpleMovingAverage&) = delete;
        SimpleMovingAverage& operator = (const SimpleMovingAverage&) = delete;

        /**
         * @brief Feeds one sample into the filter and returns the current average.
         *
         * @param[in]  input   New sample to add.
         * @param[out] output  Current windowed average.
         * @return @c true once the buffer has been filled for the first time
         *         (output is valid); @c false during the initial warm-up period.
         */
        bool calculate(float &input, float &output);

        /**
         * @brief Resets all internal state to zero and restarts the warm-up period.
         */
        void reset();

    private:
        FilterWSize windowSize;
        float*      outputBuffer;
        uint8_t     bufferIndex;
        float       reciprocalDivisor;
        float       sum;
        bool        outputValid;
};

// ─── circular (angular) quantities ────────────────────────────────────────────
//
// Headings, bearings and phases live on a circle, where 359 deg and 1 deg are 2
// deg apart, not 358.  Every linear tool above is WRONG on such a signal:
// SimpleMovingAverage over {359, 1} returns 180 (due south instead of due
// north), and rateLimit() sweeps the long way round.  Use these instead.
//
// Float-suffixed constants on purpose: Arduino's DEG_TO_RAD / RAD_TO_DEG are
// double literals, and on the SAMD21 (Cortex-M0+, NO hardware FPU) mixing them
// in silently promotes the whole expression to software double arithmetic.

/// Degrees to radians, single precision.
#define DEG_TO_RAD_F              0.0174532925f
/// Radians to degrees, single precision.
#define RAD_TO_DEG_F              57.2957795f

/**
 * @brief Normalises an angle to [0, 360).
 * @param[in] deg  Angle in degrees; @c NAN passes through.
 */
inline float wrapAngle360(float deg){
    if (isnan(deg)) return NAN;
    deg = fmodf(deg, 360.0f);
    if (deg < 0.0f) deg += 360.0f;
    return deg;
}

/**
 * @brief Shortest signed angular difference, @p to - @p from.
 *
 * @return Difference in (-180, +180]; positive is clockwise.  E.g.
 *         @c angleDiff360(350, 10) is +20, not -340.
 */
inline float angleDiff360(float from, float to){
    float d = wrapAngle360(to - from);
    if (isnan(d)) return NAN;
    if (d > 180.0f) d -= 360.0f;
    return d;
}

/**
 * @brief Slew-rate limits an angle, taking the short way around the circle.
 *
 * The circular counterpart of @c rateLimit().  Turning from 350 deg to 10 deg
 * moves through 0, not backwards through 180.
 *
 * @param[in]     input   Target angle in degrees.
 * @param[in,out] output  Current angle; updated in-place, always in [0, 360).
 * @param[in]     delta   Maximum change per call in degrees (must be >= 0).
 */
inline void rateLimitAngle(float input, float &output, float delta){
    const float d = angleDiff360(output, input);
    if (isnan(d)) return;
    if (d > delta)        output = wrapAngle360(output + delta);
    else if (d < -delta)  output = wrapAngle360(output - delta);
    else                  output = wrapAngle360(input);
}

/// Conversion factor from km/h to m/s (1 / 3.6).
#define KMH_TO_MS                 0.277777778f
/// Longitudinal acceleration outside this magnitude is a sensor glitch, not a
/// vehicle: emergency braking tops out near -11 m/s2 on dry tarmac.
#define ACCEL_LIMIT_MS2           12.0f
/// Jerk ceiling (m/s3) used to slew-rate limit the published acceleration.
/// Applied as delta = ACCEL_JERK_LIMIT_MS3 * dt, so the limit is independent of
/// how often update() happens to be called.
#define ACCEL_JERK_LIMIT_MS3      10.0f
/// Gap after which the derivative chain is restarted rather than differentiated
/// across missing samples (ms).
#define ACCEL_MAX_GAP_MS          2000UL

/**
 * @brief Causal moving average for CIRCULAR quantities (headings, bearings).
 *
 * Averages by unit vectors, not by value: each sample becomes (cos, sin), the
 * components are averaged over the window, and the mean direction is recovered
 * with atan2().  This is the standard directional-statistics mean and is
 * correct across the 359 -> 0 wrap, where a plain @c SimpleMovingAverage is not.
 *
 * The alternative — unwrapping the angle into a continuous accumulator and
 * running a linear average on it — avoids trigonometry, which is attractive on
 * this FPU-less SAMD21.  It is not used here because the accumulator grows
 * without bound whenever the vehicle keeps turning the same way (roundabouts,
 * a spiral car park) until float precision degrades.  At the few-Hz rates this
 * filter is fed, the software sin/cos/atan2 cost is negligible — roughly 0.1 %
 * of the core at 10 Hz — so correctness wins.
 *
 * @c resultantLength() reports how consistent the window is, which is what makes
 * this usable for GPS course over ground: a stationary or crawling vehicle
 * produces headings scattered around the whole circle, whose vector mean is an
 * arbitrary direction with a resultant near 0.  Gate on it before believing the
 * output — a confident-looking heading derived from noise is worse than none.
 *
 * @note Copy construction and assignment are disabled; the filter owns buffers.
 */
class CircularMovingAverage{
    public:
        /**
         * @brief Constructs the filter and allocates its buffers.
         * @param[in] wSize  Window size; must be @c SIZE_8, @c SIZE_16, or @c SIZE_32.
         */
        CircularMovingAverage(const FilterWSize wSize);

        /** @brief Destructor — frees the internal buffers. */
        ~CircularMovingAverage();

        CircularMovingAverage(const CircularMovingAverage&) = delete;
        CircularMovingAverage& operator = (const CircularMovingAverage&) = delete;

        /**
         * @brief Feeds one angle into the filter and returns the mean direction.
         *
         * Unlike @c SimpleMovingAverage this takes the sample BY VALUE, so a
         * computed expression can be passed directly.
         *
         * A @c NAN input is skipped, not folded in: the running component sums
         * are incremental, so one NAN would make every later output NAN until
         * @c reset().  Skipping keeps a brief GPS dropout from destroying the
         * window, at the cost of the window spanning a longer real interval.
         *
         * @param[in]  inputDeg   Sample in degrees (any range; wrapped internally).
         * @param[out] outputDeg  Mean direction in [0, 360). Untouched if the
         *                        sample was skipped or the filter is warming up.
         * @return @c true once the window has been filled and @p outputDeg is valid.
         */
        bool calculate(float inputDeg, float &outputDeg);

        /**
         * @brief Directional consistency of the current window, in [0, 1].
         *
         * 1.0 = every sample points the same way; 0.0 = samples cancel out, so
         * the mean direction is meaningless.  For GPS course, values below
         * roughly 0.7 usually mean the receiver is reporting noise rather than
         * travel.
         *
         * @return Resultant length, or @c NAN before the first valid window.
         */
        float resultantLength() const;

        /** @brief True once the window has been filled at least once. */
        bool isValid() const { return outputValid; }

        /** @brief Clears the filter and restarts the warm-up period. */
        void reset();

    private:
        FilterWSize windowSize;
        float*      sinBuffer;
        float*      cosBuffer;
        uint8_t     bufferIndex;
        float       reciprocalDivisor;
        float       sumSin;
        float       sumCos;
        bool        outputValid;
};

/**
 * @brief Estimates longitudinal acceleration from a quantised speed signal.
 *
 * OBD-II reports vehicle speed as whole km/h, so differentiating consecutive
 * readings directly yields a train of spikes: long runs of exactly zero
 * separated by a single 1 km/h step, which at a 10 Hz sample rate is an
 * apparent 2.8 m/s2 impulse.  Averaging that *after* differentiating does not
 * help much, because the spike energy is real, just misplaced in time.
 *
 * So the order is deliberate: smooth the SPEED first with a
 * @c SimpleMovingAverage, turning the staircase into a ramp, and differentiate
 * the ramp.  The result is then jerk-limited with @c rateLimit() and clamped
 * with @c saturate() so a dropped or glitched reading cannot publish a physically
 * impossible value.
 *
 * Output is @c NAN until the moving average has filled its window once, so a
 * consumer can distinguish "still warming up" from "coasting at 0 m/s2".
 *
 * Call @c update() at a steady cadence; it timestamps its own samples and
 * restarts the derivative when the gap exceeds @c ACCEL_MAX_GAP_MS.
 *
 * @note Copy construction and assignment are disabled (it owns a filter).
 */
class AccelerationEstimator{
    public:
        /**
         * @brief Constructs the estimator.
         *
         * @param[in] wSize      Speed-smoothing window.  Pick it with the call
         *                       cadence in mind: SIZE_16 at 10 Hz is a 1.6 s
         *                       window, which comfortably spans several 1 km/h
         *                       quantisation steps at normal road speeds.
         * @param[in] maxAbsMs2  Saturation limit, magnitude (m/s2).
         * @param[in] jerkMs3    Slew limit on the published value (m/s3).
         */
        AccelerationEstimator(const FilterWSize wSize = SIZE_16,
                              float maxAbsMs2 = ACCEL_LIMIT_MS2,
                              float jerkMs3   = ACCEL_JERK_LIMIT_MS3);

        AccelerationEstimator(const AccelerationEstimator&) = delete;
        AccelerationEstimator& operator = (const AccelerationEstimator&) = delete;

        /**
         * @brief Feeds one speed sample.
         *
         * @param[in] speedKmh  Vehicle speed in km/h; @c NAN invalidates the
         *                      estimate (the ECU has not supplied a reading).
         * @param[in] sampleMs  @c millis() at which this sample was taken.
         * @return @c true when @c value() holds a valid estimate.
         */
        bool update(float speedKmh, uint32_t sampleMs);

        /** @brief Latest estimate in m/s2 (+ve = accelerating); @c NAN until valid. */
        float value() const { return valid ? accel : NAN; }

        /** @brief True once the filter has warmed up and a derivative exists. */
        bool isValid() const { return valid; }

        /** @brief Clears the filter and restarts the warm-up period. */
        void reset();

    private:
        SimpleMovingAverage speedFilter;
        float    accel;        ///< Published, jerk-limited estimate (m/s2).
        float    prevSmoothed; ///< Previous smoothed speed (km/h).
        uint32_t prevMs;       ///< millis() of the previous accepted sample.
        bool     havePrev;     ///< A previous smoothed sample exists to differentiate against.
        bool     valid;        ///< accel holds a meaningful value.
        float    maxAbs;       ///< Saturation magnitude (m/s2).
        float    jerk;         ///< Slew limit (m/s3).
};

/**
 * @brief Incremental PID controller in velocity (ISA) form.
 *
 * Implements the standard ISA PID:
 * @code
 *   u = Kp * e  +  Ki * integral(e, dt)  +  Kd * d(pv)/dt
 * @endcode
 * Output is saturated at [lBound, uBound] with integral anti-windup.
 * Derivative action is applied to the process variable (not the error) to
 * avoid derivative kick on setpoint changes.
 *
 * @note Ti (integral time) is in minutes; Td (derivative time) should be in
 *       seconds to match the per-second derivative term.
 * @note Copy construction and assignment are disabled.
 */
class PIDControls{
    public:
        float sp;     ///< Setpoint.
        float uBound; ///< Upper output saturation limit.
        float lBound; ///< Lower output saturation limit.

        /** Controller run state. */
        enum state{ON, OFF};

        /**
         * @brief Constructs the controller with the given tuning parameters.
         *
         * @param[in] gainP          Proportional gain (Kp).
         * @param[in] integralTime   Integral time Ti in minutes (0 disables integral).
         * @param[in] derivativeTime Derivative time Td in seconds.
         * @param[in] setpoint       Initial setpoint.
         * @param[in] upperBound     Upper output limit.
         * @param[in] lowerBound     Lower output limit.
         */
        PIDControls(float gainP, float integralTime, float derivativeTime, float setpoint, float upperBound, float lowerBound);

        /** @brief Destructor. */
        ~PIDControls();

        PIDControls(const PIDControls&) = delete;
        PIDControls& operator = (const PIDControls&) = delete;

        /**
         * @brief Bumplessly initialises the controller from a known process state.
         *
         * Sets the integral term so the output equals @p output at the current
         * @p input, avoiding a step on first @c calculate() call.
         *
         * @param[in]     input   Current process variable.
         * @param[in,out] output  Current actuator output; clamped to [lBound, uBound].
         */
        void reset(float input, float &output);

        /**
         * @brief Computes one PID iteration.
         *
         * @param[in]     input   Current process variable measurement.
         * @param[in,out] output  Controller output; updated and saturated in-place.
         * @param[in]     dt_sec  Time elapsed since the last call, in seconds.
         * @return @c true if the calculation was performed, @c false if the
         *         controller is @c OFF or @p dt_sec is zero.
         */
        bool calculate(float input, float &output, float dt_sec);

        /**
         * @brief Enables the controller with a bumpless transfer from the current state.
         * @param[in] input   Current process variable.
         * @param[in] output  Current actuator output.
         */
        void enable(float input, float output);

        /**
         * @brief Disables the controller; subsequent @c calculate() calls return @c false.
         */
        void disable();

        /**
         * @brief Returns whether the controller is currently active.
         * @return @c true if @c ON, @c false if @c OFF.
         */
        bool getState();

        /**
         * @brief Updates proportional gain and recalculates Ki / Kd on the fly.
         * @param[in]     newKp   New proportional gain.
         * @param[in,out] output  Current output (used to update integral for bumpless change).
         */
        void setKp(float newKp, float &output);

        /**
         * @brief Updates integral time and recalculates Ki on the fly.
         * @param[in]     newTi   New integral time in minutes.
         * @param[in,out] output  Current output (used to update integral for bumpless change).
         */
        void setTi(float newTi, float &output);

        /**
         * @brief Updates derivative time and recalculates Kd on the fly.
         * @param[in]     newTd   New derivative time in seconds.
         * @param[in,out] output  Current output (used to update integral for bumpless change).
         */
        void setTd(float newTd, float &output);

    private:
        float Kp;
        float Ti;
        float Td;
        float Ki;
        float Kd;
        float prevInput;
        float prevError;
        float integral;
        float derivative;
        state controlState;
};

#endif
