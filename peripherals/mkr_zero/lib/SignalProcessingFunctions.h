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
