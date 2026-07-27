#ifndef MATH_FUNCTIONS
#define MATH_FUNCTIONS 1

/// Minimum absolute value a divisor is allowed to have before it is clamped.
#define DIVISOR_LIMIT 0.001F

#include <Arduino.h>
#include <math.h>

/**
 * @brief Swaps two values — a lightweight substitute for @c std::swap.
 *
 * Lives here (rather than in ExternalLibConfig.h) so that any module needing it
 * can include just this header. Used by @c interpolate() and @c saturate().
 */
template <typename T>
void swapCustom(T& a, T& b) {
    T temp = a;
    a = b;
    b = temp;
}

/**
 * @brief Computes an approximate reciprocal of @p num using Newton–Raphson.
 *
 * Uses a bit-level initial guess (Quake/IEEE-754 trick) followed by two
 * Newton–Raphson refinement steps.  Accurate to roughly 5 significant digits
 * for typical floating-point inputs on a 32-bit platform.
 *
 * @param[in] num  Value to invert.  Must not be exactly 0; values with
 *                 absolute value below @c DIVISOR_LIMIT are clamped first.
 * @return Approximation of @c 1/num.
 * @warning Only reliable on platforms where @c sizeof(float)==sizeof(int)==4
 *          (ARM Cortex-M, ESP32).  Do not use on 8-bit AVR.
 * @note Prefer the normal @c / operator unless profiling shows a bottleneck.
 */
float fastReciprocal(float num);

/**
 * @brief Linearly maps @p num from [lBound, uBound] to [resultlBound, resultuBound].
 *
 * Values outside the source range are clamped to the corresponding result
 * boundary.  Source and result ranges are automatically sorted so that
 * reversed bounds are handled gracefully.
 *
 * @param[in] num          Input value to interpolate.
 * @param[in] uBound       Upper bound of the source range.
 * @param[in] lBound       Lower bound of the source range.
 * @param[in] resultuBound Upper bound of the output range (default 100).
 * @param[in] resultlBound Lower bound of the output range (default 0).
 * @return Mapped output value, clamped to [resultlBound, resultuBound].
 */
float interpolate(float num, float uBound, float lBound, float resultuBound = 100.0, float resultlBound = 0.0);

/**
 * @brief Returns @p num clamped away from zero by @c DIVISOR_LIMIT.
 *
 * Prevents division-by-zero by ensuring the returned value has absolute
 * value >= @c DIVISOR_LIMIT while preserving the original sign.
 *
 * @param[in] num  Candidate divisor.
 * @return @p num unchanged if |num| >= @c DIVISOR_LIMIT, otherwise
 *         ±@c DIVISOR_LIMIT with the same sign as @p num.
 */
inline float divThreshold(float num){
    return fabsf(num) < DIVISOR_LIMIT ? (num >= 0 ? DIVISOR_LIMIT : -DIVISOR_LIMIT) : num;
}

/**
 * @brief Fast integer approximation of @p number / 10.
 *
 * Uses a fixed-point multiply-and-shift instead of hardware division.
 * Error is at most 1 LSB for inputs that fit in int32_t.
 *
 * @param[in] number  Dividend (positive values recommended).
 * @return Approximation of @p number / 10.
 * @warning Does not check for overflow; results are unreliable for very
 *          large inputs (roughly |number| > 200 000 000).
 */
inline int32_t div10Approx(int32_t number){
    return (6554 * number) >> 16;
}

/**
 * @brief Fast integer approximation of @p number / 100.
 *
 * Uses a fixed-point multiply-and-shift.  Error is at most 1 LSB.
 *
 * @param[in] number  Dividend (positive values recommended).
 * @return Approximation of @p number / 100.
 * @warning Does not check for overflow.
 */
inline int32_t div100Approx(int32_t number){
    return (10486 * number) >> 20;
}

/**
 * @brief Fast integer approximation of @p number / 1000.
 *
 * Uses a 64-bit fixed-point multiply-and-shift.  Error is at most 1 LSB.
 *
 * @param[in] number  Dividend (positive values recommended).
 * @return Approximation of @p number / 1000.
 * @warning Does not check for overflow.
 */
inline int64_t div1000Approx(int64_t number){
   return (536871 * number) >> 29;
}

#endif
