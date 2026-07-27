#ifndef SERVO_FUNCTIONS
#define SERVO_FUNCTIONS 1

#include <Arduino.h>
#include <Servo.h>
#include "DataDictionary.h"

/// Servos use predefined analog output pins (SERVO_XAXIS_PIN / SERVO_YAXIS_PIN).

#define MAX_XAXIS_ANGLE 180 ///< Maximum travel angle for the X-axis servo (degrees).
#define MAX_YAXIS_ANGLE 180 ///< Maximum travel angle for the Y-axis servo (degrees).

/** Return codes used by servo functions. */
enum class ServoReturnStatus{
    OK = 0,                  ///< Operation succeeded.
    NOK_INTERNAL_ERROR = -1, ///< Servo library reported an error.
    NOK_NOT_ATTACHED = -2,   ///< Servo is not attached to a pin.
    SERVO_NOT_DETACHED = -3  ///< closeServo() failed to detach the servo.
};

/**
 * @brief Attaches a servo to the specified pin and sets its range.
 *
 * @param[in,out] servo_x  Servo object to configure.
 * @param[in]     pin      Arduino pin number to attach to.
 * @param[in]     min      Minimum angle in degrees (default 0).
 * @param[in]     max      Maximum angle in degrees (default @c MAX_XAXIS_ANGLE).
 * @return @c ServoReturnStatus::OK if attached successfully,
 *         @c NOK_INTERNAL_ERROR if the library failed to attach.
 */
ServoReturnStatus initializeServo(Servo &servo_x, const int pin, int min = 0, int max = MAX_XAXIS_ANGLE);

/**
 * @brief Reads the last commanded angle of the servo.
 *
 * @param[in]  servo_x   Attached servo object.
 * @param[out] position  Last written angle in degrees.
 * @return @c ServoReturnStatus::OK, or @c NOK_NOT_ATTACHED if the servo is
 *         not attached to a pin.
 */
ServoReturnStatus readPosition(Servo &servo_x, int &position);

/**
 * @brief Commands the servo to move to @p position, clamped to [min, max].
 *
 * @param[in,out] servo_x  Attached servo object.
 * @param[in]     position Desired angle in degrees.
 * @param[in]     min      Lower clamp limit (default 0).
 * @param[in]     max      Upper clamp limit (default @c MAX_XAXIS_ANGLE).
 * @return @c ServoReturnStatus::OK, or @c NOK_NOT_ATTACHED.
 */
ServoReturnStatus writePosition(Servo &servo_x, int position, int min = 0, int max = MAX_XAXIS_ANGLE);

/**
 * @brief Detaches the servo from its pin, stopping PWM output.
 *
 * @param[in,out] servo_x  Servo object to release.
 * @return @c ServoReturnStatus::OK, or @c SERVO_NOT_DETACHED if the servo
 *         library still reports attached after the call.
 */
ServoReturnStatus closeServo(Servo &servo_x);

#endif
