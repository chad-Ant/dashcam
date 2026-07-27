#ifndef SEGMENT_LED_FUNCTIONS
#define SEGMENT_LED_FUNCTIONS 1

#include <Arduino.h>
#include "Adafruit_LEDBackpack.h"
#include "DataDictionary.h"

/// 4×14-segment Adafruit AlphaNum4 display uses I2C address 0x70 (112).

/**
 * @brief Raw 16-bit segment bitmasks for the Adafruit AlphaNum4 14-segment display.
 *
 * Values ending in @c _MIRROR are horizontally reflected — use these when
 * the display is viewed through a reflective surface (e.g. instrument glass).
 * Values ending in @c _DP include the decimal-point segment (bit 14).
 */
enum DigitMapping
{
    NUM_0 = 0x213F,           ///< Digit '0'.
    NUM_0_DP = 0x613F,        ///< '0' with decimal point.
    NUM_1 = 0x406,            ///< Digit '1'.
    NUM_1_DP = 0x4406,        ///< '1' with decimal point.
    NUM_1_MIRROR = 0x130,     ///< Mirrored '1'.
    NUM_1_DP_MIRROR = 0x4130, ///< Mirrored '1' with decimal point.
    NUM_2 = 0x88B,
    NUM_2_DP = 0x488B,
    NUM_2_MIRROR = 0x2069,
    NUM_2_DP_MIRROR = 0x6069,
    NUM_3 = 0x8F,
    NUM_3_DP = 0x408F,
    NUM_3_MIRROR = 0x79,
    NUM_3_DP_MIRROR = 0x4079,
    NUM_4 = 0xE6,
    NUM_4_DP = 0x40E6,
    NUM_4_MIRROR = 0xF2,
    NUM_4_DP_MIRROR = 0x40F2,
    NUM_5 = 0x2069,
    NUM_5_DP = 0x6069,
    NUM_5_MIRROR = 0x88B,
    NUM_5_DP_MIRROR = 0x488B,
    NUM_6 = 0xFD,
    NUM_6_DP = 0x40FD,
    NUM_6_MIRROR = 0xDF,
    NUM_6_DP_MIRROR = 0x40DF,
    NUM_7 = 0x7,
    NUM_7_DP = 0x4007,
    NUM_7_MIRROR = 0x31,
    NUM_7_DP_MIRROR = 0x4031,
    NUM_8 = 0xFF,
    NUM_8_DP = 0x40FF,
    NUM_9 = 0xEF,
    NUM_9_DP = 0x40EF,
    NUM_9_MIRROR = 0xFB,
    NUM_9_DP_MIRROR = 0x40FB,
    CHAR_A_MIRROR = 0xF7,
    CHAR_B_MIRROR = 0x1DF,
    CHAR_C_MIRROR = 0xF,
    CHAR_D_MIRROR = 0xC0E,
    CHAR_E_MIRROR = 0x8F,
    CHAR_F_MIRROR = 0x87,
    CHAR_G_MIRROR = 0x5F,
    CHAR_H_MIRROR = 0xF6,
    CHAR_I_MIRROR = 0x1209,
    CHAR_J_MIRROR = 0x3C,
    CHAR_K_MIRROR = 0x986,
    CHAR_L_MIRROR = 0xE,
    CHAR_M_MIRROR = 0x536,
    CHAR_N_MIRROR = 0xC36,
    CHAR_O_MIRROR = 0x3F,
    CHAR_P_MIRROR = 0xE7,
    CHAR_Q_MIRROR = 0x83F,
    CHAR_R_MIRROR = 0x8E7,
    CHAR_S_MIRROR = 0xDB,
    CHAR_T_MIRROR = 0x1201,
    CHAR_U_MIRROR = 0x3E,
    CHAR_V_MIRROR = 0x2106,
    CHAR_W_MIRROR = 0x2836,
    CHAR_X_MIRROR = 0x2D00,
    CHAR_Y_MIRROR = 0x1500,
    CHAR_Z_MIRROR = 0x2109,
    CHAR_HYPHEN = 0x200,
    ALL_SEGMENTS = 0x7FFF,        ///< All 15 segments on (used for self-test).
    X_CROSS = 0x2D00,             ///< Diagonal cross (×).
    PLUS_CROSS = 0x12C0,          ///< Plus sign (+).
    STAR_5 = 0x2AC0,              ///< 5-pointed star.
    STAR_8 = 0x3FC0,              ///< 8-pointed star.
    EXPONENT_E_MIRROR = 0x208C,   ///< Superscript-E symbol (mirrored).
    MINUS_SIGN = 0x40,            ///< Minus / hyphen.
    EQUAL_SIGN = 0xC8,            ///< Equal sign (=).
    GREATER_SIGN_MIRROR = 0x2400, ///< Greater-than (> mirrored).
    GEQ_SIGN_MIRROR = 0x2408,     ///< Greater-or-equal (≥ mirrored).
    LESS_SIGN_MIRROR = 0x900,     ///< Less-than (< mirrored).
    LEQ_SIGN_MIRROR = 0x908,      ///< Less-or-equal (≤ mirrored).
    LEFT_ARROW1_MIRROR = 0x940,   ///< Thin left arrow (mirrored).
    LEFT_ARROW2_MIRROR = 0x9C0,   ///< Bold left arrow (mirrored).
    RIGHT_ARROW1_MIRROR = 0x2480, ///< Thin right arrow (mirrored).
    RIGHT_ARROW2_MIRROR = 0x24C0, ///< Bold right arrow (mirrored).
    UP_ARROW1 = 0x3800,           ///< Thin up arrow.
    UP_ARROW2 = 0x3A00,           ///< Bold up arrow.
    DOWN_ARROW1 = 0x700,          ///< Thin down arrow.
    DOWN_ARROW2 = 0x1700,         ///< Bold down arrow.
    DASH = 0xC0,                  ///< Middle dash.
    UNDERSCORE = 0x8,             ///< Underscore.
    COMMA_MIRROR = 0x2000,        ///< Comma (mirrored).
    FSLASH_MIRROR = 0x2040,       ///< Forward slash / (mirrored).
    BSLASH_MIRROR = 0xC00,        ///< Back slash \\ (mirrored).
    NONE_TO_DISPLAY = 0x0         ///< Blank — all segments off.
};

/**
 * @brief Initialises the AlphaNum4 display and runs a brief all-segments self-test.
 *
 * Connects to the display at @c SEGLED_ADDRESS, sets maximum brightness,
 * lights all segments for 500 ms, then clears the display.
 *
 * @param[in,out] alpha4  Adafruit_AlphaNum4 instance to initialise.
 * @return @c true if @c begin() succeeded, @c false if the display was not
 *         found on the I2C bus.
 */
bool initializeSegmentLED(Adafruit_AlphaNum4 &alpha4);

/**
 * @brief Turns off all digits and writes the blank frame to the display.
 * @param[in,out] alpha4  Initialised display instance.
 */
void clearSegmentLED(Adafruit_AlphaNum4 &alpha4);

/**
 * @brief Adjusts display brightness based on an ambient light sensor reading.
 *
 * Maps an 8-bit ADC reading to the 0–15 brightness range accepted by
 * @c setBrightness().  Enforces a minimum brightness of 5 so the display
 * remains visible in low-light conditions.
 *
 * @param[in,out] alpha4            Initialised display instance.
 * @param[in]     ambientLuminosity Raw 8-bit ambient light level (0–255).
 *                                  Values ≥ 80 are mapped via right-shift;
 *                                  values < 80 are clamped to brightness 5.
 */
void adjustLEDBrightness(Adafruit_AlphaNum4 &alpha4, uint8_t ambientLuminosity);

/**
 * @brief Renders a floating-point number using mirrored segment bitmaps.
 *
 * Intended for displays viewed through a reflective surface.  Supports the
 * range [−999, 9999] with one decimal place for values in (−100, 1000).
 * Special values: @c +INF, @c -INF, and @c NAN are displayed symbolically.
 *
 * @param[in,out] alpha4   Initialised display instance.
 * @param[in]     number   Value to display; clamped to [−999, 9999].
 */
void writeFloatLED_Mirror(Adafruit_AlphaNum4 &alpha4, float number);

/**
 * @brief Renders up to four characters of a C-string using mirrored segment bitmaps.
 *
 * Only the first four characters are displayed (matching the 4-digit
 * display capacity).  Unsupported characters are shown as blank.
 *
 * @param[in,out] alpha4       Initialised display instance.
 * @param[in]     stringInput  Null-terminated string to display; passing
 *                             @c nullptr clears the display.
 */
void writeStringLED_Mirror(Adafruit_AlphaNum4 &alpha4, const char *stringInput);

/**
 * @brief Renders a floating-point number using standard (non-mirrored) segment bitmaps.
 *
 * Supports the range [−999, 9999] with one decimal place for values in
 * (−100, 1000).  Special values @c +INF, @c -INF, and @c NAN are displayed
 * symbolically using ASCII characters.
 *
 * @param[in,out] alpha4   Initialised display instance.
 * @param[in]     number   Value to display; clamped to [−999, 9999].
 */
void writeFloatLED(Adafruit_AlphaNum4 &alpha4, float number);

/**
 * @brief Renders up to four characters of a C-string using standard ASCII mapping.
 *
 * Passes each character to @c writeDigitAscii() via the Adafruit library's
 * built-in ASCII lookup table.  Only the first four characters are shown.
 *
 * @param[in,out] alpha4       Initialised display instance.
 * @param[in]     stringInput  Null-terminated string to display; passing
 *                             @c nullptr clears the display.
 */
void writeStringLED(Adafruit_AlphaNum4 &alpha4, const char *stringInput);

#endif
