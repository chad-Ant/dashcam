#ifndef HOST_ARDUINO_STUB_H
#define HOST_ARDUINO_STUB_H 1

/**
 * @file Arduino.h
 * @brief Host stand-in for the Arduino API, plus a model of the 74HC165 chain.
 *
 * Resolved INSTEAD of the real Arduino.h because the Makefile puts this
 * directory first on the include path.  It provides only what
 * lib/SwitchFunctions.cpp actually uses — this is not an Arduino emulator and
 * should never grow into one.  If a future module under test needs more, add
 * the smallest thing that compiles.
 *
 * ── WHY THERE IS A DEVICE MODEL HERE AND NOT JUST A PIN ARRAY ────────────────
 * The obvious stub records pin writes and returns canned bits, which tests the
 * state machine and leaves the shift loop uncovered.  That is the wrong half:
 * the shift loop is where the bit ORDER lives, and bit order is the thing a
 * reader cannot check by inspection and a bench cannot check without a known
 * pattern.
 *
 * So @c digitalRead() on the data pin is backed by an actual 16-stage shift
 * register that loads asynchronously while PL is LOW and shifts on the rising
 * edge of CP, exactly as the datasheet describes.  A driver that clocked before
 * reading, or shifted the wrong way, or mislocated the sentinels, fails against
 * it — including the read-before-clock rule, which is otherwise only enforced
 * by a comment.
 */

#include <stdint.h>
#include <stddef.h>

#define HIGH 1
#define LOW  0

#define INPUT          0x0
#define OUTPUT         0x1
#define INPUT_PULLUP   0x2
#define INPUT_PULLDOWN 0x3

/// Only present so DataDictionary.h's STATUS_INDICATOR macro has a definition
/// if anything ever expands it. Nothing under test does.
#define LED_BUILTIN 32

// ─── the Arduino surface SwitchFunctions.cpp uses ────────────────────────────

void     pinMode(uint32_t pin, uint32_t mode);
void     digitalWrite(uint32_t pin, uint32_t value);
int      digitalRead(uint32_t pin);
void     delayMicroseconds(uint32_t us);
uint32_t millis();

// ─── test control ────────────────────────────────────────────────────────────

/// Sets the value millis() returns. The clock never advances on its own.
void hostSetMillis(uint32_t ms);

/// Last mode passed to pinMode() for @p pin, or 0xFF if never configured.
uint32_t hostPinMode(uint32_t pin);

/// Resets pins, counters and the chain model. Call before every test.
void hostReset();

/**
 * @brief Sets the sixteen parallel inputs, in the driver's raw-word layout.
 *
 * Bit 15 is #B D7, the first bit out; bit 0 is #A D0, the last. A faithful read
 * of a present chain therefore returns exactly this word, which is what makes
 * the transport assertion a real check rather than a tautology.
 */
void fakeChainSetPanel(uint16_t panel);

/**
 * @brief Fitted or not.
 *
 * When absent the data pin simply floats to whatever bias the driver selected —
 * which, with INPUT_PULLUP, reads HIGH and produces 0xFFFF. That word failing
 * the sentinel test is the entire absence-detection mechanism, so it is worth a
 * test of its own.
 */
void fakeChainSetPresent(bool present);

uint32_t fakeChainLoadPulses();  ///< Falling edges seen on PL.
uint32_t fakeChainClocks();      ///< Rising edges seen on CP while PL is HIGH.

#endif // HOST_ARDUINO_STUB_H
