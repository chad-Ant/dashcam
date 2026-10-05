#ifndef HOST_ARDUINO_STUB_H
#define HOST_ARDUINO_STUB_H 1

/**
 * @file Arduino.h
 * @brief Host stand-in for the Arduino API, plus a model of the 74HC165 chain.
 *
 * Resolved INSTEAD of the real Arduino.h because the Makefile puts this
 * directory first on the include path.  It provides only what the modules
 * under test actually use — this is not an Arduino emulator and should never
 * grow into one.  If a future module under test needs more, add the smallest
 * thing that compiles.
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

/// Enumerators, not macros — as in the real core (ArduinoCore-API
/// api/Common.h, used by arduino:samd 1.8.14). Macros would rewrite every
/// scoped enumerator of the same name, and VehGear::LOW is one.
enum PinStatus { LOW = 0, HIGH = 1 };

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
void     delay(uint32_t ms);
uint32_t millis();

// ─── ...and the little more IMUFunctions.cpp and CANSniffFunctions.cpp use ───
//
// There is one thread and no interrupt source on the host, so masking is a
// no-op. The drain timer's interrupt is modelled separately, and explicitly,
// in can_stream_hw_stub.h: a test fires it, and the model defers it while the
// firmware holds it masked, as the NVIC does.

inline void noInterrupts() {}
inline void interrupts()   {}

/// micros() as the test sets it, independent of millis(). The drain ISR stamps
/// frames with it, so tests that care about arrival times set both.
uint32_t micros();

typedef uint8_t byte;   // OBD2Functions.h, via the telemetry builder

#define F(s) (s)
#define DEC 10
#define HEX 16

/**
 * @brief The USB CDC console, and the one bulk IN packet it can have in flight.
 *
 * print()/println() still swallow everything: the modules narrate to it, and a
 * test asserts on state, never on console text. write() is different — it is
 * how the raw CAN stream leaves (CANRawStream.cpp) — so it is captured, and it
 * models the part of the core that decides whether a write BLOCKS: after a
 * write the bank stays armed until the test "collects" it (hostUsbCollect()),
 * and a write issued while it is armed is exactly where arduino:samd's
 * USBDeviceClass::send() would spin for up to 70 ms. Such writes are counted,
 * and the stream module must never make one.
 *
 * availableForWrite() returns what the test sets, 63 by default — the constant
 * EPX_SIZE - 1 the real core returns whatever the endpoint is doing.
 */
struct HostSerial {
    template <typename T> void print(const T &, int = DEC) {}
    template <typename T> void println(const T &, int = DEC) {}
    void println() {}

    bool   dtr();
    int    availableForWrite();
    size_t write(const uint8_t *buf, size_t n);
};
extern HostSerial Serial;

/// Arduino's Stream, as far as CommProtocol.cpp's pollFrame() reads one.
class Stream {
public:
    virtual ~Stream() {}
    virtual int available() = 0;
    virtual int read() = 0;
};

/// Serial1, the C3 link: never delivers a byte and accepts every write. Only the
/// telemetry builder's file needs it to link; no test drives the link.
struct HostUart : public Stream {
    void   begin(unsigned long) {}
    int    available() override { return 0; }
    int    read() override { return -1; }
    int    availableForWrite() { return 0; }
    size_t write(const uint8_t *, size_t n) { return n; }
};
extern HostUart Serial1;

// ─── test control ────────────────────────────────────────────────────────────

/// Sets the value millis() returns. The clock never advances on its own.
void hostSetMillis(uint32_t ms);

/// Sets the value micros() returns. Never advances on its own either.
void hostSetMicros(uint32_t us);

// ─── the USB console's bank model (see HostSerial) ──────────────────────────
//
// Reset by hostReset(): no host (DTR low), enumerated, bank free, room 63.

void        hostUsbSetDtr(bool high);         ///< A host opened (true) or closed the port.
void        hostUsbSetConfigured(bool on);    ///< Enumerated by a host, or not.
void        hostUsbSetRoom(int bytes);        ///< What availableForWrite() reports.
void        hostUsbSetWriteFails(bool fail);  ///< write() returns (size_t)-1, as the core does on a bus reset.
void        hostUsbCollect();                 ///< The host takes the armed packet; the bank is free.
bool        hostUsbBankBusy();                ///< A packet is armed and not yet collected.
bool        hostUsbConfigured();
const char *hostUsbOutput();                  ///< Everything written so far, NUL-terminated.
void        hostUsbClearOutput();             ///< Forgets the output AND the three counters below.
uint32_t    hostUsbWrites();                  ///< write() calls that were accepted.
uint32_t    hostUsbBlockingWrites();          ///< write() calls made while the bank was armed.
uint32_t    hostUsbLargestWrite();            ///< Longest single write(), in bytes.

/// Last mode passed to pinMode() for @p pin, or 0xFF if never configured.
uint32_t hostPinMode(uint32_t pin);

/// Resets pins, counters and the chain model. Call before every test.
void hostReset();

/// Runs @p fn once, on the next digitalRead() of @p pin, just before or just
/// after the level is sampled — an interrupt landing mid-read. Cleared by
/// hostReset().
void hostOnNextRead(uint32_t pin, bool beforeSample, void (*fn)());

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
