/**
 * @file libgpio_pwm.h
 * @brief Hardware PWM output via the kernel's sysfs PWM interface.
 *
 * A DIFFERENT SUBSYSTEM FROM THE REST OF libgpio, which is why it is a separate
 * header. GpioPin and GpioWatcher talk to /dev/gpiochipN through libgpiod;
 * this talks to /sys/class/pwm through the filesystem. They share a namespace
 * and a library because they are both "pins" to a caller, and nothing else.
 *
 * ── Before this works at all ──────────────────────────────────────────────────
 * The Orin Nano's 40-pin header pins default to plain GPIO. A pin has to be
 * MUXED to its PWM function before the kernel exposes a chip for it:
 *
 *   sudo /opt/nvidia/jetson-io/jetson-io.py     # enable the PWM function
 *   sudo reboot                                 # required, not optional
 *   ls /sys/class/pwm/                          # pwmchip0, pwmchip1, ...
 *
 * Without that step /sys/class/pwm is empty or lacks the chip, and open() fails
 * with a message saying so rather than something vaguer — the pinmux step is
 * easy to forget and the resulting silence is otherwise hard to attribute.
 *
 * ── The chip index is NOT guaranteed to be 0 ──────────────────────────────────
 * @c chipIndex maps to a Tegra PWM controller through the device tree, and the
 * numbering depends on which controllers the running DTB enables and in what
 * order — it is not a property of the header pin. Callers pass it in rather
 * than having it assumed for exactly that reason.
 *
 * Determine it on the device, after the pinmux step:
 *
 *   ls -l /sys/class/pwm/                    # which chips exist
 *   cat /sys/class/pwm/pwmchip*/device/uevent | grep -i of_node
 *
 * The tests default to chip 0 because that is the common case, not because it
 * is reliable. If a channel exports but the pin does not move, a wrong chip
 * index is the first thing to check.
 *
 * Writing to sysfs generally needs root or a udev rule granting the group
 * access; in a container the host's /sys must be visible.
 *
 * ── Polarity ─────────────────────────────────────────────────────────────────
 * @c setDuty() is the fraction of the period the output is HIGH. Tegra's PWM
 * driver does not reliably support the sysfs @c polarity attribute, so no
 * attempt is made to use it. An ACTIVE-LOW load — such as a 74HCT595 output
 * enable, where LOW is lit — must invert at the caller. That inversion belongs
 * with the thing that knows the load is active low, not buried here.
 *
 * @code
 *   dashcam::gpio::PwmPin oe;
 *   oe.open(0, 0, dashcam::gpio::pins::LED595_PWM_HZ, log);
 *   oe.setDuty(1.0f - brightness);   // OE is active low
 *   oe.enable(true);
 * @endcode
 */

#ifndef LIBGPIO_PWM_H
#define LIBGPIO_PWM_H

#include "liblog.h"

#include <cstdint>
#include <string>

namespace dashcam::gpio {

/**
 * @brief One hardware PWM channel.
 *
 * Non-copyable and owns its export: the destructor disables the output and
 * unexports the channel, so a crash-free exit leaves /sys/class/pwm as it was
 * found. An export leaked by a previous run is adopted rather than treated as
 * an error — see open().
 */
class PwmPin {
public:
    PwmPin();
    ~PwmPin();

    PwmPin(const PwmPin&)            = delete;
    PwmPin& operator=(const PwmPin&) = delete;

    /**
     * @brief Exports the channel and sets its carrier frequency.
     *
     * Leaves the output DISABLED with a duty of zero. Callers set a duty and
     * then enable, so the load never sees an unintended full-scale pulse
     * between configuration steps.
     *
     * @param chipIndex  N in /sys/class/pwm/pwmchipN.
     * @param channel    M in pwmchipN/pwmM.
     * @param frequency  Carrier in Hz. Must be > 0.
     */
    bool open(unsigned chipIndex, unsigned channel, unsigned frequencyHz,
              const dashcam::log::LogCallback& log = {});

    /**
     * @brief Changes the carrier frequency, preserving the duty FRACTION.
     *
     * The duty is re-derived from the fraction rather than kept in nanoseconds,
     * because a duty that survived a frequency change unscaled would silently
     * become a different brightness — or exceed the new period and be rejected.
     */
    bool setFrequency(unsigned frequencyHz);

    /**
     * @brief Sets the duty as a fraction of the period, clamped to [0, 1].
     *
     * Fraction of the period the output is HIGH. For an active-low load, invert
     * at the call site.
     */
    bool setDuty(float fraction);

    bool enable(bool on);

    float    duty()      const { return m_duty; }
    unsigned frequency() const { return m_frequencyHz; }
    bool     isOpen()    const { return m_open; }
    bool     isEnabled() const { return m_enabled; }

    /// Disables the output and unexports. Safe to call twice.
    void close();

private:
    bool writeAttr(const std::string& attr, const std::string& value) const;
    bool readAttr (const std::string& attr, std::string& value) const;
    /// Writes then reads back and compares. Used for period and duty, where a
    /// silently rejected value would show up much later as the wrong brightness.
    bool writeAttrVerified(const std::string& attr, uint64_t value) const;
    bool applyPeriodAndDuty(unsigned frequencyHz, float fraction);

    std::string m_chanPath;              ///< /sys/class/pwm/pwmchipN/pwmM
    std::string m_chipPath;              ///< /sys/class/pwm/pwmchipN
    unsigned    m_channel     = 0;
    unsigned    m_frequencyHz = 0;
    float       m_duty        = 0.0f;
    bool        m_open        = false;
    bool        m_enabled     = false;
    bool        m_ownExport   = false;   ///< False when adopting a leaked export.
    dashcam::log::LogCallback m_log;
};

} // namespace dashcam::gpio

#endif // LIBGPIO_PWM_H
