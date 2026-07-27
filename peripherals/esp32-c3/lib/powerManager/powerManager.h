/**
 * @file powerManager.h
 * @brief Lightweight Battery Management System (BMS) for ESP32-C3 (Arduino framework)
 *
 * @details
 * Provides non-blocking battery-voltage measurement via an ADC voltage divider,
 * state-of-charge (SOC) estimation, on-chip die-temperature monitoring, and
 * charge-enable control for an external charger IC (e.g. TP4056, MCP73831).
 *
 * ### Hardware constraints observed
 * - ADC pin **must** be GPIO0–GPIO4 (ADC1 only). GPIO5/ADC2 is broken on the
 *   ESP32-C3 (documented SoC erratum) and is rejected in begin().
 * - Charge-enable pin must be a safe GPIO (0–10 or 20–21). Flash pins
 *   GPIO11–GPIO17 are rejected in begin().
 * - Voltage readings use `analogReadMilliVolts()` (Arduino-ESP32 3.x), which
 *   applies the eFuse calibration automatically — more accurate than raw ADC.
 * - Temperature via `temperatureRead()` returns ESP32-C3 **die** temperature,
 *   not ambient. Useful for thermal-fault detection, not room sensing.
 *
 * ### Resource footprint
 * - RAM  : ~80 B instance + 16 B ring-buffer — fixed, zero heap after begin().
 * - Flash : ~2 KB (estimate with -Os).
 * - CPU  : one `analogReadMilliVolts()` call per `updateIntervalMs`; otherwise
 *          a single subtraction and comparison per loop() pass.
 *
 * ### Typical usage
 * @code
 * #include <powerManager.h>
 *
 * PowerManagerConfig cfg;
 * cfg.adcPin            = 0;       // GPIO0 — battery divider midpoint
 * cfg.voltageDivRatio   = 2.0f;    // two equal resistors (R1=R2)
 * cfg.fullVoltage       = 4.20f;
 * cfg.emptyVoltage      = 3.00f;
 * cfg.lowThreshold      = 20.0f;
 * cfg.critThreshold     = 10.0f;
 * cfg.targetPercent     = 90.0f;
 * cfg.rechargeHysteresis = 5.0f;
 * cfg.overTempC         = 45.0f;
 * cfg.recoverTempC      = 40.0f;
 * cfg.chargeEnPin       = 6;
 * cfg.chargeEnActiveLow = false;
 * cfg.updateIntervalMs  = 5000;
 * cfg.useInternalTemp   = true;
 *
 * PowerManager bms(cfg);
 *
 * void setup() {
 *   if (!bms.begin()) { ... }
 *   bms.setAutoCharge(true);
 * }
 * void loop() {
 *   bms.update();   // non-blocking; call every iteration
 * }
 * @endcode
 *
 * @note Compile with `-Wall -Wextra` (Arduino IDE: Preferences → Compiler warnings = All).
 *
 * @author   ESP_Sentinel
 * @version  1.0.0
 */

#pragma once
#include <Arduino.h>

// ---------------------------------------------------------------------------
// Public enumerations  (all-caps per project convention)
// ---------------------------------------------------------------------------

/**
 * @brief Coarse battery-level category.
 *
 * Derived from SOC thresholds set in PowerManagerConfig.
 */
enum BATTERY_STATUS : uint8_t {
    BATTERY_CRITICAL = 0, ///< SOC below critThreshold — imminent shutdown
    BATTERY_LOW,          ///< SOC below lowThreshold  — warn / shed load
    BATTERY_OK,           ///< Normal operating range
    BATTERY_FULL          ///< SOC at or above targetPercent
};

/**
 * @brief State of the internal charge-control finite state machine.
 */
enum CHARGE_STATE : uint8_t {
    CHARGE_DISABLED = 0, ///< Auto-charge off; CE pin de-asserted
    CHARGE_IDLE,         ///< Auto-charge on; waiting for SOC to drop
    CHARGE_ACTIVE,       ///< Charging in progress; CE pin asserted
    CHARGE_FAULT         ///< Over-temperature fault; CE de-asserted until recovery
};

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/**
 * @brief All tunable parameters for PowerManager.
 *
 * Populate every field, then pass to the PowerManager constructor.
 * After begin() returns, changes to this struct have no effect unless
 * begin() is called again.
 */
struct PowerManagerConfig {
    /** GPIO connected to the battery-voltage divider midpoint. Must be GPIO0–4. */
    uint8_t adcPin;

    /**
     * Voltage-divider scale factor: (R1 + R2) / R2.
     *
     * For two equal resistors (e.g. 100 kΩ + 100 kΩ) this is 2.0.
     * Must be ≥ 1.0.
     */
    float voltageDivRatio;

    /** Battery voltage (V) representing 100 % SOC. Typical LiPo: 4.20. */
    float fullVoltage;

    /** Battery voltage (V) representing 0 % SOC. Typical LiPo: 3.00. */
    float emptyVoltage;

    /** SOC [0–100] below which getBatteryStatus() returns BATTERY_LOW. */
    float lowThreshold;

    /** SOC [0–100] below which getBatteryStatus() returns BATTERY_CRITICAL. Must be < lowThreshold. */
    float critThreshold;

    /**
     * SOC [0–100] at which auto-charge stops. Must be > lowThreshold.
     *
     * Charging to 90 % rather than 100 % significantly extends cell life.
     */
    float targetPercent;

    /**
     * SOC drop below targetPercent required to restart auto-charging.
     *
     * Prevents short charge bursts near the target level. E.g. target 90 %,
     * hysteresis 5 % → charging restarts at 85 %.
     */
    float rechargeHysteresis;

    /** Die temperature (°C) at which charging is suspended. Typical: 45.0. */
    float overTempC;

    /** Die temperature (°C) below which charging may resume after a fault. Typical: 40.0. */
    float recoverTempC;

    /** GPIO connected to the charger IC's charge-enable (CE) pin. Must be GPIO0–10 or 20–21. */
    uint8_t chargeEnPin;

    /** Set true if the charger's CE pin is active-LOW (output is inverted). */
    bool chargeEnActiveLow;

    /** Milliseconds between ADC samples. Values of 1 000–10 000 suit most applications. */
    uint32_t updateIntervalMs;

    /**
     * When true, getTemperature() returns the ESP32-C3 on-chip die reading via
     * temperatureRead(). When false, the caller must supply readings via
     * setExternalTemperature() before temperature-based fault logic is meaningful.
     */
    bool useInternalTemp;
};

// ---------------------------------------------------------------------------
// PowerManager
// ---------------------------------------------------------------------------

/**
 * @brief Lightweight Battery Management System for ESP32-C3.
 *
 * Call begin() once in setup(), then call update() every loop() iteration.
 * All getters are non-blocking and safe to call at any time after begin().
 *
 * The internal charge FSM has four states:
 * ```
 *  DISABLED ──(setAutoCharge(true))──► IDLE
 *  IDLE     ──(SOC < target−hyst)───► ACTIVE
 *  ACTIVE   ──(SOC ≥ target)─────────► IDLE
 *  ACTIVE   ──(temp > overTempC)─────► FAULT
 *  FAULT    ──(temp < recoverTempC
 *              && SOC < target)───────► IDLE
 * ```
 */
class PowerManager {
public:
    /**
     * @brief Construct a PowerManager with the given configuration.
     * @param cfg Fully populated PowerManagerConfig. Copied internally.
     */
    explicit PowerManager(const PowerManagerConfig& cfg);

    /**
     * @brief Initialise ADC, charge-enable GPIO, and prime the voltage filter.
     *
     * Validates all configuration values against ESP32-C3 hardware constraints.
     * Performs kFilterLen ADC reads to fill the averaging buffer so that the
     * first getBatteryVoltage() call returns a meaningful value.
     *
     * @return true  on success.
     * @return false if any configuration value violates a hardware constraint
     *               or a logical invariant (e.g. emptyVoltage ≥ fullVoltage).
     *               No peripheral is modified on false.
     */
    bool begin();

    /**
     * @brief Non-blocking periodic update. Call once per loop() iteration.
     *
     * Samples the ADC at the configured interval, recomputes SOC and status,
     * optionally reads the on-chip temperature, then runs the charge FSM.
     * Does nothing between intervals (pure millis() comparison, no delay).
     */
    void update();

    // ---- Getters -----------------------------------------------------------

    /**
     * @brief Averaged battery terminal voltage in Volts.
     * @return Float voltage; 0.0 before begin() is called.
     */
    float getBatteryVoltage() const;

    /**
     * @brief State of charge in percent [0.0 – 100.0].
     *
     * Linear interpolation between emptyVoltage (0 %) and fullVoltage (100 %),
     * clamped at the endpoints.
     *
     * @return SOC as a percentage.
     */
    float getBatteryPercent() const;

    /**
     * @brief Coarse battery-level category.
     * @return One of BATTERY_CRITICAL, BATTERY_LOW, BATTERY_OK, BATTERY_FULL.
     */
    BATTERY_STATUS getBatteryStatus() const;

    /**
     * @brief Current charge-FSM state.
     * @return One of CHARGE_DISABLED, CHARGE_IDLE, CHARGE_ACTIVE, CHARGE_FAULT.
     */
    CHARGE_STATE getChargeState() const;

    /**
     * @brief Temperature reading in °C.
     *
     * Returns the ESP32-C3 die temperature when useInternalTemp is true,
     * or the last value supplied via setExternalTemperature() when false.
     * Initialised to 25.0 °C before the first reading.
     *
     * @return Temperature in °C.
     */
    float getTemperature() const;

    /** @return true while the charge FSM is in CHARGE_ACTIVE state. */
    bool isCharging() const;

    /** @return true when SOC is below lowThreshold. */
    bool isLowBattery() const;

    /** @return true when SOC is below critThreshold. */
    bool isCritical() const;

    // ---- Control -----------------------------------------------------------

    /**
     * @brief Enable or disable automatic charge control.
     *
     * When enabled, update() manages the CE pin autonomously.
     * When disabled, the CE pin is immediately de-asserted and the FSM
     * transitions to CHARGE_DISABLED, where it stays until re-enabled.
     *
     * @param enable true = auto-charge mode; false = manual/off.
     */
    void setAutoCharge(bool enable);

    /**
     * @brief Directly force the charge-enable pin state.
     *
     * Bypasses the auto-charge FSM. Call setAutoCharge(false) first to
     * prevent the FSM from overriding this on the next update().
     *
     * @param charge true = assert CE (charge); false = de-assert CE (idle).
     */
    void forceCharge(bool charge);

    /**
     * @brief Change the SOC target at which auto-charging stops.
     *
     * Silently ignored if percent is outside (0.0, 100.0].
     *
     * @param percent New target SOC [0.0 – 100.0].
     */
    void setChargeTarget(float percent);

    /**
     * @brief Inject an external temperature reading.
     *
     * Use when useInternalTemp is false (e.g. an NTC on an I2C gauge IC).
     * The fault FSM uses this value, so call it before update() each cycle.
     *
     * @param tempC Temperature in °C.
     */
    void setExternalTemperature(float tempC);

    /**
     * @brief Manually clear a CHARGE_FAULT state.
     *
     * The FSM clears faults automatically once temperature recovers, but this
     * allows an immediate reset when the caller has confirmed safety.
     * Has no effect unless the current state is CHARGE_FAULT.
     */
    void clearFault();

private:
    /** Depth of the ring-buffer used for voltage averaging. Fixed at compile time. */
    static constexpr uint8_t kFilterLen = 4;

    PowerManagerConfig _cfg;

    float   _voltBuf[kFilterLen]; ///< Circular voltage buffer (Volts)
    uint8_t _bufIdx;              ///< Next write slot in _voltBuf
    bool    _bufFull;             ///< True once all kFilterLen slots have been written

    float _battV;    ///< Averaged battery voltage (V)
    float _battPct;  ///< Battery state of charge (%)
    float _tempC;    ///< Temperature reading (°C)

    CHARGE_STATE   _chargeState;
    BATTERY_STATUS _battStatus;

    bool     _autoCharge;   ///< Auto-charge mode flag
    uint32_t _lastUpdateMs; ///< millis() at last sample

    /** Sample ADC, push into ring buffer, recompute _battV. */
    void _sampleVoltage();

    /** Recompute _battPct and _battStatus from _battV. */
    void _computeSoc();

    /** Advance the charge control FSM and drive the CE pin. */
    void _runChargeFsm();

    /**
     * Drive the charge-enable pin, honouring chargeEnActiveLow.
     * @param on true = assert CE (enable charging).
     */
    void _setChargePin(bool on);
};
