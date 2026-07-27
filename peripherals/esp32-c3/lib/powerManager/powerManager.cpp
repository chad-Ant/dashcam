/**
 * @file powerManager.cpp
 * @brief PowerManager implementation — Battery Management System for ESP32-C3
 */

#include "powerManager.h"

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

PowerManager::PowerManager(const PowerManagerConfig& cfg)
    : _cfg(cfg)
    , _bufIdx(0)
    , _bufFull(false)
    , _battV(0.0f)
    , _battPct(0.0f)
    , _tempC(25.0f)
    , _chargeState(CHARGE_IDLE)
    , _battStatus(BATTERY_OK)
    , _autoCharge(false)
    , _lastUpdateMs(0)
{
    for (uint8_t i = 0; i < kFilterLen; ++i) {
        _voltBuf[i] = 0.0f;
    }
}

// ---------------------------------------------------------------------------
// begin()
// ---------------------------------------------------------------------------

bool PowerManager::begin() {
    // --- Validate ADC pin (ADC1 only: GPIO0–4) ----------------------------
    // GPIO5 is ADC2 and is documented as broken in the ESP32-C3 SoC errata.
    if (_cfg.adcPin > 4) {
        return false;
    }

    // --- Validate charge-enable pin (avoid flash pins GPIO11–17) ----------
    if (_cfg.chargeEnPin >= 11 && _cfg.chargeEnPin <= 17) {
        return false;
    }
    if (_cfg.chargeEnPin > 21) {
        return false;
    }
    // Reject same pin for ADC and CE
    if (_cfg.chargeEnPin == _cfg.adcPin) {
        return false;
    }

    // --- Validate voltage span --------------------------------------------
    if (_cfg.emptyVoltage >= _cfg.fullVoltage) {
        return false;
    }
    if (_cfg.voltageDivRatio < 1.0f) {
        return false;
    }

    // --- Validate SOC thresholds (must be strictly ascending) -------------
    if (_cfg.critThreshold < 0.0f || _cfg.critThreshold >= _cfg.lowThreshold) {
        return false;
    }
    if (_cfg.lowThreshold >= _cfg.targetPercent) {
        return false;
    }
    if (_cfg.targetPercent > 100.0f) {
        return false;
    }
    if (_cfg.rechargeHysteresis <= 0.0f || _cfg.rechargeHysteresis > _cfg.targetPercent) {
        return false;
    }

    // --- Validate temperature thresholds ----------------------------------
    if (_cfg.recoverTempC >= _cfg.overTempC) {
        return false;
    }

    // --- Configure ADC ----------------------------------------------------
    // analogReadMilliVolts() handles the eFuse calibration; we only need to
    // set attenuation so the full LiPo range (up to ~fullVoltage / ratio) fits
    // within ~3.1 V — ADC_11db gives the widest input range on the C3.
    analogSetPinAttenuation(_cfg.adcPin, ADC_11db);

    // --- Configure charge-enable pin -------------------------------------
    pinMode(_cfg.chargeEnPin, OUTPUT);
    _setChargePin(false); // safe default: charging off

    // --- Prime the averaging filter with kFilterLen baseline readings ------
    for (uint8_t i = 0; i < kFilterLen; ++i) {
        _voltBuf[i] = static_cast<float>(analogReadMilliVolts(_cfg.adcPin))
                      / 1000.0f * _cfg.voltageDivRatio;
    }
    _bufFull = true;
    _bufIdx  = 0;

    _computeSoc();
    _lastUpdateMs = millis();
    return true;
}

// ---------------------------------------------------------------------------
// update() — non-blocking
// ---------------------------------------------------------------------------

void PowerManager::update() {
    const uint32_t now = millis();
    if ((now - _lastUpdateMs) < _cfg.updateIntervalMs) {
        return;
    }
    _lastUpdateMs = now;

    _sampleVoltage();
    _computeSoc();

    if (_cfg.useInternalTemp) {
        _tempC = temperatureRead();
    }

    _runChargeFsm();
}

// ---------------------------------------------------------------------------
// Getters
// ---------------------------------------------------------------------------

float          PowerManager::getBatteryVoltage() const { return _battV;       }
float          PowerManager::getBatteryPercent()  const { return _battPct;    }
BATTERY_STATUS PowerManager::getBatteryStatus()   const { return _battStatus; }
CHARGE_STATE   PowerManager::getChargeState()     const { return _chargeState;}
float          PowerManager::getTemperature()     const { return _tempC;      }
bool           PowerManager::isCharging()         const { return _chargeState == CHARGE_ACTIVE; }
bool           PowerManager::isLowBattery()       const { return _battPct < _cfg.lowThreshold;  }
bool           PowerManager::isCritical()         const { return _battPct < _cfg.critThreshold; }

// ---------------------------------------------------------------------------
// Control
// ---------------------------------------------------------------------------

void PowerManager::setAutoCharge(bool enable) {
    _autoCharge = enable;
    if (enable) {
        if (_chargeState == CHARGE_DISABLED) {
            _chargeState = CHARGE_IDLE;
        }
    } else {
        _setChargePin(false);
        _chargeState = CHARGE_DISABLED;
    }
}

void PowerManager::forceCharge(bool charge) {
    _setChargePin(charge);
    _chargeState = charge ? CHARGE_ACTIVE : CHARGE_DISABLED;
}

void PowerManager::setChargeTarget(float percent) {
    if (percent > 0.0f && percent <= 100.0f) {
        _cfg.targetPercent = percent;
    }
}

void PowerManager::setExternalTemperature(float tempC) {
    _tempC = tempC;
}

void PowerManager::clearFault() {
    if (_chargeState == CHARGE_FAULT) {
        _chargeState = CHARGE_IDLE;
    }
}

// ---------------------------------------------------------------------------
// Private — _sampleVoltage()
// ---------------------------------------------------------------------------

void PowerManager::_sampleVoltage() {
    // analogReadMilliVolts() uses eFuse calibration (Arduino-ESP32 3.x).
    const float v = static_cast<float>(analogReadMilliVolts(_cfg.adcPin))
                    / 1000.0f * _cfg.voltageDivRatio;

    _voltBuf[_bufIdx] = v;
    _bufIdx = (_bufIdx + 1) % kFilterLen;
    if (_bufIdx == 0) { _bufFull = true; }

    const uint8_t count = _bufFull ? kFilterLen : _bufIdx;
    float sum = 0.0f;
    for (uint8_t i = 0; i < count; ++i) {
        sum += _voltBuf[i];
    }
    _battV = sum / static_cast<float>(count);
}

// ---------------------------------------------------------------------------
// Private — _computeSoc()
// ---------------------------------------------------------------------------

void PowerManager::_computeSoc() {
    const float span = _cfg.fullVoltage - _cfg.emptyVoltage;
    float pct = (_battV - _cfg.emptyVoltage) / span * 100.0f;
    if (pct < 0.0f)   { pct = 0.0f;   }
    if (pct > 100.0f) { pct = 100.0f; }
    _battPct = pct;

    if      (_battPct >= _cfg.targetPercent)  { _battStatus = BATTERY_FULL;     }
    else if (_battPct < _cfg.critThreshold)   { _battStatus = BATTERY_CRITICAL; }
    else if (_battPct < _cfg.lowThreshold)    { _battStatus = BATTERY_LOW;      }
    else                                      { _battStatus = BATTERY_OK;       }
}

// ---------------------------------------------------------------------------
// Private — _runChargeFsm()
// ---------------------------------------------------------------------------

void PowerManager::_runChargeFsm() {
    if (!_autoCharge) {
        return; // manual mode — caller owns the CE pin
    }

    switch (_chargeState) {
        case CHARGE_DISABLED:
            // Transition back to idle when auto-charge is re-enabled
            _chargeState = CHARGE_IDLE;
            break;

        case CHARGE_IDLE:
            // Start charging when SOC drops far enough below the target
            if (_battPct < (_cfg.targetPercent - _cfg.rechargeHysteresis)) {
                _setChargePin(true);
                _chargeState = CHARGE_ACTIVE;
            }
            break;

        case CHARGE_ACTIVE:
            // Stop when target reached
            if (_battPct >= _cfg.targetPercent) {
                _setChargePin(false);
                _chargeState = CHARGE_IDLE;
                break;
            }
            // Suspend on over-temperature
            if (_tempC >= _cfg.overTempC) {
                _setChargePin(false);
                _chargeState = CHARGE_FAULT;
            }
            break;

        case CHARGE_FAULT:
            // Auto-recover once temperature drops and battery still needs charge
            if (_tempC < _cfg.recoverTempC && _battPct < _cfg.targetPercent) {
                _chargeState = CHARGE_IDLE;
            }
            break;
    }
}

// ---------------------------------------------------------------------------
// Private — _setChargePin()
// ---------------------------------------------------------------------------

void PowerManager::_setChargePin(bool on) {
    const bool level = _cfg.chargeEnActiveLow ? !on : on;
    digitalWrite(_cfg.chargeEnPin, level ? HIGH : LOW);
}
