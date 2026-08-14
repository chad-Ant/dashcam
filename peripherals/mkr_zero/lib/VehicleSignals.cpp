#include "VehicleSignals.h"

void expireVehicleSignals(VehicleSignals &v, uint32_t nowMs,
                          uint32_t sniffMs, uint32_t obd2Ms)
{
    // A stamp of 0 means "never written", which is already represented by the
    // field's own unavailable value. Expiring it would be a no-op at best and,
    // for the integer fields, would look like a transition from valid to stale.
    #define VEH_EXPIRE(stampField, srcField, action)                          \
        do {                                                                  \
            if ((stampField) != 0u && (srcField) != VehSource::NONE) {        \
                const uint32_t limit = ((srcField) == VehSource::CAN_SNIFF)   \
                                     ? sniffMs : obd2Ms;                      \
                if ((nowMs - (stampField)) > limit) {                         \
                    action;                                                   \
                    (srcField) = VehSource::NONE;                             \
                }                                                             \
            }                                                                 \
        } while (0)

    // The window is chosen per field from its OWN source, not once for the
    // struct. Sniffed IDs repeat every 10-20 ms so 200 ms of silence is a real
    // outage; an OBD-II field is expected to be almost a full poll cycle old,
    // and holding it to the sniff window would blank a perfectly healthy
    // fallback mode.
    VEH_EXPIRE(v.speedMs, v.speedSrc, v.speedKmh = NAN);
    VEH_EXPIRE(v.rpmMs,   v.rpmSrc,   v.rpm      = NAN);
    VEH_EXPIRE(v.gearMs,  v.gearSrc,  v.gear     = VehGear::UNKNOWN);
    VEH_EXPIRE(v.steerMs, v.steerSrc, v.steerMotorTorque = VEH_TORQUE_INVALID);
    VEH_EXPIRE(v.yawMs,   v.yawSrc,   v.yawRateCdps      = INT16_MIN);
    VEH_EXPIRE(v.pedalMs, v.pedalSrc, v.pedalGas = 0);
    VEH_EXPIRE(v.brakeMs, v.brakeSrc, v.brakePressed = false; v.brakeSwitch = false);
    // Stamped on every frame of the indicator message, NOT on every flash - the
    // message keeps arriving at ~24 Hz whether the lamp is lit or dark, so this
    // expires when the ID goes away, which is what staleness means here. The
    // blink itself is handled by the hold in the decoder.
    VEH_EXPIRE(v.turnMs,  v.turnSrc,  v.turnLeft = false; v.turnRight = false);
    // Its own clock: the map may put hazards on a different ID from the stalk.
    VEH_EXPIRE(v.hazardMs, v.hazardSrc, v.hazard = false);
    VEH_EXPIRE(v.wheelMs, v.wheelSrc,
               for (uint8_t i = 0; i < VEH_WHEEL_COUNT; ++i) v.wheelRaw[i] = VEH_WHEEL_INVALID);

    #undef VEH_EXPIRE
}

uint8_t packVehSourceByte(const VehicleSignals &v)
{
    // Two bits each, low pair first: speed | rpm | gear | steer.
    return (uint8_t)(((uint8_t)v.speedSrc & 0x03u)        |
                     (((uint8_t)v.rpmSrc   & 0x03u) << 2) |
                     (((uint8_t)v.gearSrc  & 0x03u) << 4) |
                     (((uint8_t)v.steerSrc & 0x03u) << 6));
}
