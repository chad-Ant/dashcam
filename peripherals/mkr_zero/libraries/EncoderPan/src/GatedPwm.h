#ifndef DASHCAM_GATED_PWM_H
#define DASHCAM_GATED_PWM_H
#include "EncoderPan.h"
namespace encoderpan {
// Backend: gateOff(), detach(), bool attach(), bool write(us),
// bool enableWhenLow(). Gate MUST default off electrically on MCU reset.
// A gate that only suppresses PWM is NOT a guaranteed torque-off circuit.
template<class Backend> class GatedPwm {
public:
    explicit GatedPwm(Backend& backend) : b_(backend) {}
    void begin(bool installationVerified) {
        off(); verified_ = installationVerified; failed_ = false;
    }
    void off() {
        b_.gateOff();
        if (attached_) b_.detach();
        attached_ = false; enabled_ = false;
    }
    bool apply(const Output& out, uint32_t nowUs) {
        if (!out.enabled) { off(); return !failed_; }
        if (!verified_ || failed_ || out.pulseUs < 600 || out.pulseUs > 2300) return fail();
        if (!attached_) {
            b_.gateOff();
            if (!b_.attach()) return fail();
            attached_ = true; startedUs_ = nowUs;
        }
        if (!b_.write(out.pulseUs)) return fail();
        // Flush Servo library's initial centre pulse behind the disabled gate.
        if (!enabled_ && uint32_t(nowUs-startedUs_) >= 40000)
            enabled_ = b_.enableWhenLow();
        if (!enabled_ && uint32_t(nowUs-startedUs_) > 100000) return fail();
        return true;
    }
    bool enabled() const { return enabled_; }
private:
    bool fail() { failed_ = true; off(); return false; }
    Backend& b_;
    bool verified_ = false, failed_ = false, attached_ = false, enabled_ = false;
    uint32_t startedUs_ = 0;
};
} // namespace encoderpan
#endif
