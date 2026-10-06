#ifndef DASHCAM_DR_C_API_H
#define DASHCAM_DR_C_API_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
/* Thin optional binding for the read-only Python demo. C++ callers use the
 * typed API. All angles below are radians; speed is signed m/s. */
typedef struct {
    double x_m, y_m, heading_rad, distance_m, speed_mps, yaw_radps;
    int status, continuous, limited;
} dr_result;
void* dr_create(double wheelbase, double speed_scale, double yaw_scale,
                double mismatch, double tau, double max_speed_mps);
void dr_destroy(void* handle);
int dr_reset(void* handle);
/* gear: 1 forward, -1 reverse, 0 unknown. valid must be 0 or 1.
 * This demo binding never asserts standstill from zero wheel counts. */
int dr_update(void* handle, uint64_t sample_ns, uint64_t now_ns,
              uint16_t rl, uint16_t rr, int gear, int valid, dr_result* out);
const char* dr_status_name(int status);
#ifdef __cplusplus
}
#endif
#endif
