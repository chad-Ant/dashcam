#pragma once
#include <cuda_runtime.h>
#include <cstdint>

namespace dashcam::lane {

/**
 * @brief Single-pass GPU kernel: bilinear resize + BGR→RGB + per-channel
 *        normalise + HWC→CHW layout, writing directly into the TensorRT
 *        input buffer.
 *
 * Replaces all CPU preprocessing.  On Orin Nano this runs in ~0.5 ms vs
 * ~30 ms for the equivalent single-threaded C++ loop.
 *
 * @param dSrc   Device pointer to BGR uint8 HWC source   (srcW × srcH × 3 bytes).
 * @param dDst   Device pointer to float32 RGB CHW output  (3 × dstH × dstW floats).
 */
void launchPreprocessKernel(
    const uint8_t* dSrc, float* dDst,
    int srcW, int srcH,
    int dstW, int dstH,
    float meanR, float meanG, float meanB,
    float stdR,  float stdG,  float stdB,
    cudaStream_t stream);

} // namespace dashcam::lane
