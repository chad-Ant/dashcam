#pragma once
#include <cuda_runtime.h>
#include <cstdint>

namespace dashcam::driver {

/**
 * @brief Single-pass GPU kernel: bilinear resize + BGR→RGB + normalise + HWC→CHW.
 *
 * @param dSrc  Device pointer to BGR uint8 HWC source.
 * @param dDst  Device pointer to float32 RGB CHW output.
 */
void launchPreprocessKernel(
    const uint8_t* dSrc, float* dDst,
    int srcW, int srcH,
    int dstW, int dstH,
    float meanR, float meanG, float meanB,
    float stdR,  float stdG,  float stdB,
    cudaStream_t stream);

} // namespace dashcam::driver
