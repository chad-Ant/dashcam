#pragma once
#include <cuda_runtime.h>
#include <cstdint>

namespace dashcam::driver {

/**
 * @brief Single-pass GPU kernel: ROI crop + bilinear resize + BGR→RGB +
 *        ImageNet normalise + HWC→CHW.
 *
 * The ROI (typically a centred square, matching the square face crops the
 * drowsiness model was trained on) is resampled to dstW×dstH; sampling is
 * clamped to the ROI so no pixels outside it bleed into the model input.
 *
 * @param dSrc  Device pointer to BGR uint8 HWC source (srcW×srcH).
 * @param dDst  Device pointer to float32 RGB CHW output (3×dstH×dstW).
 * @param roiX,roiY,roiW,roiH  Source crop rectangle; must lie within the
 *              source frame and have roiW,roiH >= 1.
 * @return cudaSuccess, or the kernel launch error.
 */
cudaError_t launchPreprocessKernel(
    const uint8_t* dSrc, float* dDst,
    int srcW, int srcH,
    int roiX, int roiY, int roiW, int roiH,
    int dstW, int dstH,
    float meanR, float meanG, float meanB,
    float stdR,  float stdG,  float stdB,
    cudaStream_t stream);

} // namespace dashcam::driver
