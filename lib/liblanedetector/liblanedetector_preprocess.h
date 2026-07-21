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
 * Vertical sampling reproduces the UFLD v2 training transform
 * (Resize(H/crop_ratio × W) then keep the bottom H rows) as a source-side
 * ROI: output row dy samples source y = srcYOff + (dy+0.5)·srcYScale − 0.5.
 * For a plain full-frame resize pass srcYOff = 0, srcYScale = srcH/dstH.
 *
 * @param dSrc       Device pointer to BGR uint8 HWC source (srcW × srcH × 3 bytes).
 * @param dDst       Device pointer to float32 RGB CHW output (3 × dstH × dstW floats).
 * @param srcYOff    Source-space y offset of the sampled ROI (pixels).
 * @param srcYScale  Source pixels advanced per output row.
 * @return cudaSuccess, or the kernel-launch error (invalid config etc.).
 */
cudaError_t launchPreprocessKernel(
    const uint8_t* dSrc, float* dDst,
    int srcW, int srcH,
    int dstW, int dstH,
    float srcYOff, float srcYScale,
    float meanR, float meanG, float meanB,
    float stdR,  float stdG,  float stdB,
    cudaStream_t stream);

} // namespace dashcam::lane
