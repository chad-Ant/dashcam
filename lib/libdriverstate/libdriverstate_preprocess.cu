#include "libdriverstate_preprocess.h"

namespace dashcam::driver {

__device__ __forceinline__ static float ldgByte(const uint8_t* p) {
    return static_cast<float>(__ldg(p));
}

__global__ static void preprocessKernel(
    const uint8_t* __restrict__ src,
    float*         __restrict__ dst,
    int srcW,
    int roiX, int roiY, int roiW, int roiH,
    int dstW, int dstH,
    float meanR, float meanG, float meanB,
    float stdR,  float stdG,  float stdB)
{
    const int dx = static_cast<int>(blockIdx.x * blockDim.x + threadIdx.x);
    const int dy = static_cast<int>(blockIdx.y * blockDim.y + threadIdx.y);
    if (dx >= dstW || dy >= dstH) return;

    // Map the destination pixel into the ROI; clamp samples to the ROI so the
    // model never sees pixels outside the crop.
    const float scaleX = static_cast<float>(roiW) / dstW;
    const float scaleY = static_cast<float>(roiH) / dstH;
    const float sx = roiX + (dx + 0.5f) * scaleX - 0.5f;
    const float sy = roiY + (dy + 0.5f) * scaleY - 0.5f;

    const int   ix = static_cast<int>(floorf(sx));
    const int   iy = static_cast<int>(floorf(sy));
    const int   x0 = max(roiX, min(ix,     roiX + roiW - 1));
    const int   x1 = max(roiX, min(ix + 1, roiX + roiW - 1));
    const int   y0 = max(roiY, min(iy,     roiY + roiH - 1));
    const int   y1 = max(roiY, min(iy + 1, roiY + roiH - 1));
    const float fx = sx - static_cast<float>(ix);
    const float fy = sy - static_cast<float>(iy);

    const uint8_t* tl = src + (y0 * srcW + x0) * 3;
    const uint8_t* tr = src + (y0 * srcW + x1) * 3;
    const uint8_t* bl = src + (y1 * srcW + x0) * 3;
    const uint8_t* br = src + (y1 * srcW + x1) * 3;

    const float bv = (1-fy)*((1-fx)*ldgByte(tl)   + fx*ldgByte(tr))
                   +     fy*((1-fx)*ldgByte(bl)   + fx*ldgByte(br));
    const float gv = (1-fy)*((1-fx)*ldgByte(tl+1) + fx*ldgByte(tr+1))
                   +     fy*((1-fx)*ldgByte(bl+1) + fx*ldgByte(br+1));
    const float rv = (1-fy)*((1-fx)*ldgByte(tl+2) + fx*ldgByte(tr+2))
                   +     fy*((1-fx)*ldgByte(bl+2) + fx*ldgByte(br+2));

    const int plane = dstH * dstW;
    const int idx   = dy * dstW + dx;
    dst[0 * plane + idx] = (rv / 255.0f - meanR) / stdR;
    dst[1 * plane + idx] = (gv / 255.0f - meanG) / stdG;
    dst[2 * plane + idx] = (bv / 255.0f - meanB) / stdB;
}

cudaError_t launchPreprocessKernel(
    const uint8_t* dSrc, float* dDst,
    int srcW, int srcH,
    int roiX, int roiY, int roiW, int roiH,
    int dstW, int dstH,
    float meanR, float meanG, float meanB,
    float stdR,  float stdG,  float stdB,
    cudaStream_t stream)
{
    if (roiW < 1 || roiH < 1 || roiX < 0 || roiY < 0 ||
        roiX + roiW > srcW || roiY + roiH > srcH)
        return cudaErrorInvalidValue;

    const dim3 block(16, 16);
    const dim3 grid((dstW + 15) / 16, (dstH + 15) / 16);

    preprocessKernel<<<grid, block, 0, stream>>>(
        dSrc, dDst, srcW, roiX, roiY, roiW, roiH, dstW, dstH,
        meanR, meanG, meanB, stdR, stdG, stdB);

    return cudaGetLastError();
}

} // namespace dashcam::driver
