#include "liblanedetector_preprocess.h"
#include <cstdio>

namespace dashcam::lane {

// Read-only cache helper: routes global loads through the texture cache,
// which has much higher bandwidth for random-access patterns like bilinear
// interpolation where neighbouring threads hit non-contiguous addresses.
__device__ __forceinline__ static float ldgByte(const uint8_t* p) {
    return static_cast<float>(__ldg(p));
}

// Each thread handles one output pixel.
// One kernel launch replaces: CPU bilinear loop + normalise + channel reorder.
__global__ static void preprocessKernel(
    const uint8_t* __restrict__ src,
    float*         __restrict__ dst,
    int srcW, int srcH,
    int dstW, int dstH,
    float meanR, float meanG, float meanB,
    float stdR,  float stdG,  float stdB)
{
    const int dx = static_cast<int>(blockIdx.x * blockDim.x + threadIdx.x);
    const int dy = static_cast<int>(blockIdx.y * blockDim.y + threadIdx.y);
    if (dx >= dstW || dy >= dstH) return;

    // align_corners=false: map output pixel centre to source pixel centre.
    const float scaleX = static_cast<float>(srcW) / dstW;
    const float scaleY = static_cast<float>(srcH) / dstH;

    const float sx = (dx + 0.5f) * scaleX - 0.5f;
    const float sy = (dy + 0.5f) * scaleY - 0.5f;

    // Use floorf for the integer part so negative coordinates (e.g. sx=-0.25
    // when upsampling) floor to -1, making x0=x1=0 after clamping and giving
    // correct border-pixel clamping.  static_cast<int> truncates toward zero,
    // which would give x0=0 but x1=1 with fx=0.75 — a "smear" artefact.
    const int   ix = static_cast<int>(floorf(sx));
    const int   iy = static_cast<int>(floorf(sy));
    const int   x0 = max(0, min(ix,     srcW - 1));
    const int   x1 = max(0, min(ix + 1, srcW - 1));
    const int   y0 = max(0, min(iy,     srcH - 1));
    const int   y1 = max(0, min(iy + 1, srcH - 1));
    const float fx = sx - static_cast<float>(ix);
    const float fy = sy - static_cast<float>(iy);

    // Read 4 neighbours via __ldg (texture/read-only cache).
    // BGR layout: byte 0 = B, byte 1 = G, byte 2 = R.
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

    // Write CHW RGB (channel 0=R, 1=G, 2=B), normalised.
    const int plane = dstH * dstW;
    const int idx   = dy * dstW + dx;
    dst[0 * plane + idx] = (rv / 255.0f - meanR) / stdR;
    dst[1 * plane + idx] = (gv / 255.0f - meanG) / stdG;
    dst[2 * plane + idx] = (bv / 255.0f - meanB) / stdB;
}

void launchPreprocessKernel(
    const uint8_t* dSrc, float* dDst,
    int srcW, int srcH,
    int dstW, int dstH,
    float meanR, float meanG, float meanB,
    float stdR,  float stdG,  float stdB,
    cudaStream_t stream)
{
    // 16×16 = 256 threads/block — good occupancy for 1024-core Orin Nano GPU.
    const dim3 block(16, 16);
    const dim3 grid((dstW + 15) / 16, (dstH + 15) / 16);

    preprocessKernel<<<grid, block, 0, stream>>>(
        dSrc, dDst,
        srcW, srcH, dstW, dstH,
        meanR, meanG, meanB,
        stdR,  stdG,  stdB);

    // Catch launch errors (invalid grid/block config, resource limits, etc.).
    // Async execution errors surface later at cudaStreamSynchronize.
    const cudaError_t launchErr = cudaGetLastError();
    if (launchErr != cudaSuccess)
        std::fprintf(stderr, "[liblanedetector] preprocess kernel launch failed: %s\n",
                     cudaGetErrorString(launchErr));
}

} // namespace dashcam::lane
