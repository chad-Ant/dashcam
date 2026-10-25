#include "lane_geometry.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace dashcam::lane {

LaneGeometry computeLaneGeometry(uint32_t srcW, uint32_t srcH,
                                 const LaneDetectorConfig& cfg) {
    if (srcW == 0 || srcH == 0)
        throw std::runtime_error("liblanedetector: source dimensions are 0");
    if (cfg.numLanes != 4)
        throw std::runtime_error("liblanedetector: UFLD v2 decode requires "
                                 "numLanes == 4 (slots 1,2 row / 0,3 col)");
    if (cfg.numRowAnchors < 2 || cfg.numColAnchors < 2 ||
        cfg.numCellRow < 2 || cfg.numCellCol < 2)
        throw std::runtime_error("liblanedetector: head geometry must be >= 2");
    if (!(cfg.cropRatio > 0.0f) || cfg.cropRatio > 1.0f)
        throw std::runtime_error("liblanedetector: cropRatio must be in (0, 1]");
    if (cfg.inputCropTop < 0.0f || cfg.inputCropTop > 0.9f)
        throw std::runtime_error("liblanedetector: inputCropTop must be in [0, 0.9]");
    if (cfg.inputCropBottom < 0.0f || cfg.inputCropBottom > 0.9f)
        throw std::runtime_error("liblanedetector: inputCropBottom must be in [0, 0.9]");

    LaneGeometry g;
    g.srcW = srcW;
    g.srcH = srcH;

    // Branch crop: even row counts for NVMM/NV12 chroma alignment.
    g.cropTopPx    = static_cast<uint32_t>(
        std::lround(srcH * cfg.inputCropTop)) & ~1u;
    g.cropBottomPx = static_cast<uint32_t>(
        std::lround(srcH * cfg.inputCropBottom)) & ~1u;
    if (g.cropTopPx + g.cropBottomPx + 64 > srcH)
        throw std::runtime_error("liblanedetector: crop leaves fewer than "
                                 "64 rows of frame");
    g.branchW = srcW;
    g.branchH = srcH - g.cropTopPx - g.cropBottomPx;

    // The model should see the bottom cropRatio of the ORIGINAL frame.
    // Inside the branch-cropped band that region is cropRatio/keptFrac of
    // the height — clamped to 1 when the branch already cropped more than
    // the training transform would have (then the whole band is used).
    // A bottom crop does NOT always clamp: top 0 / bottom 0.2 keeps 0.8 of
    // the frame and gives effRatio 0.75.
    g.keptFrac = static_cast<float>(g.branchH) / srcH;
    g.effRatio = std::min(1.0f, cfg.cropRatio / g.keptFrac);
    g.resizedH = static_cast<int>(std::lround(cfg.modelInputH / g.effRatio));
    g.cropTopRows = g.resizedH - static_cast<int>(cfg.modelInputH);
    g.preYScale = static_cast<float>(g.branchH) / static_cast<float>(g.resizedH);
    g.preYOff   = static_cast<float>(g.cropTopRows) * g.preYScale;

    // The model reads its input as the bottom cropRatio of a training frame
    // trainH rows tall, whatever the deployment geometry: model row m is
    // implied fraction (trainTop + m) / trainH, and branch row
    // preYOff + m·preYScale (edge coordinates).
    const double trainH   = static_cast<double>(cfg.modelInputH)
                          / static_cast<double>(cfg.cropRatio);
    const double trainTop = trainH - static_cast<double>(cfg.modelInputH);
    g.impliedH      = trainH * g.preYScale;
    g.impliedTopSrc = static_cast<double>(g.cropTopPx) + g.preYOff
                    - trainTop * g.preYScale;
    g.refRowSrc         = g.impliedToSourceRow(cfg.laneReferenceY);
    g.anchorRowSrcFirst = g.impliedToSourceRow(cfg.rowAnchorStart);
    g.anchorRowSrcLast  = g.impliedToSourceRow(1.0);
    return g;
}

} // namespace dashcam::lane
