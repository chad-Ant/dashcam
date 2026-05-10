#include "libstereocam.h"

#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>

// OpenCV: calibration loading and CPU SGM backend.
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

// VPI 3.x: CUDA SGM backend.
#include <vpi/Image.h>
#include <vpi/Stream.h>
#include <vpi/algo/StereoDisparity.h>

namespace dashcam::stereo {

// ─── file-local log helper ────────────────────────────────────────────────────

static void doLog(const dashcam::log::LogCallback& cb, dashcam::log::LogLevel lvl,
                  const char* fmt, ...) {
    if (!cb) return;
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    std::vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    cb(lvl, buf);
}

// ─── helpers ─────────────────────────────────────────────────────────────────

void StereoRangefinder::bgrToGray(const uint8_t* bgr, uint8_t* gray, int width, int height) {
    cv::Mat bgrMat(height, width, CV_8UC3, const_cast<uint8_t*>(bgr));
    cv::Mat grayMat(height, width, CV_8UC1, gray);
    cv::cvtColor(bgrMat, grayMat, cv::COLOR_BGR2GRAY);
}

void StereoRangefinder::applyRectification(const uint8_t* src,
                                            const std::vector<float>& mapX,
                                            const std::vector<float>& mapY,
                                            uint8_t* dst, int width, int height) {
    // Zero-copy wrappers; remap only reads srcMat and the map Mats.
    cv::Mat srcMat(height, width, CV_8UC1, const_cast<uint8_t*>(src));
    cv::Mat dstMat(height, width, CV_8UC1, dst);
    cv::Mat mx(height, width, CV_32FC1, const_cast<float*>(mapX.data()));
    cv::Mat my(height, width, CV_32FC1, const_cast<float*>(mapY.data()));
    cv::remap(srcMat, dstMat, mx, my, cv::INTER_LINEAR, cv::BORDER_CONSTANT, 0);
}

// ─── VPI lifecycle ────────────────────────────────────────────────────────────

bool StereoRangefinder::initVpi() {
    const int w = calib_.imageWidth;
    const int h = calib_.imageHeight;

    if (vpiStreamCreate(0, reinterpret_cast<VPIStream*>(&vpiStream_)) != VPI_SUCCESS) {
        doLog(log_, dashcam::log::LogLevel::ERROR, "vpiStreamCreate failed");
        return false;
    }

    // Regular device images — locked for CPU write each frame, then submitted to CUDA.
    if (vpiImageCreate(w, h, VPI_IMAGE_FORMAT_U8, 0,
                       reinterpret_cast<VPIImage*>(&vpiLeft_)) != VPI_SUCCESS ||
        vpiImageCreate(w, h, VPI_IMAGE_FORMAT_U8, 0,
                       reinterpret_cast<VPIImage*>(&vpiRight_)) != VPI_SUCCESS ||
        vpiImageCreate(w, h, VPI_IMAGE_FORMAT_S16, 0,
                       reinterpret_cast<VPIImage*>(&vpiDisparity_)) != VPI_SUCCESS) {
        doLog(log_, dashcam::log::LogLevel::ERROR, "vpiImageCreate failed");
        destroyVpi();
        return false;
    }

    VPIStereoDisparityEstimatorCreationParams params = {};
    vpiInitStereoDisparityEstimatorCreationParams(&params);
    params.maxDisparity = 64;  // adequate for narrow-baseline webcam pair at 640×360

    if (vpiCreateStereoDisparityEstimator(
            VPI_BACKEND_CUDA, w, h,
            VPI_IMAGE_FORMAT_U8, VPI_IMAGE_FORMAT_S16,
            &params,
            reinterpret_cast<VPIPayload*>(&vpiPayload_)) != VPI_SUCCESS) {
        doLog(log_, dashcam::log::LogLevel::ERROR,
              "vpiCreateStereoDisparityEstimator failed");
        destroyVpi();
        return false;
    }

    return true;
}

void StereoRangefinder::destroyVpi() {
    if (vpiPayload_)   { vpiPayloadDestroy(*reinterpret_cast<VPIPayload*>(&vpiPayload_));    vpiPayload_   = nullptr; }
    if (vpiDisparity_) { vpiImageDestroy(*reinterpret_cast<VPIImage*>(&vpiDisparity_));      vpiDisparity_ = nullptr; }
    if (vpiRight_)     { vpiImageDestroy(*reinterpret_cast<VPIImage*>(&vpiRight_));           vpiRight_     = nullptr; }
    if (vpiLeft_)      { vpiImageDestroy(*reinterpret_cast<VPIImage*>(&vpiLeft_));            vpiLeft_      = nullptr; }
    if (vpiStream_)    { vpiStreamDestroy(*reinterpret_cast<VPIStream*>(&vpiStream_));        vpiStream_    = nullptr; }
}

// ─── constructor / destructor ─────────────────────────────────────────────────

StereoRangefinder::StereoRangefinder(dashcam::camera::iCamera* left,
                                     dashcam::camera::iCamera* right,
                                     Backend backend)
    : left_(left), right_(right),
      requestedBackend_(backend), activeBackend_(backend) {
}

StereoRangefinder::~StereoRangefinder() {
    destroyVpi();
}

void StereoRangefinder::setLogCallback(dashcam::log::LogCallback cb) {
    log_ = std::move(cb);
}

// ─── calibration ─────────────────────────────────────────────────────────────

bool StereoRangefinder::loadCalibration(const std::string& yamlPath) {
    cv::FileStorage fs(yamlPath, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        doLog(log_, dashcam::log::LogLevel::ERROR,
              "loadCalibration: cannot open %s", yamlPath.c_str());
        return false;
    }

    cv::Mat M1, D1, M2, D2, R, T;
    fs["M1"] >> M1;
    fs["D1"] >> D1;
    fs["M2"] >> M2;
    fs["D2"] >> D2;
    fs["R"]  >> R;
    fs["T"]  >> T;
    int w = 0, h = 0;
    fs["imageWidth"]  >> w;
    fs["imageHeight"] >> h;
    fs.release();

    if (M1.empty() || D1.empty() || M2.empty() || D2.empty() ||
        R.empty()  || T.empty()  || w == 0 || h == 0) {
        doLog(log_, dashcam::log::LogLevel::ERROR,
              "loadCalibration: missing required key(s) in %s", yamlPath.c_str());
        return false;
    }

    // Ensure double precision for rectification.
    M1.convertTo(M1, CV_64F);
    D1.convertTo(D1, CV_64F);
    M2.convertTo(M2, CV_64F);
    D2.convertTo(D2, CV_64F);
    R.convertTo(R,   CV_64F);
    T.convertTo(T,   CV_64F);

    const cv::Size imgSize(w, h);
    cv::Mat R1, R2, P1, P2, Q;
    cv::stereoRectify(M1, D1, M2, D2, imgSize, R, T, R1, R2, P1, P2, Q,
                      cv::CALIB_ZERO_DISPARITY, -1, imgSize);

    calib_.imageWidth  = w;
    calib_.imageHeight = h;
    calib_.focalPx  = P1.at<double>(0, 0);
    calib_.baselineM = -P2.at<double>(0, 3) / P2.at<double>(0, 0);

    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            calib_.Q[i * 4 + j] = Q.at<double>(i, j);

    // Precompute float remap arrays (CV_32FC1, row-major).
    cv::Mat lmx, lmy, rmx, rmy;
    cv::initUndistortRectifyMap(M1, D1, R1, P1, imgSize, CV_32FC1, lmx, lmy);
    cv::initUndistortRectifyMap(M2, D2, R2, P2, imgSize, CV_32FC1, rmx, rmy);

    const std::size_t npx = static_cast<std::size_t>(w * h);
    calib_.leftMapX.assign(reinterpret_cast<const float*>(lmx.data),
                           reinterpret_cast<const float*>(lmx.data) + npx);
    calib_.leftMapY.assign(reinterpret_cast<const float*>(lmy.data),
                           reinterpret_cast<const float*>(lmy.data) + npx);
    calib_.rightMapX.assign(reinterpret_cast<const float*>(rmx.data),
                            reinterpret_cast<const float*>(rmx.data) + npx);
    calib_.rightMapY.assign(reinterpret_cast<const float*>(rmy.data),
                            reinterpret_cast<const float*>(rmy.data) + npx);

    calibLoaded_ = true;
    return true;
}

// ─── configuration ────────────────────────────────────────────────────────────

void StereoRangefinder::setFormatIndex(uint16_t leftIdx, uint16_t rightIdx) {
    leftFmtIdx_  = leftIdx;
    rightFmtIdx_ = rightIdx;
}

// ─── lifecycle ────────────────────────────────────────────────────────────────

StereoError StereoRangefinder::open() {
    left_->open();
    right_->open();
    left_->setCameraVideoFormat(leftFmtIdx_);
    right_->setCameraVideoFormat(rightFmtIdx_);

    dashcam::camera::cameraStatus ls, rs;
    left_->getCameraStatus(ls);
    right_->getCameraStatus(rs);
    if (ls.status == dashcam::camera::CAMERA_STATUS::ERROR ||
        rs.status == dashcam::camera::CAMERA_STATUS::ERROR)
        return StereoError::CAMERA_FAILED;

    return StereoError::NONE;
}

StereoError StereoRangefinder::start() {
    if (!calibLoaded_) {
        doLog(log_, dashcam::log::LogLevel::ERROR,
              "start() called before loadCalibration()");
        return StereoError::NOT_CALIBRATED;
    }

    const std::size_t npx    = static_cast<std::size_t>(calib_.imageWidth * calib_.imageHeight);
    const std::size_t bgrSz  = npx * 3u;

    leftBuf_.resize(bgrSz);
    rightBuf_.resize(bgrSz);
    leftGray_.resize(npx);
    rightGray_.resize(npx);
    leftRect_.resize(npx);
    rightRect_.resize(npx);

    if (requestedBackend_ == Backend::VPI_CUDA) {
        if (!initVpi()) {
            doLog(log_, dashcam::log::LogLevel::WARN,
                  "VPI init failed — falling back to OpenCV CPU");
            activeBackend_ = Backend::OpenCV_CPU;
        } else {
            activeBackend_ = Backend::VPI_CUDA;
        }
    } else {
        activeBackend_ = Backend::OpenCV_CPU;
    }

    left_->start();
    right_->start();

    dashcam::camera::cameraStatus ls, rs;
    left_->getCameraStatus(ls);
    right_->getCameraStatus(rs);
    if (ls.status != dashcam::camera::CAMERA_STATUS::RUNNING ||
        rs.status != dashcam::camera::CAMERA_STATUS::RUNNING) {
        destroyVpi();
        return StereoError::CAMERA_FAILED;
    }

    return StereoError::NONE;
}

StereoError StereoRangefinder::stop() {
    destroyVpi();
    left_->stop();
    right_->stop();
    return StereoError::NONE;
}

StereoError StereoRangefinder::close() {
    left_->close();
    right_->close();
    return StereoError::NONE;
}

// ─── capture + dispatch ───────────────────────────────────────────────────────

bool StereoRangefinder::computeDepth(DepthResult& result, int syncToleranceMs) {
    if (!calibLoaded_) return false;

    const int      w      = calib_.imageWidth;
    const int      h      = calib_.imageHeight;
    const uint32_t bufSz  = static_cast<uint32_t>(w * h * 3);

    uint32_t lWritten = 0, rWritten = 0;

    left_->captureFrame(leftBuf_.data(), bufSz, lWritten);
    const int64_t lTs = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();

    right_->captureFrame(rightBuf_.data(), bufSz, rWritten);
    const int64_t rTs = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();

    if (lWritten == 0 || rWritten == 0) return false;

    result.leftTsMs    = lTs;
    result.rightTsMs   = rTs;
    result.timeDeltaMs = std::abs(lTs - rTs);
    result.width       = w;
    result.height      = h;
    result.valid       = false;

    if (result.timeDeltaMs > static_cast<int64_t>(syncToleranceMs))
        doLog(log_, dashcam::log::LogLevel::WARN,
              "frame desync: %lld ms", static_cast<long long>(result.timeDeltaMs));

    // BGR → grey
    bgrToGray(leftBuf_.data(),  leftGray_.data(),  w, h);
    bgrToGray(rightBuf_.data(), rightGray_.data(), w, h);

    // Rectify
    applyRectification(leftGray_.data(),  calib_.leftMapX,  calib_.leftMapY,  leftRect_.data(),  w, h);
    applyRectification(rightGray_.data(), calib_.rightMapX, calib_.rightMapY, rightRect_.data(), w, h);

    if (activeBackend_ == Backend::VPI_CUDA)
        return computeDepthVpi(result);
    return computeDepthCpu(result);
}

// ─── VPI backend ──────────────────────────────────────────────────────────────

bool StereoRangefinder::computeDepthVpi(DepthResult& result) {
    const int      w   = calib_.imageWidth;
    const int      h   = calib_.imageHeight;
    VPIImage       lImg = *reinterpret_cast<VPIImage*>(&vpiLeft_);
    VPIImage       rImg = *reinterpret_cast<VPIImage*>(&vpiRight_);
    VPIImage       dImg = *reinterpret_cast<VPIImage*>(&vpiDisparity_);
    VPIStream      str  = *reinterpret_cast<VPIStream*>(&vpiStream_);
    VPIPayload     pld  = *reinterpret_cast<VPIPayload*>(&vpiPayload_);

    // Upload rectified grey into VPI images via CPU lock.
    // Copy row-by-row to honour the hardware pitch (row stride may exceed width).
    auto upload = [&](VPIImage img, const uint8_t* data) -> bool {
        VPIImageData d = {};
        if (vpiImageLockData(img, VPI_LOCK_WRITE,
                             VPI_IMAGE_BUFFER_HOST_PITCH_LINEAR, &d) != VPI_SUCCESS)
            return false;
        uint8_t*       dst   = static_cast<uint8_t*>(d.buffer.pitch.planes[0].data);
        const int32_t  pitch = d.buffer.pitch.planes[0].pitchBytes;
        for (int y = 0; y < h; ++y)
            std::memcpy(dst + y * pitch, data + y * w, static_cast<std::size_t>(w));
        vpiImageUnlock(img);
        return true;
    };

    if (!upload(lImg, leftRect_.data()) || !upload(rImg, rightRect_.data())) {
        doLog(log_, dashcam::log::LogLevel::ERROR,
              "vpiImageLockData (write) failed");
        return false;
    }

    if (vpiSubmitStereoDisparityEstimator(str, VPI_BACKEND_CUDA, pld,
                                          lImg, rImg, dImg,
                                          nullptr, nullptr) != VPI_SUCCESS) {
        doLog(log_, dashcam::log::LogLevel::ERROR,
              "vpiSubmitStereoDisparityEstimator failed");
        return false;
    }
    vpiStreamSync(str);

    // Download S16 disparity; convert to float (VPI SGM scale: value = disp × 32).
    VPIImageData dispData = {};
    if (vpiImageLockData(dImg, VPI_LOCK_READ,
                         VPI_IMAGE_BUFFER_HOST_PITCH_LINEAR, &dispData) != VPI_SUCCESS) {
        doLog(log_, dashcam::log::LogLevel::ERROR,
              "vpiImageLockData (read) failed");
        return false;
    }

    // Read row-by-row; pitchBytes is in bytes, each element is int16_t.
    const uint8_t* base       = static_cast<const uint8_t*>(dispData.buffer.pitch.planes[0].data);
    const int32_t  pitchBytes = dispData.buffer.pitch.planes[0].pitchBytes;
    result.disparityF32.resize(static_cast<std::size_t>(w * h));
    for (int y = 0; y < h; ++y) {
        const int16_t* row = reinterpret_cast<const int16_t*>(base + y * pitchBytes);
        for (int x = 0; x < w; ++x)
            result.disparityF32[static_cast<std::size_t>(y * w + x)] =
                (row[x] > 0) ? (row[x] / 32.0f) : -1.0f;
    }

    vpiImageUnlock(dImg);
    result.valid = true;
    return true;
}

// ─── OpenCV CPU backend ───────────────────────────────────────────────────────

bool StereoRangefinder::computeDepthCpu(DepthResult& result) {
    const int w = calib_.imageWidth;
    const int h = calib_.imageHeight;

    // Wrap rectified grey buffers as OpenCV Mats (no copy).
    const cv::Mat left(h,  w, CV_8UC1, leftRect_.data());
    const cv::Mat right(h, w, CV_8UC1, rightRect_.data());

    // SGBM with parameters suited to 640×360 images and a narrow webcam baseline.
    // blockSize=5, nDisparities=64 (must be multiple of 16).
    const int blockSize   = 5;
    const int numDisp     = 64;
    const int P1          = 8  * blockSize * blockSize;
    const int P2          = 32 * blockSize * blockSize;
    auto sgbm = cv::StereoSGBM::create(
        0, numDisp, blockSize,
        P1, P2,
        1,    // disp12MaxDiff
        0,    // preFilterCap
        5,    // uniquenessRatio
        100,  // speckleWindowSize
        2,    // speckleRange
        cv::StereoSGBM::MODE_SGBM_3WAY);

    cv::Mat disparity;
    sgbm->compute(left, right, disparity);  // output: S16, value = disp × 16

    result.disparityF32.resize(static_cast<std::size_t>(w * h));
    const int16_t* src = reinterpret_cast<const int16_t*>(disparity.data);
    for (int i = 0; i < w * h; ++i)
        result.disparityF32[static_cast<std::size_t>(i)] =
            (src[i] > 0) ? (src[i] / 16.0f) : -1.0f;

    result.valid = true;
    return true;
}

// ─── depth conversion ─────────────────────────────────────────────────────────

float StereoRangefinder::getDistanceAt(int x, int y, const DepthResult& result) const {
    if (!result.valid || x < 0 || y < 0 || x >= result.width || y >= result.height)
        return -1.0f;

    const float disp = result.disparityF32[static_cast<std::size_t>(y * result.width + x)];
    if (disp <= 0.0f) return -1.0f;

    return static_cast<float>(calib_.focalPx * calib_.baselineM / static_cast<double>(disp));
}

} // namespace dashcam::stereo
