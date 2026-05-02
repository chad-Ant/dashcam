#include "libcamera.h"
#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <system_error>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/videodev2.h>

struct ScopedFd {
    int fd;
    explicit ScopedFd(int fd) : fd(fd) {}
    ~ScopedFd() { if (fd >= 0) ::close(fd); }
    operator int() const { return fd; }
};

/**
 * @brief Enumerate discrete pixel formats supported by a V4L2 capture device.
 *
 * Issues VIDIOC_ENUM_FMT in a loop and appends one skeleton cameraVideoFormat
 * (description + pixelFormat only; width/height/frameRate are zeroed) per
 * reported format to @p info.videoFormats.  Stops at MAX_VIDEO_FORMATS entries
 * or when the ioctl returns an error.
 *
 * @param[in]     fd    Open, readable V4L2 file descriptor.
 * @param[in,out] info  cameraInfo whose videoFormats vector is appended to.
 */
static void queryPixelFormats(int fd, cameraInfo& info) {
    struct v4l2_fmtdesc fmtdesc;
    cameraVideoFormat format{0, 0, 0.0f, 0, ""};

    memset(&fmtdesc, 0, sizeof(fmtdesc));
    fmtdesc.index = 0;
    fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    while (ioctl(fd, VIDIOC_ENUM_FMT, &fmtdesc) == 0) {
        if (fmtdesc.index >= MAX_VIDEO_FORMATS) {
            break;
        }
        format.description = std::string(reinterpret_cast<const char*>(fmtdesc.description));
        format.pixelFormat = fmtdesc.pixelformat;
        info.videoFormats.push_back(format);
        fmtdesc.index++;
    }
}

/**
 * @brief Enumerate discrete frame sizes for a given pixel format.
 *
 * Issues VIDIOC_ENUM_FRAMESIZES and appends one cameraVideoFormat per discrete
 * (width, height) pair to @p formats.  Stepwise and continuous size ranges are
 * silently skipped; this library targets hardware with fixed discrete modes.
 *
 * @param[in]  fd       Open, readable V4L2 file descriptor.
 * @param[in]  info     Template format entry carrying the pixel format to query.
 * @param[out] formats  Destination vector; entries are appended (not replaced).
 */
static void queryResolutions(int fd, const cameraVideoFormat& info, std::vector<cameraVideoFormat>& formats) {
    struct v4l2_frmsizeenum frmsize;
    cameraVideoFormat resolution = info;

    memset(&frmsize, 0, sizeof(frmsize));
    frmsize.index = 0;
    frmsize.pixel_format = info.pixelFormat;

    while (ioctl(fd, VIDIOC_ENUM_FRAMESIZES, &frmsize) == 0) {
        if (frmsize.index >= MAX_VIDEO_FORMATS) {
            break;
        }
        if (frmsize.type == V4L2_FRMSIZE_TYPE_DISCRETE) {
            resolution.width = frmsize.discrete.width;
            resolution.height = frmsize.discrete.height;
            formats.push_back(resolution);
        }
        frmsize.index++;
    }
}

/**
 * @brief Enumerate discrete frame intervals for a given format and resolution.
 *
 * Issues VIDIOC_ENUM_FRAMEINTERVALS and appends one fully-populated
 * cameraVideoFormat per discrete interval to @p formats.  The frame rate is
 * stored as @c denominator/numerator (i.e. fps, not the raw interval).
 * Stepwise and continuous intervals are silently skipped.
 *
 * @param[in]  fd       Open, readable V4L2 file descriptor.
 * @param[in]  info     Template format entry carrying pixel format, width, and height.
 * @param[out] formats  Destination vector; entries are appended (not replaced).
 */
static void queryFrameRates(int fd, const cameraVideoFormat& info, std::vector<cameraVideoFormat>& formats) {
    struct v4l2_frmivalenum frmival;
    cameraVideoFormat frate = info;

    memset(&frmival, 0, sizeof(frmival));
    frmival.index = 0;
    frmival.pixel_format = info.pixelFormat;
    frmival.width  = info.width;
    frmival.height = info.height;

    while (ioctl(fd, VIDIOC_ENUM_FRAMEINTERVALS, &frmival) == 0) {
        if (frmival.index >= MAX_VIDEO_FORMATS) {
            break;
        }
        if (frmival.type == V4L2_FRMIVAL_TYPE_DISCRETE) {
            frate.frameRate = static_cast<float>(frmival.discrete.denominator) / frmival.discrete.numerator;
            formats.push_back(frate);
        }
        frmival.index++;
    }
}

/**
 * @brief Fully populate cameraInfo::videoFormats with all discrete capture modes.
 *
 * Runs the three-stage pipeline:
 *  1. queryPixelFormats  — discovers format codes.
 *  2. queryResolutions   — expands each format code into (format, w, h) triples.
 *  3. queryFrameRates    — expands each (format, w, h) triple into a full entry.
 *
 * On return, @p info.videoFormats contains one entry per fully-qualified discrete
 * mode.  UNKNOWN and GIGE camera types are skipped entirely.
 *
 * @param[in]     fd    Open, readable V4L2 file descriptor.
 * @param[in,out] info  cameraInfo to populate; videoFormats is replaced on each stage.
 */
static void populateCameraVideoFormats(int fd, cameraInfo& info) {
    if (fd < 0 || info.type == CAMERA_TYPE::UNKNOWN || info.type == CAMERA_TYPE::GIGE) {
        return;
    }

    queryPixelFormats(fd, info);

    std::vector<cameraVideoFormat> tempFormats;
    size_t tempFormatCount = info.videoFormats.size();
    for (size_t i = 0; i < tempFormatCount; ++i) {
        queryResolutions(fd, info.videoFormats[i], tempFormats);
    }
    info.videoFormats = tempFormats;

    tempFormatCount = info.videoFormats.size();
    tempFormats.clear();
    for (size_t i = 0; i < tempFormatCount; ++i) {
        queryFrameRates(fd, info.videoFormats[i], tempFormats);
    }
    info.videoFormats = tempFormats;
}

/**
 * @brief Populate cameraInfo::attributes with all non-disabled V4L2 controls.
 *
 * Uses the @c V4L2_CTRL_FLAG_NEXT_CTRL iteration pattern to walk the full
 * control list without relying on contiguous IDs.  Disabled controls are
 * skipped.  Stops at MAX_ATTRIBUTES entries.
 *
 * For MENU controls, VIDIOC_QUERYMENU is called for each index in [minimum,
 * maximum].  INTEGER_MENU entries store the 64-bit integer value as a decimal
 * string; string-menu entries store the driver-provided name.
 *
 * UNKNOWN and GIGE camera types are skipped entirely.
 *
 * @param[in]     fd    Open, readable V4L2 file descriptor.
 * @param[in,out] info  cameraInfo whose attributes vector is populated.
 */
static void populateCameraAttributes(int fd, cameraInfo& info) {
    if (fd < 0 || info.type == CAMERA_TYPE::UNKNOWN || info.type == CAMERA_TYPE::GIGE) {
        return;
    }

    cameraAttribute attr;
    struct v4l2_queryctrl queryctrl;
    memset(&queryctrl, 0, sizeof(queryctrl));
    queryctrl.id = V4L2_CTRL_FLAG_NEXT_CTRL;

    while (ioctl(fd, VIDIOC_QUERYCTRL, &queryctrl) == 0) {
        if (info.attributes.size() >= MAX_ATTRIBUTES) {
            break;
        }
        if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED) {
            queryctrl.id |= V4L2_CTRL_FLAG_NEXT_CTRL;
            continue;
        }
        attr.name = std::string(reinterpret_cast<const char*>(queryctrl.name));
        switch (queryctrl.type) {
            case V4L2_CTRL_TYPE_INTEGER:
            case V4L2_CTRL_TYPE_BITMASK:
                attr.type = CAMERA_ATTRIBUTE_TYPE::INTEGER;
                break;
            case V4L2_CTRL_TYPE_INTEGER64:
                attr.type = CAMERA_ATTRIBUTE_TYPE::I64;
                break;
            case V4L2_CTRL_TYPE_U32:
                attr.type = CAMERA_ATTRIBUTE_TYPE::U32;
                break;
            case V4L2_CTRL_TYPE_U16:
                attr.type = CAMERA_ATTRIBUTE_TYPE::U16;
                break;
            case V4L2_CTRL_TYPE_U8:
                attr.type = CAMERA_ATTRIBUTE_TYPE::U8;
                break;
            case V4L2_CTRL_TYPE_BOOLEAN:
                attr.type = CAMERA_ATTRIBUTE_TYPE::BOOLEAN;
                break;
            case V4L2_CTRL_TYPE_MENU:
            case V4L2_CTRL_TYPE_INTEGER_MENU:
                attr.type = CAMERA_ATTRIBUTE_TYPE::MENU;
                break;
            case V4L2_CTRL_TYPE_STRING:
                attr.type = CAMERA_ATTRIBUTE_TYPE::STRING;
                break;
            case V4L2_CTRL_TYPE_BUTTON:
                attr.type = CAMERA_ATTRIBUTE_TYPE::BUTTON;
                break;
            default:
                attr.type = CAMERA_ATTRIBUTE_TYPE::UNKNOWN;
                break;
        }
        attr.isWritable = !(queryctrl.flags & V4L2_CTRL_FLAG_READ_ONLY);
        attr.isReadable = !(queryctrl.flags & V4L2_CTRL_FLAG_WRITE_ONLY);
        attr.minValue = static_cast<float>(queryctrl.minimum);
        attr.maxValue = static_cast<float>(queryctrl.maximum);
        attr.step = static_cast<float>(queryctrl.step);
        attr.menuOptions.clear();

        if (attr.type == CAMERA_ATTRIBUTE_TYPE::MENU) {
            for (int i = queryctrl.minimum; i <= queryctrl.maximum; ++i) {
                if (attr.menuOptions.size() >= MAX_MENU_OPTIONS) break;
                struct v4l2_querymenu querymenu;
                memset(&querymenu, 0, sizeof(querymenu));
                querymenu.id    = queryctrl.id;
                querymenu.index = i;
                if (ioctl(fd, VIDIOC_QUERYMENU, &querymenu) == -1) {
                    continue;
                }
                // INTEGER_MENU entries carry a 64-bit integer value, not a name string.
                if (queryctrl.type == V4L2_CTRL_TYPE_INTEGER_MENU) {
                    attr.menuOptions.push_back(std::to_string(querymenu.value));
                } else {
                    attr.menuOptions.push_back(std::string(reinterpret_cast<const char*>(querymenu.name)));
                }
            }
        }
        info.attributes.push_back(attr);
        queryctrl.id |= V4L2_CTRL_FLAG_NEXT_CTRL;
    }
}

ERROR_CODE getCameraList(std::vector<cameraInfo>& cameraList) {
    cameraList.clear();

    // Enumerate /dev/videoN nodes via the filesystem so we're not limited to video0..15.
    std::vector<std::filesystem::path> videoPaths;
    std::error_code ec;
    for (auto it = std::filesystem::directory_iterator("/dev", ec);
         !ec && it != std::filesystem::directory_iterator();
         it.increment(ec)) {
        const std::string name = it->path().filename().string();
        if (name.size() <= 5 || name.substr(0, 5) != "video") {
            continue;
        }
        bool allDigits = true;
        for (size_t j = 5; j < name.size(); ++j) {
            if (!std::isdigit(static_cast<unsigned char>(name[j]))) {
                allDigits = false;
                break;
            }
        }
        if (allDigits) {
            videoPaths.push_back(it->path());
        }
    }
    std::sort(videoPaths.begin(), videoPaths.end());

    // CSI cameras are addressed by Argus sensor-id (0-based among CSI cameras),
    // which is independent of the /dev/videoN numbering.
    uint32_t csiSensorCount = 0;
    int fd;
    
    for (const auto& devicePath : videoPaths) {
        ScopedFd fd(::open(devicePath.c_str(), O_RDONLY | O_NONBLOCK));
        if (fd < 0) continue;

        struct v4l2_capability cap;
        if (ioctl(fd, VIDIOC_QUERYCAP, &cap) == -1) continue;

        if (!(cap.device_caps & V4L2_CAP_VIDEO_CAPTURE)) continue;

        cameraInfo info;
        info.address = devicePath.string();
        std::string driverName(reinterpret_cast<const char*>(cap.driver));

        uint32_t parsedDevId = static_cast<uint32_t>(std::strtoul(devicePath.filename().string().c_str() + 5, nullptr, 10));

        if (driverName == "tegra-video" || driverName == "vi") {
            info.type     = CAMERA_TYPE::CSI;
            info.deviceId = csiSensorCount++;
        } else if (driverName == "uvcvideo") {
            info.type     = CAMERA_TYPE::USB;
            info.deviceId = parsedDevId;
        } else {
            info.type     = CAMERA_TYPE::UNKNOWN;
            info.deviceId = parsedDevId;
        }

        populateCameraAttributes(fd, info);
        populateCameraVideoFormats(fd, info);
        cameraList.push_back(info);
    }

    if (cameraList.empty()) {
        return ERROR_CODE::NO_CAMERAS_FOUND;
    }
    return ERROR_CODE::NONE;
}
