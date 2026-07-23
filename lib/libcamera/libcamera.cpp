#include "libcamera.h"
#include <algorithm>
#include <cctype>
#include <climits>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <system_error>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/videodev2.h>

namespace dashcam::camera {

struct ScopedFd {
    int fd;
    explicit ScopedFd(int fd) : fd(fd) {}
    ~ScopedFd() { if (fd >= 0) ::close(fd); }
    ScopedFd(const ScopedFd&)            = delete;  // non-copyable: prevents double-close
    ScopedFd& operator=(const ScopedFd&) = delete;
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
            if (frmival.discrete.numerator != 0) {
                frate.frameRate = static_cast<float>(frmival.discrete.denominator)
                                / frmival.discrete.numerator;
                formats.push_back(frate);
            }
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
 * @param[in]     log   Optional diagnostic callback; warns when the format
 *                      cross-product exceeds MAX_VIDEO_FORMATS and gets truncated.
 */
static void populateCameraVideoFormats(int fd, cameraInfo& info,
                                       const dashcam::log::LogCallback& log) {
    if (fd < 0 || info.type == CAMERA_TYPE::UNKNOWN) return;

    queryPixelFormats(fd, info);

    // Each stage expands the previous set (format → +resolution → +framerate),
    // so the running total is bounded to MAX_VIDEO_FORMATS to cap the cross
    // product.  A stage may overshoot by up to one query's worth of entries;
    // trim afterwards so currentFormatIndex (uint16_t) stays addressable.
    // Truncation is legal but must not be silent — a C270 already enumerates
    // ~227 modes, so a richer device would silently lose capture modes.
    bool truncated = false;
    std::vector<cameraVideoFormat> tempFormats;
    size_t tempFormatCount = info.videoFormats.size();
    size_t i = 0;
    for (; i < tempFormatCount && tempFormats.size() < MAX_VIDEO_FORMATS; ++i) {
        queryResolutions(fd, info.videoFormats[i], tempFormats);
    }
    if (i < tempFormatCount || tempFormats.size() > MAX_VIDEO_FORMATS) truncated = true;
    if (tempFormats.size() > MAX_VIDEO_FORMATS) tempFormats.resize(MAX_VIDEO_FORMATS);
    info.videoFormats = tempFormats;

    tempFormatCount = info.videoFormats.size();
    tempFormats.clear();
    for (i = 0; i < tempFormatCount && tempFormats.size() < MAX_VIDEO_FORMATS; ++i) {
        queryFrameRates(fd, info.videoFormats[i], tempFormats);
    }
    if (i < tempFormatCount || tempFormats.size() > MAX_VIDEO_FORMATS) truncated = true;
    if (tempFormats.size() > MAX_VIDEO_FORMATS) tempFormats.resize(MAX_VIDEO_FORMATS);
    info.videoFormats = tempFormats;

    if (truncated && log) {
        log(dashcam::log::LogLevel::WARN,
            info.address + ": video format list capped at "
            + std::to_string(MAX_VIDEO_FORMATS)
            + " entries; some capture modes were not enumerated");
    }
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
    if (fd < 0 || info.type == CAMERA_TYPE::UNKNOWN) {
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
            // Menu indices may be sparse, so scanning stops on either the option
            // cap or a bounded span — the latter guards against a driver that
            // misreports a huge maximum (which would otherwise spam ioctls and
            // risk signed overflow on ++i near INT_MAX).
            uint32_t scanned = 0;
            for (int i = queryctrl.minimum;
                 i <= queryctrl.maximum
                 && attr.menuOptions.size() < MAX_MENU_OPTIONS
                 && scanned < MAX_MENU_OPTIONS * 64u;
                 ++i, ++scanned) {
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

/**
 * @brief Resolve a CSI video node's Argus sensor-id from the device tree.
 *
 * /dev/videoN ordering follows i2c PROBE order, which is NOT the Argus
 * enumeration order: Argus numbers sensors by their index in the
 * tegra-camera-platform module list.  With mixed sensors on the two CSI ports
 * the two orders genuinely diverge (observed: IMX296 on i2c bus 9 probes
 * before IMX219 on bus 10, yet IMX219 is module0 → Argus sensor-id 0), so a
 * sequential count would silently open the wrong camera.
 *
 * Chain: V4L2 card string "vi-output, imx219 10-0010" → i2c device name →
 * /sys/bus/i2c/devices/<dev>/of_node symlink → sensor DT path → index of the
 * tegra-camera-platform module whose drivernode0 names that same path.
 *
 * @param[in] card  V4L2 capability card string of the vi-output node.
 * @return Argus sensor-id (module index), or -1 if the chain cannot be
 *         resolved (caller falls back to sequential numbering).
 */
static int argusIdFromCard(const std::string& card) {
    const std::string::size_type sp = card.find_last_of(' ');
    if (sp == std::string::npos) return -1;
    const std::string i2cDev = card.substr(sp + 1);          // e.g. "10-0010"
    if (i2cDev.empty()) return -1;

    char link[PATH_MAX];
    const std::string ofNode = "/sys/bus/i2c/devices/" + i2cDev + "/of_node";
    const ssize_t n = ::readlink(ofNode.c_str(), link, sizeof(link) - 1);
    if (n <= 0) return -1;
    link[n] = '\0';

    // Both the symlink target and the module entry contain
    // ".../devicetree/base/<sensor DT path>" — compare the tails.
    static constexpr const char kBase[] = "devicetree/base";
    const std::string sensorPath(link);
    const std::string::size_type sb = sensorPath.find(kBase);
    if (sb == std::string::npos) return -1;
    const std::string sensorTail = sensorPath.substr(sb + sizeof(kBase) - 1);

    for (int k = 0; k < MAX_CAMERAS; ++k) {
        std::ifstream f("/proc/device-tree/tegra-camera-platform/modules/module"
                        + std::to_string(k) + "/drivernode0/sysfs-device-tree");
        if (!f) break;   // modules are contiguous; first miss ends the scan
        std::string mod((std::istreambuf_iterator<char>(f)),
                        std::istreambuf_iterator<char>());
        while (!mod.empty() && (mod.back() == '\0' || mod.back() == '\n'))
            mod.pop_back();
        const std::string::size_type mb = mod.find(kBase);
        if (mb == std::string::npos) continue;
        if (mod.substr(mb + sizeof(kBase) - 1) == sensorTail) return k;
    }
    return -1;
}

ERROR_CODE getCameraList(std::vector<cameraInfo>& cameraList,
                         const dashcam::log::LogCallback& log) {
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
    // Sort by the numeric suffix, not lexicographically: path ordering would
    // put video10 before video2, which scrambles the CSI sensor-id assignment
    // below on systems with more than 9 video nodes (4 UVC cameras plus their
    // metadata nodes get there easily).  The suffix is all digits — verified
    // during the scan above.
    auto videoIndex = [](const std::filesystem::path& p) {
        return std::strtoul(p.filename().string().c_str() + 5, nullptr, 10);
    };
    std::sort(videoPaths.begin(), videoPaths.end(),
              [&videoIndex](const std::filesystem::path& a, const std::filesystem::path& b) {
                  return videoIndex(a) < videoIndex(b);
              });

    // Argus sensor-id is a dense 0-based index over ACTIVE camera modules.
    // Device-tree module indices are not dense when a module is disabled or
    // physically absent (for example, only module1 populated still appears to
    // Argus as sensor-id 0).  Remember each discovered module index and compact
    // them after discovery in module order.
    struct CsiModule {
        size_t cameraIndex;
        int    moduleIndex;  // -1 when the device-tree chain was unavailable
        size_t discoveryOrder;
    };
    std::vector<CsiModule> csiModules;

    for (const auto& devicePath : videoPaths) {
        ScopedFd fd(::open(devicePath.c_str(), O_RDONLY | O_NONBLOCK));
        if (fd < 0) continue;

        struct v4l2_capability cap;
        if (ioctl(fd, VIDIOC_QUERYCAP, &cap) == -1) continue;

        // device_caps is only valid when V4L2_CAP_DEVICE_CAPS is advertised.
        // Older drivers leave device_caps zero and report capabilities only
        // through cap.capabilities, so fall back to it in that case.
        const uint32_t caps = (cap.capabilities & V4L2_CAP_DEVICE_CAPS)
                            ? cap.device_caps : cap.capabilities;
        if (!(caps & V4L2_CAP_VIDEO_CAPTURE)) continue;

        cameraInfo info;
        info.address = devicePath.string();
        std::string driverName(reinterpret_cast<const char*>(cap.driver));

        uint32_t parsedDevId = static_cast<uint32_t>(std::strtoul(devicePath.filename().string().c_str() + 5, nullptr, 10));

        if (driverName == "tegra-video" || driverName == "vi") {
            info.type = CAMERA_TYPE::CSI;
            const std::string cardName(reinterpret_cast<const char*>(cap.card));
            const int moduleIndex = argusIdFromCard(cardName);
            // Temporary value; overwritten with the compact active ordinal below.
            info.deviceId = 0;
            if (moduleIndex < 0 && log)
                log(dashcam::log::LogLevel::WARN,
                    devicePath.string() + ": camera module not resolvable from "
                    "device tree; placing it after resolved CSI modules");
            csiModules.push_back(
                {cameraList.size(), moduleIndex, csiModules.size()});
        } else if (driverName == "uvcvideo") {
            info.type     = CAMERA_TYPE::USB;
            info.deviceId = parsedDevId;
        } else {
            info.type     = CAMERA_TYPE::UNKNOWN;
            info.deviceId = parsedDevId;
        }

        populateCameraAttributes(fd, info);
        populateCameraVideoFormats(fd, info, log);
        cameraList.push_back(info);
    }

    // Match Argus' dense enumeration of active modules.  Known module positions
    // sort first in device-tree order; unresolved modules retain discovery order.
    std::stable_sort(csiModules.begin(), csiModules.end(),
        [](const CsiModule& a, const CsiModule& b) {
            if ((a.moduleIndex >= 0) != (b.moduleIndex >= 0))
                return a.moduleIndex >= 0;
            if (a.moduleIndex >= 0 && a.moduleIndex != b.moduleIndex)
                return a.moduleIndex < b.moduleIndex;
            return a.discoveryOrder < b.discoveryOrder;
        });
    for (size_t argusId = 0; argusId < csiModules.size(); ++argusId)
        cameraList[csiModules[argusId].cameraIndex].deviceId =
            static_cast<uint32_t>(argusId);

    if (cameraList.empty()) {
        return ERROR_CODE::NO_CAMERAS_FOUND;
    }
    return ERROR_CODE::NONE;
}

} // namespace dashcam::camera
