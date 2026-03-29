#include "libcamera.h"
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/videodev2.h>

void queryPixelFormats(int fd, cameraInfo& info) {
    struct v4l2_fmtdesc fmtdesc;
    cameraVideoFormat format;

    memset(&fmtdesc, 0, sizeof(fmtdesc));

    fmtdesc.index = 0;
    fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    while (ioctl(fd, VIDIOC_ENUM_FMT, &fmtdesc) == 0) {
        if (fmtdesc.index >= MAX_VIDEO_FORMATS) {
            break; // Exceeded max formats, stop enumerating
        }
        format.description = std::string(reinterpret_cast<const char*>(fmtdesc.description));
        format.pixelFormat = fmtdesc.pixelformat;
        info.videoFormats.push_back(format);
        fmtdesc.index++;
    }
}

void queryResolutions(int fd,const cameraVideoFormat& info, std::vector<cameraVideoFormat>& formats) {
    struct v4l2_frmsizeenum frmsize;
    cameraVideoFormat resolution = info;

    memset(&frmsize, 0, sizeof(frmsize));

    frmsize.index = 0;
    frmsize.pixel_format = info.pixelFormat;

    while (ioctl(fd, VIDIOC_ENUM_FRAMESIZES, &frmsize) == 0) {
        if (frmsize.index >= MAX_VIDEO_FORMATS) {
            break; // Exceeded max formats, stop enumerating
        }
        if (frmsize.type == V4L2_FRMSIZE_TYPE_DISCRETE) {
            resolution.width = frmsize.discrete.width;
            resolution.height = frmsize.discrete.height;
            formats.push_back(resolution);
        }
        frmsize.index++;
    }
}

void queryFrameRates(int fd,const cameraVideoFormat& info, std::vector<cameraVideoFormat>& formats) {
    struct v4l2_frmivalenum frmival;
    cameraVideoFormat frate = info;

    memset(&frmival, 0, sizeof(frmival));

    frmival.index = 0;
    frmival.pixel_format = info.pixelFormat;
    frmival.width = info.width;
    frmival.height = info.height;

    while (ioctl(fd, VIDIOC_ENUM_FRAMEINTERVALS, &frmival) == 0) {
        if (frmival.index >= MAX_VIDEO_FORMATS) {
            break; // Exceeded max formats, stop enumerating
        }
        if (frmival.type == V4L2_FRMIVAL_TYPE_DISCRETE) {
            frate.frameRate = static_cast<float>(frmival.discrete.denominator) / frmival.discrete.numerator;

            formats.push_back(frate);
        }
        frmival.index++;
    }
}

void populateCameraVideoFormats(int fd, cameraInfo& info) {
    if (fd < 0 || info.type == CAMERA_TYPE::UNKNOWN || info.type == CAMERA_TYPE::GIGE) { // GIGE cameras don't support V4L2 controls, skip attribute enumeration for now
        return;
    }

    queryPixelFormats(fd, info);
    int tempFormatCount = info.videoFormats.size();
    std::vector<cameraVideoFormat> tempFormats;
    for (uint16_t i = 0; i < tempFormatCount; ++i) {
        queryResolutions(fd, info.videoFormats[i], tempFormats);
    }
    info.videoFormats = tempFormats; // Replace with resolution-enriched formats
    tempFormatCount = info.videoFormats.size();
    tempFormats.clear();
    for (uint16_t i = 0; i < tempFormatCount; ++i) {
        queryFrameRates(fd, info.videoFormats[i], tempFormats);
    }
    info.videoFormats = tempFormats; // Replace with frame rate-enriched formats
}

void populateCameraAttributes(int fd, cameraInfo& info) {
    if (fd < 0 || info.type == CAMERA_TYPE::UNKNOWN || info.type == CAMERA_TYPE::GIGE) { // GIGE cameras don't support V4L2 controls, skip attribute enumeration for now
        return;
    }

    cameraAttribute attr;
    struct v4l2_queryctrl queryctrl;
    memset(&queryctrl, 0, sizeof(queryctrl));
    queryctrl.id = V4L2_CTRL_FLAG_NEXT_CTRL;

    while (ioctl(fd, VIDIOC_QUERYCTRL, &queryctrl) == 0) {
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
                struct v4l2_querymenu querymenu;
                memset(&querymenu, 0, sizeof(querymenu));
                querymenu.id = queryctrl.id;
                querymenu.index = i;

                if (ioctl(fd, VIDIOC_QUERYMENU, &querymenu) == -1) {
                    continue; // Not a valid menu option, skip it
                }
                attr.menuOptions.push_back(std::string(reinterpret_cast<const char*>(querymenu.name)));
            }
        }
        info.attributes.push_back(attr);
        queryctrl.id |= V4L2_CTRL_FLAG_NEXT_CTRL;
    }
}

ERROR_CODE getCameraList(std::vector<cameraInfo>& cameraList) {
    cameraList.clear();
for (int i = 0; i < 10; ++i) {
        std::string device_path = "/dev/video" + std::to_string(i);

        if (!std::filesystem::exists(device_path)) {
            continue; 
        }

        cameraInfo info;
        info.address = device_path;

        int fd = open(info.address.c_str(), O_RDONLY | O_NONBLOCK);
        if (fd < 0) {
            continue; 
        }
        struct v4l2_capability cap;
        if (ioctl(fd, VIDIOC_QUERYCAP, &cap) == -1) {
            close(fd);
            continue; 
        }

        if (cap.device_caps & V4L2_CAP_VIDEO_CAPTURE) {
            std::string driver_name(reinterpret_cast<const char*>(cap.driver));
            if (driver_name == "tegra-video" || driver_name == "vi") {
                info.type = CAMERA_TYPE::CSI;
            } else if (driver_name == "uvcvideo") {
                info.type = CAMERA_TYPE::USB;
            } else {
                info.type = CAMERA_TYPE::UNKNOWN;
            }
            populateCameraAttributes(fd, info);
            populateCameraVideoFormats(fd, info);
            cameraList.push_back(info);
        }
        // Always close the file descriptor when done!
        close(fd);
        }
    return ERROR_CODE::SUCCESS;
}