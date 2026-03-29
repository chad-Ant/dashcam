#ifndef LIBCAMERA_H
#define LIBCAMERA_H

#include <string>
#include <vector>
#include <cstdint>

enum class CAMERA_TYPE {
    CSI,
    USB,
    GIGE,
    UNKNOWN
};

struct cameraAttribute {
    std::string name;
    std::string type;
    bool isWritable;
    bool isReadable;
};

struct cameraVideoFormat {
    uint32_t width;
    uint32_t height;
    _Float32 frameRate;
    std::string pixelFormat;
};

struct cameraStatus {
    bool isOpen;
    bool isRunning;
    uint64_t frameCount;
    uint64_t freeBufferCount;
};

struct cameraInfo {
    std::string name;
    CAMERA_TYPE type;
    std::string address;
    std::vector<cameraAttribute> attributes;
    std::vector<cameraVideoFormat> videoFormats;
    cameraStatus status;
};

int getCameraList(std::vector<cameraInfo>& cameraList);

class iCamera {
public:
    virtual ~iCamera() = default;
    virtual int open() = 0;
    virtual int close() = 0;
    virtual bool isOpen() const = 0;
    virtual cameraInfo getCameraInfo() const = 0;
    virtual int setCameraAttribute(const std::string& name, const std::string& value) = 0;
    virtual int setCameraVideoFormat(const cameraVideoFormat& format) = 0;
    virtual cameraStatus getCameraStatus() const = 0;
    virtual int start() = 0;
    virtual int stop() = 0;
    virtual int captureFrame(std::vector<uint8_t>& buffer) = 0;
};

#endif //LIBCAMERA_H