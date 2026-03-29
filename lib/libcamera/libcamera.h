#ifndef LIBCAMERA_H
#define LIBCAMERA_H

#include <cstdint>
#include <string>
#include <vector>

// --- Static Memory Limits ---
constexpr uint32_t MAX_CAMERAS = 10;
constexpr uint32_t MAX_ATTRIBUTES = 64;
constexpr uint32_t MAX_VIDEO_FORMATS = 256; // PER QUERY, NOT TOTAL
constexpr uint32_t MAX_MENU_OPTIONS = 16;
constexpr uint32_t MAX_STRING_LEN = 32;

enum class ERROR_CODE {
    SUCCESS = 0,
    INVALID_FILE_DESCRIPTOR = -1,
    INVALID_ATTRIBUTE,
    UNSUPPORTED_FORMAT,
    CAMERA_BUSY,
    UNKNOWN_ERROR
};

enum class CAMERA_TYPE {
    CSI,
    USB,
    GIGE,
    UNKNOWN
};

enum class CAMERA_ATTRIBUTE_TYPE {
    INTEGER,
    I64,
    U32,
    U16,
    U8,
    FLOAT,
    STRING,
    BOOLEAN,
    MENU,
    BUTTON,
    UNKNOWN
};

struct cameraAttribute {
    std::string name;
    CAMERA_ATTRIBUTE_TYPE type;
    bool isWritable;
    bool isReadable;
    float minValue;
    float maxValue;
    float step;
    std::vector<std::string> menuOptions;
};

struct cameraVideoFormat {
    uint32_t width;
    uint32_t height;
    float frameRate;
    uint32_t pixelFormat; // V4L2 pixel format code (e.g., V4L2_PIX_FMT_YUYV)
    std::string description;
};

struct cameraStatus {
    bool isOpen;
    bool isRunning;
    uint64_t frameCount;
    uint64_t freeBufferCount;
};

struct cameraInfo {
    CAMERA_TYPE type;
    std::string address;
    std::vector<cameraAttribute> attributes;
    std::vector<cameraVideoFormat> videoFormats;
};

ERROR_CODE getCameraList(std::vector<cameraInfo>& cameraList);

class iCamera {
public:
    virtual ~iCamera() = default;
    virtual ERROR_CODE open() = 0;
    virtual ERROR_CODE close() = 0;
    virtual bool isOpen() const = 0;
    virtual ERROR_CODE getCameraInfo(cameraInfo& info) const = 0;
    virtual ERROR_CODE setCameraAttribute(const std::string& name, const std::string& value) = 0;
    virtual ERROR_CODE setCameraVideoFormat(const cameraVideoFormat& format) = 0;
    virtual ERROR_CODE getCameraStatus(cameraStatus& status) const = 0;
    virtual ERROR_CODE start() = 0;
    virtual ERROR_CODE stop() = 0;
    virtual ERROR_CODE captureFrame(uint8_t* buffer, uint32_t bufferSize, uint32_t& bytesWritten) = 0;
};

#endif //LIBCAMERA_H