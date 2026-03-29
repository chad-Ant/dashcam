#ifndef LIBCAMERA_CSI_H
#define LIBCAMERA_CSI_H

#include "libcamera.h"
#include <string>
#include <vector>

typedef struct _GstElement GstElement;

class Camera_CSI : public iCamera {
private:
    cameraInfo info_;
    cameraStatus status_;

    GstElement* pipeline_;
    GstElement* camera_src_;
    GstElement* appsink_;

public:
    Camera_CSI(const cameraInfo& camera);
    ~Camera_CSI() override;

    ERROR_CODE open() override;
    ERROR_CODE close() override;
    bool isOpen() const override;
    ERROR_CODE getCameraInfo(cameraInfo& info) const override;
    ERROR_CODE setCameraAttribute(const std::string& name, const std::string& value) override;
    ERROR_CODE setCameraVideoFormat(const cameraVideoFormat& format) override;
    ERROR_CODE getCameraStatus(cameraStatus& status) const override;
    ERROR_CODE start() override;
    ERROR_CODE stop() override;
    ERROR_CODE captureFrame(uint8_t* buffer, uint32_t bufferSize, uint32_t& bytesWritten) override;
};

#endif // LIBCAMERA_CSI_H
