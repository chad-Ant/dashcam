#include "libcamera.h"
#include <iostream>
#include <filesystem>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/videodev2.h>

int getCameraList(std::vector<cameraInfo>& cameraList) {
    for (int i = 0; i < 16; ++i) {
        std::string device_path = "/dev/video" + std::to_string(i);

        // Check if the file physically exists
        if (!std::filesystem::exists(device_path)) {
            continue; 
        }

        // Open the device file descriptor (read-only)
        int fd = open(device_path.c_str(), O_RDONLY);
        if (fd == -1) {
            continue; // Can't open, skip it
        }

        // Query the hardware capabilities
        struct v4l2_capability caps;
        if (ioctl(fd, VIDIOC_QUERYCAP, &caps) != -1) {
            
            // THE MAGIC FILTER: Check if it actually captures video
            // This safely ignores Jetson metadata nodes (/dev/video2, etc.)
            if (caps.device_caps & V4L2_CAP_VIDEO_CAPTURE) {
                
                cameraInfo info;
                info.address = device_path;
                info.name = reinterpret_cast<char*>(caps.card);
                
                // Identify the camera type by looking at the Linux driver string
                std::string driver_name = reinterpret_cast<char*>(caps.driver);
                
                if (driver_name == "tegra-video" || driver_name == "vi") {
                    info.type = CAMERA_TYPE::CSI;
                } 
                else if (driver_name == "uvcvideo") {
                    info.type = CAMERA_TYPE::USB;
                } 
                else {
                    info.type = CAMERA_TYPE::UNKNOWN;
                }

                cameraList.push_back(info);
            }
        }
        close(fd); // Always close the file descriptor!
    }

    return 0;
}