#include "libcamera.h"
#include <iostream>
#include <iomanip>
#include <vector>

void printCameraData(const cameraInfo& info) {
    std::cout << "\n======================================================\n";
    std::cout << "📷 DEVICE: " << info.address << "\n";
    std::cout << "   TYPE:   ";
    switch (info.type) {
        case CAMERA_TYPE::CSI: std::cout << "Jetson CSI (tegra-video / vi)"; break;
        case CAMERA_TYPE::USB: std::cout << "USB Webcam (uvcvideo)"; break;
        case CAMERA_TYPE::GIGE: std::cout << "GigE Network Camera"; break;
        default: std::cout << "Unknown"; break;
    }
    std::cout << "\n======================================================\n";
    
    std::cout << "\n--- CAMERA ATTRIBUTES (" << info.attributes.size() << " found) ---\n";
    for (const auto& attr : info.attributes) {
        std::cout << std::left << std::setw(28) << attr.name 
                  << " | Writable: " << (attr.isWritable ? "Yes" : "No ")
                  << " | Range: [" << attr.minValue << " to " << attr.maxValue << ", step " << attr.step << "]";
        
        if (attr.type == CAMERA_ATTRIBUTE_TYPE::MENU) {
            std::cout << "\n   --> Menu Options: [ ";
            for (const auto& opt : attr.menuOptions) {
                std::cout << "'" << opt << "' ";
            }
            std::cout << "]";
        }
        std::cout << "\n";
    }

    std::cout << "\n--- VIDEO FORMATS (" << info.videoFormats.size() << " found) ---\n";
    for (const auto& fmt : info.videoFormats) {
        std::cout << "Format: " << std::left << std::setw(25) << fmt.description 
                  << " | Res: " << std::setw(4) << fmt.width << "x" << std::left << std::setw(4) << fmt.height 
                  << " | FPS: " << fmt.frameRate << "\n";
    }
    std::cout << "\n";
}

int main() {
    std::cout << "🔍 Scanning system for connected cameras...\n";

    std::vector<cameraInfo> myCameras;
    
    // ONE LINE OF CODE does all the heavy lifting!
    ERROR_CODE status = getCameraList(myCameras);

    if (status != ERROR_CODE::SUCCESS) {
        std::cerr << "❌ ERROR: The camera discovery process failed.\n";
        return -1;
    }

    if (myCameras.empty()) {
        std::cout << "⚠️ No valid V4L2 cameras were found on this system.\n";
        return 0;
    }

    std::cout << "✅ Discovery Complete! Found " << myCameras.size() << " valid camera(s).\n";

    // Loop through the vector and print everything
    for (const auto& cam : myCameras) {
        printCameraData(cam);
    }

    return 0;
}