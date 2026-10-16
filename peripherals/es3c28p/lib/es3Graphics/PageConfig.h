#pragma once
#include <stdint.h>
#include <cstring>

struct Es3DashboardPage {
    const char* tab;
    const char* title;
    const char* line1;
    const char* line2;
    const char* line3;
    const char* line4;
    bool showBrightnessControl;
    bool showSdSplash;
    bool allowTouchCalibration;
};

// Text storage and USB PAGE fields both reserve one byte for the terminator.
inline bool es3ValidPages(const Es3DashboardPage* pages, uint8_t count, uint8_t maximum) {
    if (!pages || !count || count > maximum) return false;
    for (uint8_t i = 0; i < count; ++i) {
        if (!pages[i].tab) return false;
        const char* fields[] = {pages[i].title, pages[i].line1, pages[i].line2,
                                pages[i].line3, pages[i].line4};
        for (const char* field : fields) {
            if (!field || std::strlen(field) >= 24) return false;
        }
    }
    return true;
}
