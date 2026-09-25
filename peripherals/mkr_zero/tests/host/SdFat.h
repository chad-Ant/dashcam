#ifndef HOST_SDFAT_STUB_H
#define HOST_SDFAT_STUB_H 1

/**
 * @file SdFat.h
 * @brief Host stand-in for the vendored SdFat — just enough to compile.
 *
 * lib/CANMap.cpp shares a file with the map's SD loader, so linking the map
 * builder the CAN tests need drags the loader's declarations in with it. There
 * is no card on the host: the sd* functions it calls are defined in
 * can_probe_tests.cpp to report one absent, and nothing here opens anything.
 */

#include <stddef.h>
#include <stdint.h>

/// lib/SDFunctions.h refuses to compile against a stock SdFat; the stand-in is
/// neither, and saying it is vendored is what lets the real header through.
#define DASHCAM_SDFAT_VENDORED 1

#define O_RDONLY 0

class File32 {
public:
    bool     openNext(File32 *, int) { return false; }
    bool     isDir() const           { return false; }
    bool     getName(char *, size_t) { return false; }
    uint32_t fileSize() const        { return 0u; }
    void     close()                 {}
};

#endif // HOST_SDFAT_STUB_H
