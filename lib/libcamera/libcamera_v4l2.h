/**
 * @file libcamera_v4l2.h
 * @brief The V4L2 plumbing libcamera shares: an EINTR-safe ioctl, an owning
 *        file descriptor, and control query / get / set.
 *
 * Discovery (libcamera.cpp), the exposure loop and the focus hold each used to
 * carry their own copy of these. Header-only, internal to libcamera: the public
 * headers do not include it.
 */

#ifndef LIBCAMERA_V4L2_H
#define LIBCAMERA_V4L2_H

#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <string>

namespace dashcam::camera::v4l2 {

/// ioctl(), retried while a signal interrupts it (EINTR), the usual V4L2
/// idiom: the apps install shutdown signal handlers, and an interrupted
/// control write would otherwise read as a refused one.
inline int xioctl(int fd, unsigned long request, void* arg) {
    int r;
    do {
        r = ::ioctl(fd, request, arg);
    } while (r == -1 && errno == EINTR);
    return r;
}

/// An owning file descriptor: closed on destruction, moved but never copied.
class Fd {
public:
    Fd() = default;
    explicit Fd(int fd) : fd_(fd) {}
    ~Fd() { reset(); }

    Fd(Fd&& other) noexcept : fd_(other.release()) {}
    Fd& operator=(Fd&& other) noexcept {
        if (this != &other) reset(other.release());
        return *this;
    }
    Fd(const Fd&)            = delete;
    Fd& operator=(const Fd&) = delete;

    int  get() const { return fd_; }
    bool valid() const { return fd_ >= 0; }

    /// Gives up ownership: the caller now closes the descriptor.
    int release() {
        const int fd = fd_;
        fd_ = -1;
        return fd;
    }

    void reset(int fd = -1) {
        if (fd_ >= 0) ::close(fd_);
        fd_ = fd;
    }

private:
    int fd_ = -1;
};

/// Opens a device node with @p accessMode (O_RDONLY / O_RDWR), non-blocking and
/// close-on-exec: the apps fork helpers (nmcli), which must not inherit it.
inline Fd openNode(const std::string& path, int accessMode) {
    return Fd(::open(path.c_str(), accessMode | O_NONBLOCK | O_CLOEXEC));
}

/// VIDIOC_QUERYCTRL for @p id: true when the control exists and is not disabled.
inline bool queryControl(int fd, uint32_t id, struct v4l2_queryctrl& q) {
    std::memset(&q, 0, sizeof(q));
    q.id = id;
    return xioctl(fd, VIDIOC_QUERYCTRL, &q) == 0 && !(q.flags & V4L2_CTRL_FLAG_DISABLED);
}

/// VIDIOC_G_CTRL. @p value is untouched on failure (errno says why).
inline bool getControl(int fd, uint32_t id, int& value) {
    struct v4l2_control c;
    std::memset(&c, 0, sizeof(c));
    c.id = id;
    if (xioctl(fd, VIDIOC_G_CTRL, &c) != 0) return false;
    value = c.value;
    return true;
}

/// VIDIOC_S_CTRL. On failure errno says why.
inline bool setControl(int fd, uint32_t id, int value) {
    struct v4l2_control c;
    std::memset(&c, 0, sizeof(c));
    c.id    = id;
    c.value = value;
    return xioctl(fd, VIDIOC_S_CTRL, &c) == 0;
}

} // namespace dashcam::camera::v4l2

#endif // LIBCAMERA_V4L2_H
