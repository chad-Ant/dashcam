#include "librecord.h"

#include <dirent.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/statvfs.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <map>
#include <vector>

namespace dashcam::record {

using dashcam::log::LogLevel;

namespace {

struct Unit {                            ///< One segment: its .mkv and/or .ass.
    std::vector<std::string> names;
    uint64_t                 bytes = 0;
};

// Free bytes available to an unprivileged writer; -1 when statvfs fails.
int64_t freeBytes(int dirFd) {
    struct statvfs vfs;
    if (::fstatvfs(dirFd, &vfs) != 0) return -1;
    return static_cast<int64_t>(vfs.f_bavail) * static_cast<int64_t>(vfs.f_frsize);
}

void logf(const dashcam::log::LogCallback& log, LogLevel lvl, const std::string& msg) {
    if (log) log(lvl, msg);
}

} // namespace

RetentionResult enforceRetention(const RetentionPolicy& policy, uint64_t deletableBelowSeq,
                                 const std::function<bool()>& keepGoing,
                                 const dashcam::log::LogCallback& log) {
    RetentionResult res;

    DIR* d = ::opendir(policy.dir.c_str());
    if (!d) {
        logf(log, LogLevel::ERROR, "retention: cannot open " + policy.dir + ": " +
                                   std::strerror(errno));
        res.errors = 1;
        return res;
    }
    const int dirFd = ::dirfd(d);

    // One snapshot per pass.  Key (SEQ, stem) orders oldest first by SEQ with the
    // name as a deterministic tie-break; the wall-clock part is never trusted.
    std::map<std::pair<uint64_t, std::string>, Unit> units;
    while (const struct dirent* e = ::readdir(d)) {
        const std::string name = e->d_name;
        const auto seq = parseSegmentName(name, policy.prefix);
        if (!seq) continue;                                    // foreign: never touched
        struct stat st;
        if (::fstatat(dirFd, name.c_str(), &st, AT_SYMLINK_NOFOLLOW) != 0) continue;
        if (!S_ISREG(st.st_mode)) continue;                    // symlinks, dirs, fifos
        Unit& u = units[{*seq, name.substr(0, name.size() - 4)}];
        u.names.push_back(name);
        u.bytes += static_cast<uint64_t>(st.st_blocks) * 512;
    }
    for (const auto& [key, u] : units) res.ownedBytes += u.bytes;
    res.freeBytes = freeBytes(dirFd);

    auto over = [&] {
        const bool quota = policy.maxBytes > 0 && res.ownedBytes > policy.maxBytes;
        const bool floor = res.freeBytes >= 0 &&
                           static_cast<uint64_t>(res.freeBytes) < policy.minFreeBytes;
        return quota || floor;
    };

    for (auto& [key, u] : units) {
        if (!over()) break;
        if (keepGoing && !keepGoing()) break;
        if (key.first >= deletableBelowSeq) break;             // open / finalising / newer
        for (const std::string& name : u.names) {
            if (::unlinkat(dirFd, name.c_str(), 0) == 0 || errno == ENOENT) {
                ++res.deleted;
                continue;
            }
            ++res.errors;
            logf(log, LogLevel::WARN, "retention: cannot delete " + policy.dir + "/" + name +
                                      ": " + std::strerror(errno));
        }
        // Account the unit as gone only when every file in it is; a failed unlink
        // leaves its bytes counted and the pass moves on to the next unit.
        bool allGone = true;
        for (const std::string& name : u.names) {
            struct stat st;
            if (::fstatat(dirFd, name.c_str(), &st, AT_SYMLINK_NOFOLLOW) == 0) allGone = false;
        }
        if (allGone) {
            res.ownedBytes -= u.bytes;
            res.freedBytes += u.bytes;
            logf(log, LogLevel::INFO, "retention: deleted " + key.second + " (" +
                                      std::to_string(u.bytes / 1000000) + " MB)");
        }
        res.freeBytes = freeBytes(dirFd);
    }
    res.stillOver = over();
    ::closedir(d);
    return res;
}

} // namespace dashcam::record
