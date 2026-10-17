// nav_tiles.cpp — tile PNG decoding, HTTP caching rules and the SQLite tile
// cache of libnavigation (see libnavigation.h).

#include "libnavigation.h"

#include <png.h>
#include <sqlite3.h>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cerrno>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <set>
#include <tuple>

namespace dashcam::navigation {

// ─── PNG ──────────────────────────────────────────────────────────────────────

bool decodeTilePng(const uint8_t* data, size_t size, std::vector<uint8_t>& rgb, std::string* why, size_t maxBytes) {
    auto fail = [&](const std::string& w) {
        if (why) *why = w;
        return false;
    };
    static const uint8_t kSig[8] = {0x89, 'P', 'N', 'G', '\r', '\n', 0x1a, '\n'};
    if (!data || size < 33) return fail("too short for a PNG");
    if (size > maxBytes) return fail("larger than " + std::to_string(maxBytes) + " bytes");
    if (std::memcmp(data, kSig, 8) != 0) return fail("not a PNG (an error page?)");
    // IHDR is always first: check the size before libpng allocates anything.
    if (std::memcmp(data + 12, "IHDR", 4) != 0) return fail("PNG without a leading IHDR");
    const uint32_t w = (uint32_t(data[16]) << 24) | (uint32_t(data[17]) << 16) | (uint32_t(data[18]) << 8) | data[19];
    const uint32_t h = (uint32_t(data[20]) << 24) | (uint32_t(data[21]) << 16) | (uint32_t(data[22]) << 8) | data[23];
    if (w != uint32_t(kTileSize) || h != uint32_t(kTileSize))
        return fail("tile is " + std::to_string(w) + "x" + std::to_string(h) + ", not 256x256");

    png_image img;
    std::memset(&img, 0, sizeof(img));
    img.version = PNG_IMAGE_VERSION;
    if (!png_image_begin_read_from_memory(&img, data, size))
        return fail(std::string("PNG header: ") + img.message);
    if (img.width != uint32_t(kTileSize) || img.height != uint32_t(kTileSize)) {
        png_image_free(&img);
        return fail("PNG size changed between checks");
    }
    img.format = PNG_FORMAT_RGB;   // palette/grey/alpha are converted (alpha composited on black)
    rgb.resize(kTileRgbBytes);
    if (!png_image_finish_read(&img, nullptr, rgb.data(), 0, nullptr)) {
        const std::string m = img.message;
        png_image_free(&img);
        return fail("PNG data: " + m);
    }
    return true;
}

// ─── HTTP caching rules ───────────────────────────────────────────────────────

namespace {
std::string lower(std::string s) {
    for (char& c : s) c = char(std::tolower(static_cast<unsigned char>(c)));
    return s;
}
std::string trim(const std::string& s) {
    size_t a = 0, b = s.size();
    while (a < b && std::isspace(static_cast<unsigned char>(s[a]))) ++a;
    while (b > a && std::isspace(static_cast<unsigned char>(s[b - 1]))) --b;
    return s.substr(a, b - a);
}
int64_t parseSeconds(const std::string& v) {
    if (v.empty() || !std::all_of(v.begin(), v.end(), [](char c) { return std::isdigit(static_cast<unsigned char>(c)); }))
        return -1;
    if (v.size() > 12) return int64_t(1) << 40;   // absurdly large: effectively forever, no overflow
    return std::stoll(v);
}
} // namespace

CacheControl parseCacheControl(const std::string& value) {
    CacheControl cc;
    size_t pos = 0;
    while (pos <= value.size()) {
        const size_t comma = value.find(',', pos);
        const std::string item = trim(value.substr(pos, comma == std::string::npos ? std::string::npos : comma - pos));
        const size_t eq = item.find('=');
        const std::string name = lower(trim(item.substr(0, eq)));
        std::string arg = eq == std::string::npos ? "" : trim(item.substr(eq + 1));
        if (arg.size() >= 2 && arg.front() == '"' && arg.back() == '"') arg = arg.substr(1, arg.size() - 2);
        if (name == "max-age") cc.maxAge = parseSeconds(arg);
        else if (name == "s-maxage") {}   // shared caches only
        else if (name == "stale-while-revalidate") cc.staleWhileRevalidate = std::max<int64_t>(0, parseSeconds(arg));
        else if (name == "stale-if-error") cc.staleIfError = std::max<int64_t>(0, parseSeconds(arg));
        else if (name == "no-store") cc.noStore = true;
        else if (name == "no-cache") cc.noCache = true;
        if (comma == std::string::npos) break;
        pos = comma + 1;
    }
    return cc;
}

int64_t parseHttpDate(const std::string& value) {
    const std::string v = trim(value);
    if (v.empty()) return -1;
    static const char* kFormats[] = {
        "%a, %d %b %Y %H:%M:%S GMT",   // IMF-fixdate
        "%A, %d-%b-%y %H:%M:%S GMT",   // obsolete RFC 850
        "%a %b %d %H:%M:%S %Y",        // obsolete asctime
    };
    for (const char* f : kFormats) {
        struct tm t;
        std::memset(&t, 0, sizeof(t));
        const char* end = ::strptime(v.c_str(), f, &t);
        if (end && *end == '\0') return int64_t(::timegm(&t));
    }
    return -1;
}

int64_t parseRetryAfter(const std::string& value, int64_t nowS) {
    const std::string v = trim(value);
    const int64_t s = parseSeconds(v);
    if (s >= 0) return s;
    const int64_t d = parseHttpDate(v);
    if (d < 0) return -1;
    return std::max<int64_t>(0, d - nowS);
}

TileFreshness freshnessFromHeaders(const CacheControl& cc, int64_t dateS, int64_t expiresS, int64_t lastModifiedS,
                                   int64_t ageS, int64_t nowS) {
    TileFreshness f;
    f.fetchedAt            = nowS;
    f.staleWhileRevalidate = cc.staleWhileRevalidate;
    f.staleIfError         = cc.staleIfError;
    const int64_t age = std::max<int64_t>(0, ageS);
    int64_t lifetime  = 0;
    if (cc.noCache) {
        lifetime = 0;   // usable only after revalidation
    } else if (cc.maxAge >= 0) {
        lifetime = cc.maxAge;
    } else if (expiresS >= 0) {
        lifetime = expiresS - (dateS >= 0 ? dateS : nowS);
    } else if (lastModifiedS >= 0) {
        const int64_t since = (dateS >= 0 ? dateS : nowS) - lastModifiedS;
        lifetime = std::min<int64_t>(86400, std::max<int64_t>(0, since / 10));
    }
    f.freshUntil = nowS + std::max<int64_t>(0, lifetime - age);
    return f;
}

bool clockPlausible(int64_t nowS) { return nowS >= 1735689600; }   // 2025-01-01T00:00:00Z

TileUse classifyTile(const TileFreshness& f, int64_t nowS, bool clockSane) {
    if (!clockSane || nowS < f.freshUntil) return TileUse::Fresh;   // a wrong clock cannot judge age
    if (nowS < f.freshUntil + f.staleWhileRevalidate) return TileUse::StaleRevalidate;
    if (nowS < f.freshUntil + f.staleIfError) return TileUse::StaleIfError;
    return TileUse::Expired;
}

TileFreshness freshnessAfter304(const TileFreshness& stored, const CacheControl& cc, bool hasCacheControl,
                                int64_t dateS, int64_t expiresS, int64_t lastModifiedS, int64_t ageS, int64_t nowS) {
    if (hasCacheControl || expiresS >= 0)
        return freshnessFromHeaders(cc, dateS, expiresS, lastModifiedS, ageS, nowS);
    // RFC 9111 4.3.4: what the 304 does not say keeps its stored value.
    TileFreshness f        = stored;
    const int64_t lifetime = std::max<int64_t>(0, stored.freshUntil - stored.fetchedAt);
    f.fetchedAt            = nowS;
    f.freshUntil           = nowS + std::max<int64_t>(0, lifetime - std::max<int64_t>(0, ageS));
    return f;
}

// ─── SQLite cache ─────────────────────────────────────────────────────────────

namespace {
/// Errors after which the file cannot be trusted: rebuild it (it is only a cache).
bool corruptCode(int rc) {
    const int primary = rc & 0xff;
    return primary == SQLITE_CORRUPT || primary == SQLITE_NOTADB;
}
} // namespace

struct TileCache::Impl {
    mutable std::mutex m;
    sqlite3*  db = nullptr;
    std::string path;
    uint64_t  budget = 0;
    bool      broken = false;   ///< Corruption seen at run time: no further use until reopened.
    std::string notice;         ///< One pending message for the owner's log (takeNotice()).
    // Read lock-free by stats(); written under m.
    std::atomic<uint64_t> bytes{0}, count{0}, evictions{0};
    sqlite3_stmt *get = nullptr, *put = nullptr, *refresh = nullptr, *touch = nullptr, *size = nullptr;

    void finalize() {
        for (sqlite3_stmt** s : {&get, &put, &refresh, &touch, &size}) {
            if (*s) sqlite3_finalize(*s);
            *s = nullptr;
        }
        if (db) sqlite3_close(db);
        db = nullptr;
    }
    int exec(const char* sql) { return sqlite3_exec(db, sql, nullptr, nullptr, nullptr); }
    /// Full recount, only at open.  length(png) reads the size from the record
    /// header; a column stored after the blob would walk every overflow page.
    int recount() {
        sqlite3_stmt* s  = nullptr;
        int           rc = sqlite3_prepare_v2(db, "SELECT COALESCE(SUM(length(png)),0), COUNT(*) FROM tiles", -1, &s,
                                              nullptr);
        if (rc == SQLITE_OK) rc = sqlite3_step(s);
        if (rc == SQLITE_ROW) {
            bytes = uint64_t(sqlite3_column_int64(s, 0));
            count = uint64_t(sqlite3_column_int64(s, 1));
            rc    = SQLITE_OK;
        }
        sqlite3_finalize(s);
        return rc;
    }
    /// Size of the stored tile (0 = none) - m held.
    uint64_t storedSize(const std::string& provider, const TileKey& k, bool& exists);
    /// One open attempt; returns the SQLite code of the first failure.
    int openOnce(std::string& why);
    void noteError(int rc, const char* what) {
        if (corruptCode(rc) && !broken) {
            broken = true;
            notice = "tile cache " + path + ": " + what + " (" + sqlite3_errstr(rc) +
                     ") - not used until restart, when it is rebuilt";
        }
    }
};

int TileCache::Impl::openOnce(std::string& why) {
    int rc = sqlite3_open_v2(path.c_str(), &db, SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE | SQLITE_OPEN_FULLMUTEX,
                             nullptr);
    if (rc != SQLITE_OK) {
        why = std::string("open ") + path + ": " + (db ? sqlite3_errmsg(db) : "out of memory");
        return rc;
    }
    sqlite3_busy_timeout(db, 200);
    // WAL + NORMAL: a crash may lose the newest tiles, never corrupt the file.
    if ((rc = exec("PRAGMA journal_mode=WAL")) != SQLITE_OK || (rc = exec("PRAGMA synchronous=NORMAL")) != SQLITE_OK ||
        (rc = exec("CREATE TABLE IF NOT EXISTS tiles(provider TEXT NOT NULL, z INTEGER NOT NULL, x INTEGER NOT NULL,"
                   " y INTEGER NOT NULL, png BLOB NOT NULL, etag TEXT, last_modified TEXT, fetched_at INTEGER,"
                   " fresh_until INTEGER, swr INTEGER, sie INTEGER, accessed_at INTEGER, bytes INTEGER,"
                   " PRIMARY KEY(provider, z, x, y))")) != SQLITE_OK ||
        (rc = exec("CREATE INDEX IF NOT EXISTS tiles_lru ON tiles(accessed_at)")) != SQLITE_OK) {
        why = std::string("schema: ") + sqlite3_errmsg(db);
        return rc;
    }
    struct {
        sqlite3_stmt** stmt;
        const char*    sql;
    } const prep[] = {
        {&get, "SELECT png, etag, last_modified, fetched_at, fresh_until, swr, sie, accessed_at FROM tiles"
               " WHERE provider=? AND z=? AND x=? AND y=?"},
        {&put, "INSERT OR REPLACE INTO tiles(provider,z,x,y,png,etag,last_modified,fetched_at,fresh_until,"
               "swr,sie,accessed_at,bytes) VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?)"},
        {&refresh, "UPDATE tiles SET fetched_at=?, fresh_until=?, swr=?, sie=?, accessed_at=?,"
                   " etag=COALESCE(NULLIF(?,''),etag), last_modified=COALESCE(NULLIF(?,''),last_modified)"
                   " WHERE provider=? AND z=? AND x=? AND y=?"},
        {&touch, "UPDATE tiles SET accessed_at=? WHERE provider=? AND z=? AND x=? AND y=?"},
        {&size, "SELECT length(png) FROM tiles WHERE provider=? AND z=? AND x=? AND y=?"},
    };
    for (const auto& p : prep)
        if ((rc = sqlite3_prepare_v2(db, p.sql, -1, p.stmt, nullptr)) != SQLITE_OK) {
            why = std::string("prepare: ") + sqlite3_errmsg(db);
            return rc;
        }
    if ((rc = recount()) != SQLITE_OK) {
        why = std::string("reading the size: ") + sqlite3_errmsg(db);
        return rc;
    }
    return SQLITE_OK;
}

TileCache::TileCache() : impl_(std::make_unique<Impl>()) {}
TileCache::~TileCache() { close(); }

bool TileCache::open(const std::string& path, uint64_t budgetBytes, std::string* why) {
    std::lock_guard<std::mutex> lk(impl_->m);
    Impl& m = *impl_;
    m.finalize();
    m.path   = path;
    m.budget = budgetBytes;
    m.broken = false;
    m.notice.clear();
    m.bytes = m.count = 0;
    std::error_code ec;
    const std::filesystem::path p(path);
    if (p.has_parent_path()) std::filesystem::create_directories(p.parent_path(), ec);
    std::string err;
    int rc = m.openOnce(err);
    if (rc != SQLITE_OK && corruptCode(rc)) {
        // A damaged file: set it aside and start an empty cache.
        m.finalize();
        for (const char* suffix : {"", "-wal", "-shm"}) {
            const std::filesystem::path f(path + suffix);
            if (std::filesystem::exists(f, ec)) std::filesystem::rename(f, path + suffix + ".corrupt", ec);
        }
        m.notice = "tile cache " + path + " was corrupt (" + err + ") - set aside as *.corrupt, starting empty";
        rc = m.openOnce(err);
    }
    if (rc != SQLITE_OK) {
        m.finalize();
        if (why) *why = err;
        return false;
    }
    return true;
}

void TileCache::close() {
    std::lock_guard<std::mutex> lk(impl_->m);
    impl_->finalize();
}

namespace {
void bindKey(sqlite3_stmt* s, int first, const std::string& provider, const TileKey& k) {
    sqlite3_bind_text(s, first, provider.c_str(), int(provider.size()), SQLITE_TRANSIENT);
    sqlite3_bind_int(s, first + 1, k.z);
    sqlite3_bind_int64(s, first + 2, k.x);
    sqlite3_bind_int64(s, first + 3, k.y);
}
std::string columnText(sqlite3_stmt* s, int i) {
    const unsigned char* t = sqlite3_column_text(s, i);
    return t ? reinterpret_cast<const char*>(t) : "";
}
} // namespace

uint64_t TileCache::Impl::storedSize(const std::string& provider, const TileKey& k, bool& exists) {
    sqlite3_reset(size);
    sqlite3_clear_bindings(size);
    bindKey(size, 1, provider, k);
    const int rc = sqlite3_step(size);
    exists = rc == SQLITE_ROW;
    const uint64_t n = exists ? uint64_t(sqlite3_column_int64(size, 0)) : 0;
    if (rc != SQLITE_ROW && rc != SQLITE_DONE) noteError(rc, "read failed");
    sqlite3_reset(size);
    return n;
}

bool TileCache::get(const std::string& provider, const TileKey& k, CachedTile& out, int64_t nowS) {
    std::lock_guard<std::mutex> lk(impl_->m);
    Impl& m = *impl_;
    if (!m.db || m.broken) return false;
    sqlite3_reset(m.get);
    sqlite3_clear_bindings(m.get);
    bindKey(m.get, 1, provider, k);
    const int rc = sqlite3_step(m.get);
    if (rc != SQLITE_ROW) {
        if (rc != SQLITE_DONE) m.noteError(rc, "read failed");
        sqlite3_reset(m.get);
        return false;
    }
    const void* blob = sqlite3_column_blob(m.get, 0);
    const int   n    = sqlite3_column_bytes(m.get, 0);
    out.png.assign(static_cast<const uint8_t*>(blob), static_cast<const uint8_t*>(blob) + n);
    out.etag                       = columnText(m.get, 1);
    out.lastModified               = columnText(m.get, 2);
    out.fresh.fetchedAt            = sqlite3_column_int64(m.get, 3);
    out.fresh.freshUntil           = sqlite3_column_int64(m.get, 4);
    out.fresh.staleWhileRevalidate = sqlite3_column_int64(m.get, 5);
    out.fresh.staleIfError         = sqlite3_column_int64(m.get, 6);
    const int64_t accessed         = sqlite3_column_int64(m.get, 7);
    sqlite3_reset(m.get);
    // LRU bookkeeping at most hourly per tile: reads must not turn into writes.
    if (clockPlausible(nowS) && nowS - accessed > 3600) {
        sqlite3_reset(m.touch);
        sqlite3_bind_int64(m.touch, 1, nowS);
        bindKey(m.touch, 2, provider, k);
        sqlite3_step(m.touch);
        sqlite3_reset(m.touch);
    }
    return true;
}

bool TileCache::put(const std::string& provider, const TileKey& k, const CachedTile& t, int64_t nowS) {
    std::lock_guard<std::mutex> lk(impl_->m);
    Impl& m = *impl_;
    if (!m.db || m.broken || t.png.empty()) return false;
    bool existed = false;
    const uint64_t old = m.storedSize(provider, k, existed);
    sqlite3_reset(m.put);
    sqlite3_clear_bindings(m.put);
    bindKey(m.put, 1, provider, k);
    sqlite3_bind_blob(m.put, 5, t.png.data(), int(t.png.size()), SQLITE_TRANSIENT);
    sqlite3_bind_text(m.put, 6, t.etag.c_str(), int(t.etag.size()), SQLITE_TRANSIENT);
    sqlite3_bind_text(m.put, 7, t.lastModified.c_str(), int(t.lastModified.size()), SQLITE_TRANSIENT);
    sqlite3_bind_int64(m.put, 8, t.fresh.fetchedAt);
    sqlite3_bind_int64(m.put, 9, t.fresh.freshUntil);
    sqlite3_bind_int64(m.put, 10, t.fresh.staleWhileRevalidate);
    sqlite3_bind_int64(m.put, 11, t.fresh.staleIfError);
    sqlite3_bind_int64(m.put, 12, nowS);
    sqlite3_bind_int64(m.put, 13, int64_t(t.png.size()));
    const int  rc = sqlite3_step(m.put);
    const bool ok = rc == SQLITE_DONE;
    sqlite3_reset(m.put);
    if (!ok) {
        m.noteError(rc, "write failed");
        return false;
    }
    m.bytes = m.bytes.load() - std::min<uint64_t>(m.bytes.load(), old) + t.png.size();
    if (!existed) ++m.count;
    return true;
}

bool TileCache::refresh(const std::string& provider, const TileKey& k, const TileFreshness& f,
                        const std::string& etag, const std::string& lastModified, int64_t nowS) {
    std::lock_guard<std::mutex> lk(impl_->m);
    Impl& m = *impl_;
    if (!m.db || m.broken) return false;
    sqlite3_reset(m.refresh);
    sqlite3_clear_bindings(m.refresh);
    sqlite3_bind_int64(m.refresh, 1, f.fetchedAt);
    sqlite3_bind_int64(m.refresh, 2, f.freshUntil);
    sqlite3_bind_int64(m.refresh, 3, f.staleWhileRevalidate);
    sqlite3_bind_int64(m.refresh, 4, f.staleIfError);
    sqlite3_bind_int64(m.refresh, 5, nowS);
    sqlite3_bind_text(m.refresh, 6, etag.c_str(), int(etag.size()), SQLITE_TRANSIENT);
    sqlite3_bind_text(m.refresh, 7, lastModified.c_str(), int(lastModified.size()), SQLITE_TRANSIENT);
    bindKey(m.refresh, 8, provider, k);
    const int  rc = sqlite3_step(m.refresh);
    const bool ok = rc == SQLITE_DONE && sqlite3_changes(m.db) == 1;
    if (rc != SQLITE_DONE) m.noteError(rc, "write failed");
    sqlite3_reset(m.refresh);
    return ok;
}

void TileCache::evict(const std::string& provider, const std::vector<TileKey>& keep) {
    std::lock_guard<std::mutex> lk(impl_->m);
    Impl& m = *impl_;
    if (!m.db || m.broken || m.bytes.load() <= m.budget) return;
    std::set<std::tuple<std::string, int, int64_t, int64_t>> protect;
    for (const TileKey& k : keep) protect.insert({provider, k.z, int64_t(k.x), int64_t(k.y)});
    // Down to 90% of the budget so a full cache does not evict on every insert.
    const uint64_t target = m.budget - m.budget / 10;
    sqlite3_stmt* sel = nullptr;
    sqlite3_stmt* del = nullptr;
    if (sqlite3_prepare_v2(m.db, "SELECT provider, z, x, y, length(png) FROM tiles ORDER BY accessed_at ASC", -1,
                           &sel, nullptr) != SQLITE_OK ||
        sqlite3_prepare_v2(m.db, "DELETE FROM tiles WHERE provider=? AND z=? AND x=? AND y=?", -1, &del, nullptr) !=
            SQLITE_OK) {
        sqlite3_finalize(sel);
        sqlite3_finalize(del);
        return;
    }
    struct Victim {
        std::tuple<std::string, int, int64_t, int64_t> key;
        uint64_t                                       size;
    };
    std::vector<Victim> victims;
    uint64_t bytes = m.bytes.load();
    int      rc    = SQLITE_ROW;
    while (bytes > target && (rc = sqlite3_step(sel)) == SQLITE_ROW) {
        auto key = std::make_tuple(columnText(sel, 0), sqlite3_column_int(sel, 1), sqlite3_column_int64(sel, 2),
                                   sqlite3_column_int64(sel, 3));
        if (protect.count(key)) continue;
        const uint64_t size = uint64_t(sqlite3_column_int64(sel, 4));
        bytes -= std::min<uint64_t>(bytes, size);
        victims.push_back(Victim{key, size});
    }
    if (rc != SQLITE_ROW && rc != SQLITE_DONE) m.noteError(rc, "scan failed");
    sqlite3_finalize(sel);
    // Counters change only once the deletions are committed.
    uint64_t freed = 0, removed = 0;
    int      txn   = m.exec("BEGIN");
    for (const Victim& v : victims) {
        if (txn != SQLITE_OK) break;
        sqlite3_reset(del);
        sqlite3_bind_text(del, 1, std::get<0>(v.key).c_str(), int(std::get<0>(v.key).size()), SQLITE_TRANSIENT);
        sqlite3_bind_int(del, 2, std::get<1>(v.key));
        sqlite3_bind_int64(del, 3, std::get<2>(v.key));
        sqlite3_bind_int64(del, 4, std::get<3>(v.key));
        const int rc2 = sqlite3_step(del);
        if (rc2 == SQLITE_DONE && sqlite3_changes(m.db) == 1) {
            freed += v.size;
            ++removed;
        } else if (rc2 != SQLITE_DONE) {
            m.noteError(rc2, "eviction failed");
        }
    }
    sqlite3_finalize(del);
    if (txn == SQLITE_OK) txn = m.exec("COMMIT");
    if (txn == SQLITE_OK) {
        m.evictions += removed;
        m.bytes = m.bytes.load() - std::min<uint64_t>(m.bytes.load(), freed);
        m.count = m.count.load() - std::min<uint64_t>(m.count.load(), removed);
    } else {
        m.exec("ROLLBACK");
        m.noteError(txn, "eviction commit failed");
        m.recount();   // whatever the file now holds
    }
}

std::string TileCache::takeNotice() {
    std::lock_guard<std::mutex> lk(impl_->m);
    std::string n;
    n.swap(impl_->notice);
    return n;
}

uint64_t TileCache::totalBytes() const { return impl_->bytes.load(); }
uint64_t TileCache::tileCount() const { return impl_->count.load(); }
uint64_t TileCache::evictions() const { return impl_->evictions.load(); }

} // namespace dashcam::navigation
