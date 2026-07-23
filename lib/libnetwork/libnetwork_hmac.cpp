/**
 * @file libnetwork_hmac.cpp
 * @brief Dependency-free SHA-256 + HMAC-SHA256 + a random-nonce helper.
 *
 * The remote-control channel (ControlServer) authenticates clients with a nonce
 * challenge answered by HMAC-SHA256 over a pre-shared key.  libnetwork must stay
 * free of external crypto dependencies (no OpenSSL link), so a small, self-
 * contained SHA-256 (FIPS 180-4) and HMAC (RFC 2104) live here.  This is used
 * only for a private-LAN control channel — it is authentication, not transport
 * encryption; TLS would be the answer if the channel ever crossed a trust
 * boundary.
 *
 * Validated against the SHA-256("abc") and RFC 4231 HMAC known-answer vectors in
 * network_test.
 */

#include "libnetwork.h"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include <cerrno>
#include <fcntl.h>
#include <unistd.h>
#if defined(__linux__)
#include <sys/random.h>
#endif

namespace dashcam::network::detail {

namespace {

// Lowercase-hex encode a byte buffer.
std::string toHex(const uint8_t* d, size_t n) {
    static const char* kHx = "0123456789abcdef";
    std::string out;
    out.reserve(n * 2);
    for (size_t i = 0; i < n; ++i) {
        out.push_back(kHx[d[i] >> 4]);
        out.push_back(kHx[d[i] & 0x0f]);
    }
    return out;
}

// Minimal streaming SHA-256 (FIPS 180-4).  Not constant-time — it hashes only
// the (public) nonce and the pre-shared key, never attacker-chosen secrets in a
// timing-sensitive comparison (that comparison is done separately, constant-time).
class Sha256 {
public:
    Sha256() { reset(); }

    void reset() {
        len_ = 0;
        bufLen_ = 0;
        h_[0] = 0x6a09e667; h_[1] = 0xbb67ae85; h_[2] = 0x3c6ef372; h_[3] = 0xa54ff53a;
        h_[4] = 0x510e527f; h_[5] = 0x9b05688c; h_[6] = 0x1f83d9ab; h_[7] = 0x5be0cd19;
    }

    void update(const uint8_t* data, size_t n) {
        len_ += n;
        while (n > 0) {
            const size_t take = std::min(n, size_t(64) - bufLen_);
            std::memcpy(buf_ + bufLen_, data, take);
            bufLen_ += take;
            data    += take;
            n       -= take;
            if (bufLen_ == 64) { block(buf_); bufLen_ = 0; }
        }
    }

    void finish(uint8_t out[32]) {
        const uint64_t bitLen = len_ * 8;             // captured before padding
        const uint8_t  one    = 0x80;
        update(&one, 1);
        const uint8_t zero = 0x00;
        while (bufLen_ != 56) update(&zero, 1);       // pad to 56 mod 64
        uint8_t lenBytes[8];
        for (int i = 0; i < 8; ++i)
            lenBytes[i] = static_cast<uint8_t>(bitLen >> (56 - 8 * i));
        update(lenBytes, 8);                          // 64-bit big-endian length
        for (int i = 0; i < 8; ++i) {
            out[4 * i + 0] = static_cast<uint8_t>(h_[i] >> 24);
            out[4 * i + 1] = static_cast<uint8_t>(h_[i] >> 16);
            out[4 * i + 2] = static_cast<uint8_t>(h_[i] >> 8);
            out[4 * i + 3] = static_cast<uint8_t>(h_[i]);
        }
    }

private:
    static uint32_t ror(uint32_t x, int n) { return (x >> n) | (x << (32 - n)); }

    void block(const uint8_t* p) {
        static const uint32_t K[64] = {
            0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
            0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
            0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
            0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
            0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
            0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
            0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
            0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2 };

        uint32_t w[64];
        for (int i = 0; i < 16; ++i)
            w[i] = (uint32_t(p[4 * i + 0]) << 24) | (uint32_t(p[4 * i + 1]) << 16) |
                   (uint32_t(p[4 * i + 2]) << 8)  |  uint32_t(p[4 * i + 3]);
        for (int i = 16; i < 64; ++i) {
            const uint32_t s0 = ror(w[i - 15], 7) ^ ror(w[i - 15], 18) ^ (w[i - 15] >> 3);
            const uint32_t s1 = ror(w[i - 2], 17) ^ ror(w[i - 2], 19)  ^ (w[i - 2] >> 10);
            w[i] = w[i - 16] + s0 + w[i - 7] + s1;
        }

        uint32_t a = h_[0], b = h_[1], c = h_[2], d = h_[3];
        uint32_t e = h_[4], f = h_[5], g = h_[6], h = h_[7];
        for (int i = 0; i < 64; ++i) {
            const uint32_t S1  = ror(e, 6) ^ ror(e, 11) ^ ror(e, 25);
            const uint32_t ch  = (e & f) ^ (~e & g);
            const uint32_t t1  = h + S1 + ch + K[i] + w[i];
            const uint32_t S0  = ror(a, 2) ^ ror(a, 13) ^ ror(a, 22);
            const uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
            const uint32_t t2  = S0 + maj;
            h = g; g = f; f = e; e = d + t1; d = c; c = b; b = a; a = t1 + t2;
        }
        h_[0] += a; h_[1] += b; h_[2] += c; h_[3] += d;
        h_[4] += e; h_[5] += f; h_[6] += g; h_[7] += h;
    }

    uint32_t h_[8];
    uint8_t  buf_[64];
    size_t   bufLen_;
    uint64_t len_;
};

} // namespace

std::string hmacSha256Hex(const std::string& key, const std::string& msg) {
    // RFC 2104: key shortened to its own hash if longer than the block size.
    uint8_t k0[64];
    std::memset(k0, 0, sizeof(k0));
    if (key.size() > 64) {
        Sha256 s;
        s.update(reinterpret_cast<const uint8_t*>(key.data()), key.size());
        s.finish(k0);   // remaining 32 bytes stay zero-padded
    } else {
        std::memcpy(k0, key.data(), key.size());
    }

    uint8_t ipad[64], opad[64];
    for (int i = 0; i < 64; ++i) {
        ipad[i] = static_cast<uint8_t>(k0[i] ^ 0x36);
        opad[i] = static_cast<uint8_t>(k0[i] ^ 0x5c);
    }

    uint8_t inner[32];
    {
        Sha256 s;
        s.update(ipad, 64);
        s.update(reinterpret_cast<const uint8_t*>(msg.data()), msg.size());
        s.finish(inner);
    }
    uint8_t outer[32];
    {
        Sha256 s;
        s.update(opad, 64);
        s.update(inner, 32);
        s.finish(outer);
    }
    return toHex(outer, 32);
}

std::string randomHex(size_t nBytes) {
    std::vector<uint8_t> b(nBytes);
    size_t got = 0;

#if defined(__linux__)
    while (got < nBytes) {
        const ssize_t r = ::getrandom(b.data() + got, nBytes - got, 0);
        if (r < 0) { if (errno == EINTR) continue; break; }
        got += static_cast<size_t>(r);
    }
#endif
    if (got < nBytes) {   // getrandom unavailable (old kernel) → /dev/urandom
        const int fd = ::open("/dev/urandom", O_RDONLY);
        if (fd >= 0) {
            while (got < nBytes) {
                const ssize_t r = ::read(fd, b.data() + got, nBytes - got);
                if (r <= 0) { if (r < 0 && errno == EINTR) continue; break; }
                got += static_cast<size_t>(r);
            }
            ::close(fd);
        }
    }
    if (got < nBytes) {   // last-resort fill so the nonce is never short/predictably empty
        uint64_t seed = static_cast<uint64_t>(
            std::chrono::steady_clock::now().time_since_epoch().count());
        for (; got < nBytes; ++got) {
            seed = seed * 6364136223846793005ULL + 1442695040888963407ULL;
            b[got] = static_cast<uint8_t>(seed >> 33);
        }
    }
    return toHex(b.data(), nBytes);
}

} // namespace dashcam::network::detail
