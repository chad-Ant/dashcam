/**
 * @file libnetwork_rtp.cpp
 * @brief RtpSession — control/session layer for H.264-over-RTP inference streams.
 *
 * Produces the GStreamer bin description, the SDP, and a viewer command for one
 * RTP output.  No GStreamer linkage: this is pure string/parameter logic.  The
 * application parses branchDescription() and attaches it to the camera tee.
 */

#include "libnetwork.h"

#include <algorithm>
#include <cmath>

namespace dashcam::network {

namespace {
std::string gstQuoted(const std::string& value) {
    std::string out;
    out.reserve(value.size() + 2);
    out.push_back('"');
    for (char ch : value) {
        if (ch == '\\' || ch == '"') out.push_back('\\');
        out.push_back(ch);
    }
    out.push_back('"');
    return out;
}
} // namespace

std::string RtpSession::branchDescription(bool nvmm, float fps,
                                          const std::string& sinkName) const {
    const int fpsI      = (fps > 0.0f) ? std::max(1, static_cast<int>(std::lround(fps))) : 30;
    const int keyFrames = std::max(1, cfg_.keyIntSec * fpsI);

    // NVMM (CSI/Argus) tee source → nvvidconv (VIC) down to system-memory I420;
    // system-memory raw (USB) source → CPU videoconvert.  Either way x264enc
    // (Orin Nano has no NVENC) consumes I420 and pushes an RTP/H.264 stream.
    const std::string head = nvmm
        ? "nvvidconv ! video/x-raw,format=(string)I420"
        : "videoconvert ! video/x-raw,format=(string)I420";

    // A named udpsink lets the app fetch it (gst_bin_get_by_name) and re-point
    // host/port at runtime — the dynamic RTP destination feature.
    const std::string sink = sinkName.empty()
        ? std::string("udpsink")
        : "udpsink name=" + sinkName;

    return head +
        " ! x264enc tune=zerolatency speed-preset=ultrafast bitrate=" +
            std::to_string(cfg_.bitrateKbps) +
        " key-int-max=" + std::to_string(keyFrames) +
        " ! video/x-h264,profile=(string)baseline" +
        " ! rtph264pay config-interval=1 pt=96" +
        " ! " + sink +
        " host=" + gstQuoted(cfg_.host) +
        " port=" + std::to_string(cfg_.port) +
        " sync=false async=false";
}

std::string RtpSession::sdp() const {
    return std::string(
        "v=0\r\n"
        "o=- 0 0 IN IP4 ") + cfg_.host + "\r\n"
        "s=dashcam-rtp\r\n"
        "c=IN IP4 " + cfg_.host + "\r\n"
        "t=0 0\r\n"
        "m=video " + std::to_string(cfg_.port) + " RTP/AVP 96\r\n"
        "a=rtpmap:96 H264/90000\r\n";
}

std::string RtpSession::viewerHint() const {
    return "gst-launch-1.0 udpsrc port=" + std::to_string(cfg_.port) +
           " caps=\"application/x-rtp,media=video,encoding-name=H264,payload=96\""
           " ! rtpjitterbuffer latency=100 ! rtph264depay ! h264parse"
           " ! avdec_h264 ! autovideosink";
}

} // namespace dashcam::network
