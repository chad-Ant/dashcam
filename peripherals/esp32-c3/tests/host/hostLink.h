#pragma once

/**
 * @file hostLink.h
 * @brief Host stand-in for lib/hostLink — the Jetson's side, scriptable.
 *
 * Shadows the real header because the Makefile puts this directory ahead of
 * lib/hostLink. src/main.cpp talks to HostLink only through the accessors
 * below, so a test can play the Jetson by setting what those accessors return
 * and read back every telemetry frame the bridge hands up.
 *
 * The session rules are the REAL hostLink's, restated: a HELLO (or the first
 * frame after silence) resets streaming, decimation and every request not yet
 * taken, and raises connectSeen(). A test that wants "the host sent X after
 * HELLO in the same USB read" calls hostHello() then X before the next loop().
 * The wire protocol itself is tested by the Jetson-side commlink_test.
 */

#include <Arduino.h>
#include <vector>
#include <string>

#include "HostProtocol.h"   // the real one, from lib/hostLink

class HostLink {
public:
    void begin() {}
    bool poll(uint32_t) { return false; }

    bool isConnected() const { return connected_; }
    bool connectSeen()       { const bool c = connectSeen_; connectSeen_ = false; return c; }
    bool onceRequested()     { const bool r = onceReq_;     onceReq_     = false; return r; }
    bool statusRequested()   { const bool r = statusReq_;   statusReq_   = false; return r; }
    uint8_t takeCanModeRequest() { return 0; }
    uint8_t takeImuModeRequest() { return 0; }
    bool takeCanFilterRequest(uint16_t *, uint8_t &) { return false; }

    bool    streaming()  const { return streaming_; }
    uint8_t decimation() const { return decim_; }

    bool sendTelemetry(const hostproto::Telemetry &t)
    {
        if (txFull) return false;
        telemetry.push_back(t);
        return true;
    }
    bool sendStatus(const hostproto::BridgeStatus &) { return true; }
    bool sendHello(const hostproto::Hello &)         { ++hellos; return true; }
    bool sendLog(uint8_t, const char *text)          { logs.push_back(text); return true; }

    uint32_t framesRx()  const { return 0; }
    uint32_t txDropped() const { return 0; }

    // ── the Jetson, scripted ──
    void hostHello()
    {
        connected_ = true;  connectSeen_ = true;
        streaming_ = false; decim_ = 1;
        onceReq_   = false; statusReq_ = false;
    }
    void hostGetOnce()           { onceReq_ = true; }
    void hostStartStream()       { streaming_ = true; }
    void hostSetDecim(uint8_t n) { decim_ = n; }

    bool txFull = false;                          ///< The CDC ring is full.
    std::vector<hostproto::Telemetry> telemetry;  ///< Every frame that went up.
    std::vector<std::string>          logs;       ///< Every sendLog() line.
    unsigned                          hellos = 0;

private:
    bool    connected_   = false;
    bool    connectSeen_ = false;
    bool    onceReq_     = false;
    bool    statusReq_   = false;
    bool    streaming_   = false;
    uint8_t decim_       = 1;
};
