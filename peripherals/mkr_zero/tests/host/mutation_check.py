"""Reintroduce IMU, BNO055 bring-up, CAN, raw-stream and sketch-policy regressions in temporary copies; never edit the checkout.

Requires Python 3, make and a hosted C++11 compiler. A compile failure is NOT a
killed mutant: every mutant must build and then fail the corresponding suite.
"""
import pathlib
import shutil
import subprocess
import tempfile

HERE = pathlib.Path(__file__).resolve().parent
MKR = HERE.parents[1]
IMU = "IMUFunctions.cpp"
INIT = "BNO055Init.cpp"
CAN = "CANSniffFunctions.cpp"
RING = "CANFrameRing.h"
STREAM = "CANRawStream.cpp"
OVR = "gOverruns = gOverruns + ((eflg & 0x40u) ? 1u : 0u) + ((eflg & 0x80u) ? 1u : 0u);"
CORE = "(st & BNO055_ST_CORE_PASSED) == BNO055_ST_CORE_PASSED"

# name, source, suite, exact unique old text, replacement
MUTATIONS = [
    ("self-test gate omitted", IMU, "imu_tests", CORE, "((void)st, true)"),
    ("reserved ST bits rejected", IMU, "imu_tests", CORE,
     "((st & 0xF0u) == 0u) && (" + CORE + ")"),
    ("reserved INT bits rejected", IMU, "imu_tests", "intSta & BNO055_INT_MOTION_MASK &", "intSta & 0xFFu &"),
    ("INT integrity omitted", IMU, "imu_tests", "intStaSound && stSound && !zeroed", "(intStaSound || true) && stSound && !zeroed"),
    ("zero burst accepted", IMU, "imu_tests", "intStaSound && stSound && !zeroed", "intStaSound && stSound && (!zeroed || true)"),
    ("zero gyro alone rejected", IMU, "imu_tests", "buf[IMU_OFF_ACCEL + i] != 0u || buf[IMU_OFF_GYRO + i] != 0u", "buf[IMU_OFF_GYRO + i] != 0u"),
    ("unconfirmed pin fires", IMU, "imu_tests",
     "if (pin && !pinUsed) countRejected(dev);  // a naked edge is not an impact",
     "if (pin && !pinUsed){ recordHighG(dev, pinMs, now, busAnswered); return; }"),
    ("held line believed without a probe", IMU, "imu_tests",
     "const bool registerFired = burstSound && (intSta & IMU_INT_STA_HIGH_G) != 0u;",
     "const bool registerFired = (burstSound && (intSta & IMU_INT_STA_HIGH_G) != 0u) || (line && !dev.intLineDistrusted);"),
    ("held line never considered", IMU, "imu_tests",
     "    if (line && !dev.intLineDistrusted){\n        // High with no register evidence",
     "    if (false){\n        // High with no register evidence"),
    ("probed on first sight", IMU, "imu_tests",
     "        dev.lineCandidate = true;\n        dev.lineSinceMs   = pin ? pinMs : now;",
     "        dev.lineProbe   = clearHighGLatch(dev);\n        dev.lineSinceMs = pin ? pinMs : now;"),
    ("self-dropped candidate probed anyway", IMU, "imu_tests",
     "        if (!line || pin){\n            // Nothing wrote RST_INT",
     "        if (false){\n            // Nothing wrote RST_INT"),
    ("self-dropped candidate not counted", IMU, "imu_tests",
     "            countRejected(dev);\n            pinUsed = true;",
     "            pinUsed = true;"),
    ("probe written without a bus", IMU, "imu_tests",
     "        } else if (busAnswered){\n            // Held a whole poll",
     "        } else if (true){\n            // Held a whole poll"),
    ("no stuck verdict", IMU, "imu_tests",
     "            dev.intLineDistrusted = true;\n", ""),
    ("stuck concluded from a corrupt read", IMU, "imu_tests",
     "        } else if (burstSound){\n            // Cleared, still high", "        } else {\n            // Cleared, still high"),
    ("trust never restored", IMU, "imu_tests",
     "    if (dev.intLineDistrusted && (!line || pin)) dev.intLineDistrusted = false;", ""),
    ("re-latch edge ignored by the probe", IMU, "imu_tests",
     "        if (!line || pin){\n            // RST_INT released it",
     "        if (!line){\n            // RST_INT released it"),
    ("released probe clears again", IMU, "imu_tests",
     "recordHighG(dev, dev.lineSinceMs, now, line && busAnswered);",
     "recordHighG(dev, dev.lineSinceMs, now, busAnswered);"),
    ("re-latched probe not cleared", IMU, "imu_tests",
     "recordHighG(dev, dev.lineSinceMs, now, line && busAnswered);",
     "recordHighG(dev, dev.lineSinceMs, now, false);"),
    ("NACK cannot decide a released probe", IMU, "imu_tests",
     "        if (!line || pin){\n            // RST_INT released it",
     "        if (busAnswered && (!line || pin)){\n            // RST_INT released it"),
    ("RST_INT not written", IMU, "imu_tests",
     "if (regWrite8Imu(dev, BNO055_SYS_TRIGGER_ADDR, trigger)) return true;",
     "if (true || regWrite8Imu(dev, BNO055_SYS_TRIGGER_ADDR, trigger)) return true;"),
    ("NACK keeps glitch edge uncounted", IMU, "imu_tests",
     "        noteHighG(dev, 0u, false, false, now);\n", "        if (false) noteHighG(dev, 0u, false, false, now);\n"),
    ("NACK counts a held latch as rejected", IMU, "imu_tests",
     "    if (line && !dev.intLineDistrusted){\n        // High with no register evidence",
     "    if (line && !dev.intLineDistrusted && busAnswered){\n        // High with no register evidence"),
    ("open episode dropped silently (retire / re-init)", IMU, "imu_tests",
     "    } else if (dev.lineProbe || dev.lineCandidate){\n        countRejected(dev);\n    }",
     "    }"),
    ("re-init drops an open episode", IMU, "imu_tests",
     "    settleLineEpisode(dev);   // imuMarkAbsent() wipes it", "    // imuMarkAbsent() wipes it"),
    ("released probe ignored (retire / re-init)", IMU, "imu_tests",
     "    if (dev.lineProbe && !intLineHigh(dev)){\n        recordHighG(dev, dev.lineSinceMs, millis(), false);\n    } else if",
     "    if (false){\n    } else if"),
    ("line sampled before the edge", IMU, "imu_tests",
     "    bool pin = takePinEdge(pinMs);\n    const bool line = intLineHigh(dev);",
     "    const bool line = intLineHigh(dev);\n    bool pin = takePinEdge(pinMs);"),
    ("mid-sample edge not merged", IMU, "imu_tests",
     "if (takePinEdge(lateMs) && !pin){ pin = true; pinMs = lateMs; }", "(void)lateMs;"),
    ("bring-up keeps edge", IMU, "imu_tests", "        gPinEvent = false;\n        interrupts();", "        interrupts();"),
    ("recovery erases hgrej", IMU, "imu_tests", "// highGRejected is boot-cumulative: recovery must not erase the evidence.", "dev.highGRejected = 0u;"),
    ("recovery erases the High-G count", IMU, "imu_tests",
     "    // Neither may it erase the High-G record:", "    dev.highGCount = 0u;\n    // Neither may it erase the High-G record:"),
    ("recovery cancels the High-G hold", IMU, "imu_tests",
     "    // Neither may it erase the High-G record:", "    dev.highGActive = false;\n    // Neither may it erase the High-G record:"),
    ("mode-switch snapshot reset drops the High-G record", IMU, "imu_tests",
     "    initIMUData(data);\n    publishHighG(dev, data, millis());", "    initIMUData(data);\n    (void)dev;"),
    ("snapshot reset copies the hold without its deadline", IMU, "imu_tests",
     "    initIMUData(data);\n    publishHighG(dev, data, millis());",
     "    initIMUData(data);\n    data.highGEvent = dev.highGActive; data.highGMs = dev.highGAtMs; data.highGCount = dev.highGCount;"),
    ("High-G hold never expires", IMU, "imu_tests",
     "    if (dev.highGActive && (static_cast<int32_t>(now - dev.highGUntilMs) >= 0)){\n        dev.highGActive = false;\n    }\n    data.highGEvent",
     "    (void)now;\n    data.highGEvent"),
    ("recovery erases the last latch time", IMU, "imu_tests",
     "    // Neither may it erase the High-G record:", "    dev.highGAtMs = 0u;\n    // Neither may it erase the High-G record:"),
    ("MAG retires whole sensor", IMU, "imu_tests", CORE,
     "(st & (dev.init.opMode == OPERATION_MODE_AMG ? 0x0Fu : 0x0Du)) == (dev.init.opMode == OPERATION_MODE_AMG ? 0x0Fu : 0x0Du)"),
    ("failed MAG still published", IMU, "imu_tests", "(buf[IMU_OFF_ST_RESULT] & BNO055_ST_MAG_PASSED) == 0u", "false"),
    ("AMG restores the calibration profile", INIT, "bno_init_tests",
     "(state.calibProfile != nullptr) &&\n                                  (state.opMode != OPERATION_MODE_AMG);",
     "(state.calibProfile != nullptr);"),
    ("SYS_ERR failure latches a stale OPR_MODE", INIT, "bno_init_tests",
     "            state.opModeSeen = mode;\n", ""),
    ("bring-up skips self-test", INIT, "bno_init_tests", "!state.selfTestReadOk ||\n                (state.selfTestResult & BNO055_ST_CORE_PASSED) != BNO055_ST_CORE_PASSED", "false"),
    ("interrupt verification only High-G", INIT, "bno_init_tests", "BNO055_INT_BIT_ACC_HIGH_G, BNO055_INT_MOTION_MASK);", "BNO055_INT_BIT_ACC_HIGH_G, BNO055_INT_BIT_ACC_HIGH_G);", 2),
    ("interrupt enables preserved", INIT, "bno_init_tests", "BNO055_INT_BIT_ACC_HIGH_G, BNO055_INT_MOTION_MASK);", "0xECu, BNO055_INT_MOTION_MASK);", 2),
    ("AMG High-G counted at 4 g", INIT, "bno_init_tests", "bno055HighGThresholdLsb(IMU_HIGHG_THRESHOLD_MG, accRange)", "bno055HighGThresholdLsb(IMU_HIGHG_THRESHOLD_MG, ACCEL_RANGE_4G)"),
    ("fixed High-G byte", INIT, "bno_init_tests", "bno055HighGThresholdLsb(IMU_HIGHG_THRESHOLD_MG, accRange)", "(uint8_t)128u"),
    ("CAN overruns cleared uncounted", CAN, "can_probe_tests", OVR, "(void)eflg;"),
    ("CAN overrun count reset with the probe", CAN, "can_probe_tests",
     "void canSniffResetCounters() { gFrames = 0; gMatches = 0; gIdsSeenMask = 0; }",
     "void canSniffResetCounters() { gFrames = 0; gMatches = 0; gIdsSeenMask = 0; gOverruns = 0; }"),
    # ── the raw stream, 2026-10-04 ──────────────────────────────────────────────
    ("ring full one slot early", RING, "can_ring_tests",
     "if ((uint16_t)(h - r.tail) >= CAN_RING_SLOTS) {", "if ((uint16_t)(h - r.tail) >= CAN_RING_SLOTS - 1u) {"),
    ("ring drop not counted", RING, "can_ring_tests", "        r.drops = r.drops + 1u;\n", ""),
    ("DLC 8 not flagged", RING, "can_ring_tests", "((dlc == 8u) ? CAN_RAW_DLC8 : 0u)", "0u"),
    ("drain ignores RXB1", CAN, "can_drain_tests",
     "if ((st & 0x02u) != 0u && taken < CAN_DRAIN_MAX_PER_TICK) {", "if (false) {"),
    ("drain believes a floating MISO", CAN, "can_drain_tests", "if (st & 0xA8u) return;", "if (false) return;"),
    ("drain silenced by a pending TXREQ", CAN, "can_drain_tests", "if (st & 0xA8u) return;", "if (st & 0xFCu) return;"),
    ("drain unbounded", CAN, "can_drain_tests",
     "round < CAN_DRAIN_MAX_PER_TICK && taken < CAN_DRAIN_MAX_PER_TICK; ++round) {",
     "round < 200u && taken < 200u; ++round) {"),
    ("refused frames not counted as drained", CAN, "can_drain_tests",
     "    (void)canRingPush(gRing, f);   // a full ring counts the loss itself",
     "    if (!canRingPush(gRing, f)) gDrained = gDrained - 1u;"),
    ("remote/extended bits lost in the drain", CAN, "can_drain_tests",
     "    const bool ext = (b[1] & 0x08u) != 0u;", "    const bool ext = false;"),
    ("drain armed in OBD2", CAN, "can_drain_tests",
     "    if (mode == CanMode::DISCOVER || mode == CanMode::SNIFF) {\n        // SNIFF decodes",
     "    if (mode != CanMode::OFF) {\n        // SNIFF decodes"),
    ("host filter write unmasked", CAN, "can_drain_tests",
     "    const CanSpiLock lock;\n    if (ids == nullptr && count != 0u) return false;",
     "    if (ids == nullptr && count != 0u) return false;"),
    ("DISCOVER decodes", CAN, "can_drain_tests",
     "    if (gMode != CanMode::SNIFF) return 0;\n\n    // One reading",
     "    if (gMode != CanMode::SNIFF && gMode != CanMode::DISCOVER) return 0;\n\n    // One reading"),
    ("decoded frames stamped at decode time", CAN, "can_drain_tests",
     "uint32_t nowMs = (ageUs > 0) ? ms0 - (uint32_t)ageUs / 1000u : ms0;",
     "uint32_t nowMs = ms0; (void)ageUs;"),
    ("remote frames decoded", CAN, "can_drain_tests",
     "if (canRawRtr(f) || canRawExt(f)) continue;", "if (canRawExt(f)) continue;"),
    ("stream passes the decoder", CAN, "can_drain_tests",
     "return (gMode == CanMode::SNIFF) ? gDecodeIdx : gRing.head;", "return gRing.head;"),
    ("DISCOVER keeps the old filter bookkeeping", CAN, "can_telemetry_tests",
     "        for (uint8_t i = 0; i < CAN_MAP_FILTER_SLOTS; ++i) gFilterIds[i] = 0u;\n        gFilterCount    = 0u;\n        gFiltersFromMap = false;\n",
     ""),
    ("stream writes into an armed bank", STREAM, "can_stream_tests",
     "        if (!usbCdcTxIdle()) return;", "        (void)usbCdcTxIdle();"),
    ("stream ignores DTR", STREAM, "can_stream_tests", "    if (!Serial.dtr()) {", "    if (false) {"),
    ("stream ignores availableForWrite", STREAM, "can_stream_tests",
     "        int room = Serial.availableForWrite();", "        int room = 64; (void)Serial.availableForWrite();"),
    ("DLC 0 gets a trailing space", STREAM, "can_stream_tests",
     "(rtr ? 3u : (1u + (dlc ? 1u + 2u * dlc : 0u)))", "(rtr ? 3u : (2u + 2u * dlc))"),
    ("extended id in three digits", STREAM, "can_stream_tests",
     "const uint8_t idDigits = ext ? 8u : 3u;", "const uint8_t idDigits = (ext && false) ? 8u : 3u;"),
    ("lowercase hex", STREAM, "can_stream_tests",
     'static const char kHex[] = "0123456789ABCDEF";', 'static const char kHex[] = "0123456789abcdef";'),
    ("FS behind the frames, starved", STREAM, "can_stream_tests",
     "        if (gStatsDue) {\n            // One spare byte", "        if (gStatsDue && !canRawStreamable()) {\n            // One spare byte"),
    ("failed write counted as streamed", STREAM, "can_stream_tests",
     "            gNoHost += lines;\n            return;", "            gStreamed += lines;\n            return;"),
    # ── review 2026-10-05: the FS columns, the SNIFF release bound, the room ────
    ("FS ringdrop column fed 0", STREAM, "can_stream_tests",
     "canSniffOverrunCount(), canRingDropCount(),", "canSniffOverrunCount(), 0u,"),
    ("FS ovf column fed 0", STREAM, "can_stream_tests",
     "canSniffOverrunCount(), canRingDropCount(),", "0u, canRingDropCount(),"),
    ("FS nohost column fed 0", STREAM, "can_stream_tests",
     "                                                 gNoHost);", "                                                 0u);"),
    ("FS drained column fed streamed", STREAM, "can_stream_tests",
     "                                                 canDrainedCount(), gStreamed,",
     "                                                 gStreamed, gStreamed,"),
    ("FS packed past the room", STREAM, "can_stream_tests",
     "            if ((int)len + (int)n <= room) {", "            if ((int)len + (int)n <= 64) {"),
    # ── the bounded wait: the bank kept full, never a wait on a stalled host ──
    ("stream never waits for its own packet", STREAM, "can_stream_tests",
     "        } else if (!waitTxIdle(CAN_STREAM_WAIT_US, deadlineUs)) {",
     "        } else if (((void)deadlineUs, !usbCdcTxIdle())) {"),
    ("stream waits before its first packet", STREAM, "can_stream_tests",
     "        if (k == 0u) {\n            if (!usbCdcTxIdle()) return;",
     "        if (k == 0u) {\n            if (!waitTxIdle(CAN_STREAM_WAIT_US, deadlineUs)) return;"),
    ("per-packet wait unbounded in time", STREAM, "can_stream_tests",
     "        if ((uint32_t)(now - t0) >= maxUs) return false;\n", "        (void)t0; (void)maxUs;\n"),
    ("call budget ignored", STREAM, "can_stream_tests",
     "        if ((int32_t)(now - deadlineUs) >= 0) return false;\n", "        (void)deadlineUs;\n"),
    ("a call that emptied the ring waits anyway", STREAM, "can_stream_tests",
     "        if (!gStatsDue && canRawStreamable() == 0u) return;\n", ""),
    ("console text handed to the core with the bank armed", STREAM, "can_stream_tests",
     "        if (!Serial.dtr() || !waitTxIdle(CAN_CONSOLE_WAIT_US, micros() + CAN_CONSOLE_WAIT_US)) {",
     "        if (!Serial.dtr()) {"),
    ("console text written with nobody listening", STREAM, "can_stream_tests",
     "        if (!Serial.dtr() || !waitTxIdle(CAN_CONSOLE_WAIT_US, micros() + CAN_CONSOLE_WAIT_US)) {",
     "        if (!waitTxIdle(CAN_CONSOLE_WAIT_US, micros() + CAN_CONSOLE_WAIT_US)) {"),
    ("dropped console text owes no break", STREAM, "can_stream_tests",
     "            gBreakOwed = true;\n            break;\n        }\n        const size_t w = Serial.write(buf + done, len);",
     "            break;\n        }\n        const size_t w = Serial.write(buf + done, len);"),
    ("no break owed after the port was closed", STREAM, "can_stream_tests",
     "        gBreakOwed = true;    // console prints meanwhile may leave half a line armed\n", ""),
    ("no break owed after a refused console write", STREAM, "can_stream_tests",
     "            // must not continue it (see gBreakOwed).\n            gBreakOwed = true;\n            break;",
     "            // must not continue it (see gBreakOwed).\n            break;"),
    ("break owed on every packet", STREAM, "can_stream_tests",
     "            if (stats) gStatsDue = false;\n            gBreakOwed = false;", "            if (stats) gStatsDue = false;"),
    ("console packets of 64", STREAM, "can_stream_tests",
     "(n - done > CAN_STREAM_PACKET_MAX) ? CAN_STREAM_PACKET_MAX : (n - done)",
     "(n - done > 64u) ? 64u : (n - done)"),
    # ── the bench self-test generator ───────────────────────────────────────────
    ("self-test rate off by one tick", CAN, "can_selftest_tests",
     "    if (gSelfAcc < CAN_DRAIN_TICKS_PER_S) return;", "    if (gSelfAcc <= CAN_DRAIN_TICKS_PER_S) return;"),
    ("self-test sequence stuck", CAN, "can_selftest_tests",
     "    gSelfSeq = seq + 1u;\n", ""),
    ("self-test runs disarmed", CAN, "can_selftest_tests",
     "    if (!gDrainArmed) return;\n\n#if defined(DASHCAM_CAN_STREAM_SELFTEST)\n    selfTestTick();",
     "#if defined(DASHCAM_CAN_STREAM_SELFTEST)\n    selfTestTick();\n#endif\n    if (!gDrainArmed) return;\n\n#if defined(DASHCAM_CAN_STREAM_SELFTEST)\n"),
    ("self-test frames not drained", CAN, "can_selftest_tests",
     "    gDrained = gDrained + 1u;\n    (void)canRingPush(gRing, f);\n}\n#endif",
     "    (void)canRingPush(gRing, f);\n}\n#endif"),
    ("self-test fill not the ring's", CAN, "can_selftest_tests",
     "    const uint16_t fill = canRingCount(gRing);", "    const uint16_t fill = 0u;"),
    ("self-test check word plain", CAN, "can_selftest_tests",
     "    const uint32_t chk  = ~seq;", "    const uint32_t chk  = seq;"),
]

# mkr_zero.ino builds only against the real core; sketch_policy_tests.py reads it.
SKETCH_MUTATIONS = [
    ("boot into SNIFF",
     "    } else if (!applyCanMode(CanMode::DISCOVER)) {\n        obdReady = false;\n        Serial.println(\"CAN: DISCOVER refused",
     "    } else if (!applyCanMode(CanMode::SNIFF)) {\n        obdReady = false;\n        Serial.println(\"CAN: DISCOVER refused"),
    ("OBD2 give-up parks OFF",
     "                applyCanMode(CanMode::DISCOVER);\n                Serial.print(\"OBD2: no ECU",
     "                applyCanMode(CanMode::OFF);\n                Serial.print(\"OBD2: no ECU"),
    ("OFF retry arms the poller",
     "            obdReady = false;\n            if (up && !applyCanMode(CanMode::DISCOVER)) {",
     "            obdReady = up;\n            if (up && !applyCanMode(CanMode::DISCOVER)) {"),
    ("tickOBD2 unmasked",
     "        canDrainIrqMask();\n        const bool replied = tickOBD2", "        const bool replied = tickOBD2"),
    ("liveness not stamped from the drain", "        lastEcuReplyMs  = millis();\n    }", "    }"),
    ("stream seam inside the status line",
     "        statusOut.print(\"spd=\");", "        canStreamService();\n        statusOut.print(\"spd=\");"),
    ("status line not flushed", "        statusOut.flush();   // the last packet", "        // the last packet"),
    ("console line opening with F", "Serial.println(\"GPS: module started\");", "Serial.println(\"F module started\");"),
]


def run(command, cwd):
    return subprocess.run(command, cwd=cwd, text=True, stdout=subprocess.PIPE,
                          stderr=subprocess.STDOUT, timeout=120)


def main():
    with tempfile.TemporaryDirectory(prefix="mkr-mutations-") as tmp:
        root = pathlib.Path(tmp)
        shutil.copytree(MKR / "lib", root / "lib")
        shutil.copy2(MKR / "mkr_zero.ino", root / "mkr_zero.ino")
        tests = root / "tests" / "host"
        tests.mkdir(parents=True)
        for source in HERE.iterdir():
            if source.suffix in (".cpp", ".h") or source.name in ("Makefile", "sketch_policy_tests.py"):
                shutil.copy2(source, tests / source.name)
        for suite in ("imu_tests", "bno_init_tests", "can_probe_tests",
                      "can_ring_tests", "can_drain_tests", "can_telemetry_tests",
                      "can_stream_tests", "can_selftest_tests"):
            build = run(["make", "-B", suite], tests)
            assert build.returncode == 0, build.stdout
            control = run([str(tests / suite)], tests)
            assert control.returncode == 0, control.stdout
            print("CONTROL:", control.stdout.strip(), flush=True)
        for mutation in MUTATIONS:
            name, filename, suite, old, new = mutation[:5]
            count = mutation[5] if len(mutation) == 6 else 1
            path = root / "lib" / filename
            original = path.read_text()
            assert original.count(old) == count, "stale mutation anchor: " + name
            try:
                path.write_text(original.replace(old, new))
                build = run(["make", "-B", suite], tests)
                assert build.returncode == 0, name + " did not compile:\n" + build.stdout
                result = run([str(tests / suite)], tests)
                assert result.returncode != 0, "SURVIVED: " + name
                assert "failed" in result.stdout, "not an assertion failure: " + result.stdout
                print("CAUGHT:", name, flush=True)
            finally:
                path.write_text(original)
        sketch = root / "mkr_zero.ino"
        policy = ["python3", "sketch_policy_tests.py"]
        control = run(policy, tests)
        assert control.returncode == 0, control.stdout
        print("CONTROL:", control.stdout.strip().splitlines()[-1], flush=True)
        original = sketch.read_text()
        for name, old, new in SKETCH_MUTATIONS:
            assert original.count(old) == 1, "stale sketch mutation anchor: " + name
            try:
                sketch.write_text(original.replace(old, new))
                result = run(policy, tests)
                assert result.returncode != 0, "SURVIVED: " + name
                assert "failed" in result.stdout, "not an assertion failure: " + result.stdout
                print("CAUGHT:", name, flush=True)
            finally:
                sketch.write_text(original)
        print(f"All {len(MUTATIONS) + len(SKETCH_MUTATIONS)} mutations caught; unmodified controls passed.")


if __name__ == "__main__":
    main()
