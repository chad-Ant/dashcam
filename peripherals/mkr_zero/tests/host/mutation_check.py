"""Reintroduce IMU regressions in temporary copies; never edit the checkout.

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
    ("retirement drops an open episode silently", IMU, "imu_tests",
     "        } else if (dev.lineProbe || dev.lineCandidate){\n            countRejected(dev);\n        }",
     "        }"),
    ("retirement ignores a released probe", IMU, "imu_tests",
     "        if (dev.lineProbe && !intLineHigh(dev)){\n            recordHighG(dev, dev.lineSinceMs, millis(), false);\n        } else if",
     "        if (false){\n        } else if"),
    ("line sampled before the edge", IMU, "imu_tests",
     "    bool pin = takePinEdge(pinMs);\n    const bool line = intLineHigh(dev);",
     "    const bool line = intLineHigh(dev);\n    bool pin = takePinEdge(pinMs);"),
    ("mid-sample edge not merged", IMU, "imu_tests",
     "if (takePinEdge(lateMs) && !pin){ pin = true; pinMs = lateMs; }", "(void)lateMs;"),
    ("bring-up keeps edge", IMU, "imu_tests", "        gPinEvent = false;\n        interrupts();", "        interrupts();"),
    ("recovery erases hgrej", IMU, "imu_tests", "// highGRejected is boot-cumulative: recovery must not erase the evidence.", "dev.highGRejected = 0u;"),
    ("MAG retires whole sensor", IMU, "imu_tests", CORE,
     "(st & (dev.init.opMode == OPERATION_MODE_AMG ? 0x0Fu : 0x0Du)) == (dev.init.opMode == OPERATION_MODE_AMG ? 0x0Fu : 0x0Du)"),
    ("failed MAG still published", IMU, "imu_tests", "(buf[IMU_OFF_ST_RESULT] & BNO055_ST_MAG_PASSED) == 0u", "false"),
    ("bring-up skips self-test", INIT, "bno_init_tests", "!state.selfTestReadOk ||\n                (state.selfTestResult & BNO055_ST_CORE_PASSED) != BNO055_ST_CORE_PASSED", "false"),
    ("interrupt verification only High-G", INIT, "bno_init_tests", "BNO055_INT_BIT_ACC_HIGH_G, BNO055_INT_MOTION_MASK);", "BNO055_INT_BIT_ACC_HIGH_G, BNO055_INT_BIT_ACC_HIGH_G);", 2),
    ("interrupt enables preserved", INIT, "bno_init_tests", "BNO055_INT_BIT_ACC_HIGH_G, BNO055_INT_MOTION_MASK);", "0xECu, BNO055_INT_MOTION_MASK);", 2),
]


def run(command, cwd):
    return subprocess.run(command, cwd=cwd, text=True, stdout=subprocess.PIPE,
                          stderr=subprocess.STDOUT, timeout=120)


def main():
    with tempfile.TemporaryDirectory(prefix="mkr-mutations-") as tmp:
        root = pathlib.Path(tmp)
        shutil.copytree(MKR / "lib", root / "lib")
        tests = root / "tests" / "host"
        tests.mkdir(parents=True)
        for source in HERE.iterdir():
            if source.suffix in (".cpp", ".h") or source.name == "Makefile":
                shutil.copy2(source, tests / source.name)
        for suite in ("imu_tests", "bno_init_tests"):
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
        print(f"All {len(MUTATIONS)} mutations caught; unmodified controls passed.")


if __name__ == "__main__":
    main()
