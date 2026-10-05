"""Static policy checks on mkr_zero.ino, the one firmware file no hosted compiler builds.

The C++ suites compile every lib/ module unmodified against stand-ins; the
sketch itself needs the real core (USB, SERCOM, the BNO055 and GNSS libraries),
so nothing there runs on the host. The rules below are the ones the raw-CAN
stream rests on and that a one-line edit to the sketch could silently break:

  * the node boots into DISCOVER and recovers into DISCOVER; nothing but the
    host's CMD_SET_CAN_MODE puts it in SNIFF or OBD2 (bus-active);
  * the drain timer is running before the first CAN bring-up;
  * the CAN library's own SPI traffic (initializeOBD2, tickOBD2) is masked from
    the drain ISR, every call;
  * vehicle liveness is stamped from canDrainedCount(), so DISCOVER (which
    decodes nothing) still knows a car is talking;
  * canStreamService() runs at least four times a pass, only between whole
    console lines, and never inside the status line, which goes out through
    statusOut in whole packets and ends with a flush;
  * no console string can start a line with "F " or "FS " — the Orin's prefix.

It reads source text, comments stripped, so it checks what the code says, not
what it does: a guard against regressions, not a proof. Run by `make check`
(and `make check-sketch`). Exit status is the number of failures.
"""
import pathlib
import re
import sys

HERE = pathlib.Path(__file__).resolve().parent
SKETCH = HERE.parents[1] / "mkr_zero.ino"

checks = 0
fails = 0


def check(ok, what):
    global checks, fails
    checks += 1
    if not ok:
        fails += 1
        print("  FAIL  " + what)


def strip_comments(src):
    """Blanks // and /* */ comments, keeping string literals and line numbers."""
    out = []
    i, n = 0, len(src)
    while i < n:
        c = src[i]
        if c == '"' or c == "'":
            q = c
            j = i + 1
            while j < n and src[j] != q:
                j += 2 if src[j] == "\\" else 1
            out.append(src[i:j + 1])
            i = j + 1
        elif src.startswith("//", i):
            j = src.find("\n", i)
            j = n if j < 0 else j
            i = j
        elif src.startswith("/*", i):
            j = src.find("*/", i + 2)
            j = n if j < 0 else j + 2
            out.append("\n" * src.count("\n", i, j))
            i = j
        else:
            out.append(c)
            i += 1
    return "".join(out)


def function_body(code, signature):
    """Text of the brace-balanced body that follows @p signature."""
    start = code.find(signature)
    if start < 0:
        return ""
    brace = code.find("{", start)
    depth = 0
    for k in range(brace, len(code)):
        if code[k] == "{":
            depth += 1
        elif code[k] == "}":
            depth -= 1
            if depth == 0:
                return code[brace:k + 1]
    return ""


def call_args(code, name):
    """Argument text of every CALL of @p name (balanced parentheses), skipping its definition."""
    out = []
    for m in re.finditer(r"\b" + re.escape(name) + r"\(", code):
        if re.search(r"\bbool\s+$", code[max(0, m.start() - 20):m.start()]):
            continue   # "static bool applyCanMode(CanMode want)" - the definition
        depth, k = 1, m.end()
        while k < len(code) and depth:
            depth += {"(": 1, ")": -1}.get(code[k], 0)
            k += 1
        out.append(code[m.end():k - 1].strip())
    return out


def main():
    code = strip_comments(SKETCH.read_text(encoding="utf-8"))
    setup = function_body(code, "void setup()")
    loop = function_body(code, "void loop()")
    check(bool(setup) and bool(loop), "setup() and loop() found")

    # ── modes ────────────────────────────────────────────────────────────────
    calls = call_args(code, "applyCanMode")
    allowed = {"CanMode::DISCOVER", "static_cast<CanMode>(commMaster.canModeRequest)"}
    for arg in calls:
        check(arg.strip() in allowed, "applyCanMode(" + arg.strip() + ") - only DISCOVER, or the host's request")
    check(call_args(code, "canSetMode") == ["want, MCP2515_DEFAULT_CS_PIN"],
          "canSetMode() called only through applyCanMode()")
    host = [a for a in calls if "canModeRequest" in a]
    check(len(host) == 1, "exactly one host-commanded transition")
    host_block = re.search(r"if \(commMaster\.canModeRequest != 0\) \{(.*?)\n    \}", loop, re.S)
    check(host_block is not None and "static_cast<CanMode>(commMaster.canModeRequest)" in host_block.group(1),
          "the host-commanded transition sits inside the canModeRequest block")
    boot = call_args(setup, "applyCanMode")
    check(boot == ["CanMode::DISCOVER"], "setup() enters DISCOVER and nothing else (found %r)" % boot)
    check("canProbeArm(" not in code and "canProbeTick(" not in code, "no boot probe, no automatic OBD2")
    give_up = re.search(r"if \(obdDeadSessions >= OBD2_DEAD_SESSIONS\) \{(.*?)\}", loop, re.S)
    check(give_up is not None and "applyCanMode(CanMode::DISCOVER)" in give_up.group(1),
          "the OBD2 give-up returns to DISCOVER")
    off_retry = re.search(r"const bool wasOff = \(canMode == CanMode::OFF\);(.*?)\n    \}", loop, re.S)
    off_branch = re.search(r"if \(!wasOff\) \{.*?\} else \{(.*?)\n        \}", off_retry.group(1), re.S) \
        if off_retry is not None else None
    check(off_branch is not None and "applyCanMode(CanMode::DISCOVER)" in off_branch.group(1)
          and re.findall(r"obdReady\s*=\s*([^;]+);", off_branch.group(1)) == ["false"],
          "the OFF retry recovers into DISCOVER and leaves the poller off (obdReady = false only)")

    # ── the drain timer and the masks ────────────────────────────────────────
    t = setup.find("canDrainTimerBegin();")
    check(t >= 0 and t < setup.find("startOBD2(") and t < setup.find("applyCanMode("),
          "canDrainTimerBegin() runs before the first CAN bring-up")
    check(code.count("canDrainIrqMask();") == code.count("canDrainIrqUnmask();"),
          "every canDrainIrqMask() has its unmask")
    for fn in ("initializeOBD2(", "tickOBD2("):
        for m in re.finditer(re.escape(fn), code):
            before = code[max(0, m.start() - 160):m.start()]
            after = code[m.end():m.end() + 260]
            check("canDrainIrqMask();" in before and "canDrainIrqUnmask();" in after
                  and "return" not in after[:after.find("canDrainIrqUnmask();")],
                  fn + "...) is masked from the drain ISR, with no return before the unmask")

    # ── liveness from the drain ──────────────────────────────────────────────
    live = re.search(r"if \(drainedNow != lastDrainedSeen\) \{(.*?)\}", loop, re.S)
    check("const uint32_t drainedNow = canDrainedCount();" in loop and live is not None
          and "vehBusEverLive  = true;" in live.group(1) and "lastEcuReplyMs  = millis();" in live.group(1)
          and "lastDrainedSeen = drainedNow;" in live.group(1),
          "vehicle liveness is stamped whenever canDrainedCount() moves")

    # ── stream seams and the status line ─────────────────────────────────────
    status_at = loop.find("if (isTimeout(DEBUG_PRINT_MS, lastDebugPrint)) {")
    check(status_at > 0, "status line found")
    status = function_body(loop[status_at:], "if (isTimeout(DEBUG_PRINT_MS, lastDebugPrint))")
    check(loop.count("canStreamService();") >= 4, "canStreamService() at four or more seams of loop()")
    check("canStreamService();" not in status, "no stream service inside the status line")
    check(re.search(r"\bSerial\.print", status) is None, "the status line prints only through statusOut")
    check(re.search(r"statusOut\.println\(\);\s*statusOut\.flush\(\);", status) is not None,
          "the status line ends with a println and a flush")
    pending = None
    for lineno, line in enumerate(loop.split("\n")):
        if re.search(r"\bSerial\.print\(", line):
            pending = lineno
        if re.search(r"\bSerial\.println\(", line):
            pending = None
        if "canStreamService();" in line:
            check(pending is None, "seam at loop() line %d follows a whole console line" % lineno)

    # ── nothing on the console may look like the stream ──────────────────────
    for m in re.finditer(r"(?:Serial|statusOut)\.print(?:ln)?\(\s*(?:F\()?\"((?:[^\"\\]|\\.)*)\"", code):
        text = m.group(1)
        check(not text.startswith("F ") and not text.startswith("FS ") and "\\nF " not in text,
              "console literal %r cannot open an F/FS line" % text[:40])

    print("sketch_policy_tests: %d checks, %d failed" % (checks, fails))
    return fails


if __name__ == "__main__":
    sys.exit(main())
