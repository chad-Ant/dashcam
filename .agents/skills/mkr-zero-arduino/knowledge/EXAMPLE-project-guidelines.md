# EXAMPLE — project guidelines (sample; safe to delete or replace)

This is a **sample** house-style document showing the kind of guidance Claude
will apply on top of the built-in references. It is illustrative only — replace
it with your real conventions, or delete it. Remember the precedence rules in
`README.md`: nothing here overrides `references/mission-critical.md` or hardware
reality.

## Naming & structure

- Pin constants in `UPPER_SNAKE_CASE` with a unit/role suffix:
  `SENSOR_CS_PIN`, `ALARM_INT_PIN`. No bare pin numbers in logic.
- One module per concern: `sensors.*`, `storage.*`, `power.*`. `loop()` calls
  named steps, never inlines hardware pokes.
- Tag constants with their port for traceability where it helps review, e.g.
  `// D7 / PA21 / EXTINT5`.

## Libraries (project defaults)

- RTC + sleep: **RTCZero** + **ArduinoLowPower** (don't hand-roll register sleep
  unless a reviewer signs off).
- Persistence: **FlashStorage** for settings, **SD** for logs (per `storage.md`'s
  split). Settings authoritative copy lives on SD for reprogramming-survival.
- Watchdog: **Adafruit SleepyDog** on every unattended build.

## Logging format (SD)

- CSV, 8.3 filenames (`LOG00001.CSV`), header row written once on file creation.
- Each record: `millis,epoch,metric,value,flags`. Build lines with `snprintf`
  into a fixed `char buf[96]` — **no `String`** (Rule 3).
- Flush/close on a fixed cadence and before any sleep; never leave a file open
  across `deepSleep`.

## Boot & health

- Log `PM->RCAUSE.reg` on every boot as the first record (forensic trail).
- Read battery voltage at boot and before each SD write batch; below the low
  threshold, flush and sleep rather than risk a brownout-corrupted write.
- Status over Serial is fine for bench work, but the field health signal is the
  SD log, not the LED (the LED is dark on battery).

## Review checklist (project-specific, in addition to the ten rules)

- [ ] No unbounded `while(!Serial)` / `while(!SD.begin())`.
- [ ] ADC resolution set explicitly (`analogReadResolution(12)` if 12-bit is
      intended).
- [ ] Every interrupt/wake pin verified against the EXTINT map (no shared line).
- [ ] No `float` in ISRs or the sampling hot path.
- [ ] SD files closed/flushed before sleep; flash written only on change.
- [ ] All inputs confirmed ≤3.3 V at the wiring level (level shifter noted if
      not).
