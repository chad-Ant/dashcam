# EXAMPLE — project guideline template

This is a sample document showing the kind of guidance you can put in the
`knowledge/` folder. **Delete or replace it** with your own. It exists only to
illustrate the format; nothing here is binding.

Anything in this folder is applied *under* the precedence rules in
`README.md` — the NASA-derived safety standard and hardware reality always win
over guidance like the below.

## Example: house conventions (illustrative)

- **Naming:** `snake_case` for variables, `UPPER_SNAKE` for `constexpr`
  constants, `PascalCase` for types.
- **Logging:** prefix every log line with the subsystem in brackets, e.g.
  `Serial.printf("[wifi] connect timeout\n");`.
- **Config block:** put all user-tunable constants (`SSID`, pins, intervals) in
  a clearly marked block at the top of the sketch.
- **Preferred libraries:** use `ArduinoJson` for any JSON; prefer NimBLE over
  the Bluedroid BLE stack on memory-tight builds.
- **Pin map:** for this project's board, the status LED is on GPIO10 and the
  I2C bus is SDA=GPIO6 / SCL=GPIO7.

## Example of a guideline that would CONFLICT (and lose)

If this document said *"retry WiFi forever until connected,"* that would
contradict Rule 2 (bounded loops) in `../references/mission-critical.md`. Per the
precedence rule, Claude would instead use a bounded wait with a defined fallback
and tell you why. Listed here only to make the precedence behavior concrete.
