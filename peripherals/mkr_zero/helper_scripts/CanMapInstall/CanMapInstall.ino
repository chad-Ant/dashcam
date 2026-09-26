/**
 * CanMapInstall - writes a CAN map onto the MKR Zero's SD card over USB.
 *
 * The production firmware loads canmap.<vehicle>.txt from the card root at
 * boot, and on an installed rig the card is not easy to reach. This helper
 * carries the map in its image (install_map.sh generates canmap_text.h from
 * config/canmap.<vehicle>.txt), so updating a map is: run install_map.sh, read
 * the report, re-flash production.
 *
 * What it does, once, at boot:
 *   1. mounts the card through the project's own SD layer (lib/SDFunctions);
 *   2. lists the root, so a second canmap.*.txt that would win the loader's
 *      lexicographic tie-break is visible;
 *   3. copies the current target file to canmap_<vehicle>_prev.bak — a name the
 *      loader's canmap.<vehicle>.txt pattern ignores;
 *   4. writes the new map with sdWriteTextAtomic() (temp file, sync, swap);
 *   5. loads it back with the PRODUCTION parser (canMapLoad) and reports its
 *      status, rows, IDs and checksum — the same checksum telemetry carries.
 * Then it repeats the report every 3 s so a late-attaching console still sees it.
 *
 * Touches only the SD card: no CAN controller setup, no I2C.
 * Build: install_map.sh (production --library flags).
 */

#include <stdarg.h>
#include <SdFat.h>
#include "DataDictionary.h"
#include "SDFunctions.h"
#include "CANMap.h"
#include "canmap_text.h"      // generated: kMapVehicle, kMapText

static char gReport[640];

static void appendf(const char *fmt, ...)
{
    const size_t n = strlen(gReport);
    va_list ap; va_start(ap, fmt);
    vsnprintf(gReport + n, sizeof(gReport) - n, fmt, ap);
    va_end(ap);
}

static void install()
{
    gReport[0] = '\0';
    char target[40], backup[48];
    snprintf(target, sizeof(target), "canmap.%s.txt", kMapVehicle);
    snprintf(backup, sizeof(backup), "canmap_%s_prev.bak", kMapVehicle);

    if (initializeSD() != SDReturnStatus::OK) { appendf("RESULT FAIL: card did not mount\n"); return; }

    appendf("card root:");
    File32 dir, f;
    if (sdOpenRoot(dir)) {
        char name[48];
        while (f.openNext(&dir, O_RDONLY)) {
            if (!f.isDir() && f.getName(name, sizeof(name))) appendf(" %s(%lu)", name, (unsigned long)f.fileSize());
            f.close();
        }
        dir.close();
    }
    appendf("\n");

    // Back up what is there now, line by line (the map format is line-based).
    File32 old;
    if (sdOpenRead(target, old)) {
        static char prev[8192];
        size_t used = 0; char line[SD_MAX_LINE + 2];
        while (sdReadLine(old, line, sizeof(line)) && used + strlen(line) + 2 < sizeof(prev)) {
            used += (size_t)snprintf(prev + used, sizeof(prev) - used, "%s\n", line);
        }
        old.close();
        appendf("backup %s: %s (%u bytes)\n", backup,
                sdWriteTextAtomic(backup, prev) == SDReturnStatus::OK ? "ok" : "FAILED", (unsigned)used);
    } else {
        appendf("no existing %s to back up\n", target);
    }

    const SDReturnStatus w = sdWriteTextAtomic(target, kMapText);
    appendf("write %s (%u bytes): %s\n", target, (unsigned)strlen(kMapText),
            w == SDReturnStatus::OK ? "ok" : "FAILED");

    static CanSignalMap m;
    const CanMapStatus st = canMapLoad(m, target);
    appendf("reload with production parser: %s, rows %u, ids %u, checksum 0x%02X\n",
            canMapStatusName(st), m.rowCount, m.idCount, m.checksum);
    appendf("RESULT %s\n", (w == SDReturnStatus::OK && st == CanMapStatus::OK) ? "OK" : "FAIL");
}

void setup()
{
    Serial.begin(115200);
    const uint32_t t0 = millis();
    while (!Serial && (millis() - t0) < 5000u) {}
    install();
    Serial.print(gReport);
}

void loop()
{
    delay(3000);
    Serial.print(gReport);
}
