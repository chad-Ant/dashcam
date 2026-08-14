/**
 * CanMapSelfTest - proves the CAN map parser and bit extractor on the bench.
 *
 * No SD card, no CAN hardware, no vehicle. Runs anywhere a MKR Zero has USB.
 *
 * This exists because the generic decoder REPLACES code that is known to work on
 * a real car. The bar is not "the new path runs" but "the new path reproduces
 * the old numbers", and the recorded vectors below are the only way to check
 * that without a vehicle.
 *
 * The wheel-speed vector is the load-bearing one. Frame 02 5E 04 BC 09 78 13 59
 * decodes under the four unaligned 15-bit fields to 303/303/303/309 - four
 * wheels agreeing within 2% on a car going straight. Read as aligned 16-bit
 * pairs the same bytes give 606/1212/2424/4953, which no vehicle produces, and
 * that difference is the entire argument for a Motorola extractor over a
 * byte-pair read.
 */

#include "CANMap.h"

static uint16_t gPass = 0;
static uint16_t gFail = 0;

static void check(const char *what, long got, long want)
{
    const bool ok = (got == want);
    if (ok) ++gPass; else ++gFail;
    Serial.print(ok ? F("  ok   ") : F("  FAIL "));
    Serial.print(what);
    Serial.print(F("  got "));
    Serial.print(got);
    if (!ok) { Serial.print(F("  want ")); Serial.print(want); }
    Serial.println();
}

/// The captured wheel-speed frame, straight off the vehicle.
static const uint8_t kWheelFrame[8] = { 0x02, 0x5E, 0x04, 0xBC, 0x09, 0x78, 0x13, 0x59 };
/// A captured powertrain frame: rpm 0x0380 = 896, pedal 0, brake bits set.
static const uint8_t kPowertrainFrame[8] = { 0x00, 0x00, 0x03, 0x80, 0x01, 0x00, 0x20, 0x1F };
/// A gearbox frame with the selector in Drive (byte 5 = 4).
static const uint8_t kGearFrame[8] = { 0x08, 0x00, 0x80, 0xA1, 0xA1, 0x04, 0x7F, 0x2B };
/// A steer-torque frame: 9-bit field = ((0x01 & 1) << 8) | 0xCC = 460.
static const uint8_t kSteerFrame[8] = { 0x01, 0xCC, 0x27, 0, 0, 0, 0, 0 };

static void testExtractor()
{
    Serial.println(F("\n-- Motorola extractor, against recorded frames --"));
    check("wheel FL  7|15", canExtractMotorola(kWheelFrame, 7, 15), 303);
    check("wheel FR  8|15", canExtractMotorola(kWheelFrame, 8, 15), 303);
    check("wheel RL 25|15", canExtractMotorola(kWheelFrame, 25, 15), 303);
    check("wheel RR 42|15", canExtractMotorola(kWheelFrame, 42, 15), 309);

    // The same four by the hand-written shifts the old decoder used. If these
    // disagree the extractor is wrong, not the vectors.
    check("FL vs old shifts",
          canExtractMotorola(kWheelFrame, 7, 15),
          ((uint16_t)kWheelFrame[0] << 7) | (kWheelFrame[1] >> 1));
    check("RR vs old shifts",
          canExtractMotorola(kWheelFrame, 42, 15),
          ((uint16_t)(kWheelFrame[5] & 0x07) << 12) |
          ((uint16_t)kWheelFrame[6] << 4) | (kWheelFrame[7] >> 4));

    check("rpm      23|16", canExtractMotorola(kPowertrainFrame, 23, 16), 896);
    check("pedal     7|8",  canExtractMotorola(kPowertrainFrame, 7, 8), 0);
    check("brake_sw 32|1",  canExtractMotorola(kPowertrainFrame, 32, 1), 1);
    check("brake_pr 53|1",  canExtractMotorola(kPowertrainFrame, 53, 1), 1);
    check("gear     47|8",  canExtractMotorola(kGearFrame, 47, 8), 4);

    // The 9-bit split field: byte 0 bit 0, then byte 1 bits 7..0.
    check("steer     0|9",  canExtractMotorola(kSteerFrame, 0, 9), 460);
    check("steer vs old shifts",
          canExtractMotorola(kSteerFrame, 0, 9),
          ((uint16_t)(kSteerFrame[0] & 0x01) << 8) | kSteerFrame[1]);

    Serial.println(F("-- fast paths must agree with the general one --"));
    check("aligned rpm",   canExtractAligned(kPowertrainFrame, 23, 16),
                           canExtractMotorola(kPowertrainFrame, 23, 16));
    check("aligned pedal", canExtractAligned(kPowertrainFrame, 7, 8),
                           canExtractMotorola(kPowertrainFrame, 7, 8));
    check("singlebit 53",  canExtractBit(kPowertrainFrame, 53),
                           canExtractMotorola(kPowertrainFrame, 53, 1));

    Serial.println(F("-- sign extension --"));
    check("0x7FFF as s16", canSignExtend(0x7FFFu, 16), 32767);
    check("0xFFFF as s16", canSignExtend(0xFFFFu, 16), -1);
    check("0x1FF  as s9",  canSignExtend(0x1FFu, 9), -1);
    check("0x0FF  as s9",  canSignExtend(0x0FFu, 9), 255);
}

static void testMinDlc()
{
    Serial.println(F("\n-- derived minDlc, against the old per-frame guards --"));
    check("speed 7|16",   canRowMinDlc(7, 16), 2);
    check("rpm  23|16",   canRowMinDlc(23, 16), 4);
    check("pedal 7|8",    canRowMinDlc(7, 8), 1);
    check("brk_sw 32|1",  canRowMinDlc(32, 1), 5);
    check("brk_pr 53|1",  canRowMinDlc(53, 1), 7);
    check("gear 47|8",    canRowMinDlc(47, 8), 6);
    check("steer 0|9",    canRowMinDlc(0, 9), 2);
    check("wheel_fl 7|15",  canRowMinDlc(7, 15), 2);
    check("wheel_fr 8|15",  canRowMinDlc(8, 15), 4);
    check("wheel_rl 25|15", canRowMinDlc(25, 15), 6);
    check("wheel_rr 42|15", canRowMinDlc(42, 15), 8);
}

/** @brief Feeds one line to the parser through a writable buffer. */
static bool feed(CanSignalMap &m, const char *text)
{
    char buf[CAN_MAP_MAX_LINE];
    strncpy(buf, text, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';
    return canMapParseLine(m, buf);
}

static void testParser()
{
    Serial.println(F("\n-- grammar: what must be accepted --"));
    CanSignalMap m;
    canMapInitDefaults(m);
    check("comment",  feed(m, "# a comment") ? 1 : 0, 1);
    check("blank",    feed(m, "   ") ? 1 : 0, 1);
    check("speed row", feed(m, "speed,0x158,7,16,u,0.01") ? 1 : 0, 1);
    check("rpm row",   feed(m, "rpm,0x17C,23,16,u,1") ? 1 : 0, 1);
    check("spaces ok", feed(m, "  pedal , 0x17C , 7 , 8 , u , 1  ") ? 1 : 0, 1);
    check("gearmap",   feed(m, "gearmap,P=1,R=2,D=4,S=0x0A") ? 1 : 0, 1);
    check("yawscale",  feed(m, "yawscale,10.83") ? 1 : 0, 1);
    check("rowCount",  m.rowCount, 3);

    check("finalise",  (long)canMapFinalise(m), (long)CanMapStatus::OK);
    check("loaded",    m.loaded ? 1 : 0, 1);
    check("idCount",   m.idCount, 2);          // 0x158 and 0x17C
    check("scale kept", (long)(m.row[m.slotRow[CAN_SIG_SPEED]].scale * 100.0f + 0.5f), 1);
    check("gear P",    m.gearRaw[(uint8_t)VehGear::PARK], 1);
    check("gear S",    m.gearRaw[(uint8_t)VehGear::SPORT], 10);
    check("checksum set", m.checksum != 0 ? 1 : 0, 1);

    Serial.println(F("\n-- grammar: what must be REJECTED --"));
    CanSignalMap b;
    canMapInitDefaults(b);
    check("unknown name",  feed(b, "torque,0x100,7,8,u,1") ? 1 : 0, 0);
    check("id > 0x7FF",    feed(b, "speed,0x800,7,16,u,1") ? 1 : 0, 0);
    check("start > 63",    feed(b, "speed,0x158,64,8,u,1") ? 1 : 0, 0);
    check("len 0",         feed(b, "speed,0x158,7,0,u,1") ? 1 : 0, 0);
    check("len > 32",      feed(b, "speed,0x158,7,33,u,1") ? 1 : 0, 0);
    check("bad sign",      feed(b, "speed,0x158,7,16,x,1") ? 1 : 0, 0);
    check("bad scale",     feed(b, "speed,0x158,7,16,u,1.2.3") ? 1 : 0, 0);
    check("too few fields",feed(b, "speed,0x158,7,16") ? 1 : 0, 0);
    // Runs past the end of an 8-byte payload: minDlc would be 9.
    check("overruns frame",feed(b, "speed,0x158,60,16,u,1") ? 1 : 0, 0);
    check("nothing stored", b.rowCount, 0);

    Serial.println(F("\n-- a map with no speed source must be refused --"));
    CanSignalMap n;
    canMapInitDefaults(n);
    (void)feed(n, "rpm,0x17C,23,16,u,1");
    check("no-speed refused", (long)canMapFinalise(n), (long)CanMapStatus::NOK_NO_SPEED);

    Serial.println(F("\n-- rows sharing an ID must land in one directory run --"));
    CanSignalMap w;
    canMapInitDefaults(w);
    (void)feed(w, "wheel_fl,0x1D0,7,15,u,0.01");
    (void)feed(w, "wheel_fr,0x1D0,8,15,u,0.01");
    (void)feed(w, "wheel_rl,0x1D0,25,15,u,0.01");
    (void)feed(w, "wheel_rr,0x1D0,42,15,u,0.01");
    check("finalise", (long)canMapFinalise(w), (long)CanMapStatus::OK);
    check("one ID",   w.idCount, 1);
    check("four rows in the run", w.id[0].count, 4);
    check("yaw available", (w.statusFlags & CAN_MAP_F_YAW_OK) ? 1 : 0, 1);
    check("filter exact",  (w.statusFlags & CAN_MAP_F_FILTER_EXACT) ? 1 : 0, 1);
    check("one filter slot used", w.filterCount, 1);

    Serial.println(F("\n-- turn signals: two single bits sharing one ID --"));
    CanSignalMap t;
    canMapInitDefaults(t);
    (void)feed(t, "speed,0x158,7,16,u,0.01");
    check("turn_left row",  feed(t, "turn_left,0x294,5,1,u,1") ? 1 : 0, 1);
    check("turn_right row", feed(t, "turn_right,0x294,6,1,u,1") ? 1 : 0, 1);
    check("finalise", (long)canMapFinalise(t), (long)CanMapStatus::OK);
    check("both single-bit",
          ((t.row[t.slotRow[CAN_SIG_TURN_LEFT]].flags  & CAN_ROW_SINGLEBIT) &&
           (t.row[t.slotRow[CAN_SIG_TURN_RIGHT]].flags & CAN_ROW_SINGLEBIT)) ? 1 : 0, 1);
    check("turn minDlc", t.row[t.slotRow[CAN_SIG_TURN_LEFT]].minDlc, 1);
    // Hazards are their OWN slot, not "both turn bits": measured on the vehicle,
    // switching them on leaves both turn bits clear. The row must parse so the
    // bit can be filled in on the card once found, with no reflash.
    {
        CanSignalMap h;
        canMapInitDefaults(h);
        (void)feed(h, "speed,0x158,7,16,u,0.01");
        check("hazard row accepted", feed(h, "hazard,0x294,4,1,u,1") ? 1 : 0, 1);
        check("finalise", (long)canMapFinalise(h), (long)CanMapStatus::OK);
        check("hazard is single-bit",
              (h.row[h.slotRow[CAN_SIG_HAZARD]].flags & CAN_ROW_SINGLEBIT) ? 1 : 0, 1);
    }
    // The bit assignment itself: byte 0 of a frame with only bit 5 set must read
    // left-on/right-off, and vice versa. This is the census result (bit 5 = left,
    // from the rear-wheel yaw sign) pinned down so a future edit cannot swap it.
    {
        const uint8_t leftOnly[8]  = { 0x20, 0, 0, 0, 0, 0, 0, 0 };
        const uint8_t rightOnly[8] = { 0x40, 0, 0, 0, 0, 0, 0, 0 };
        check("bit5 = left on",   (long)canExtractBit(leftOnly,  5), 1);
        check("bit5 not right",   (long)canExtractBit(rightOnly, 5), 0);
        check("bit6 = right on",  (long)canExtractBit(rightOnly, 6), 1);
        check("bit6 not left",    (long)canExtractBit(leftOnly,  6), 0);
    }

    Serial.println(F("\n-- filename convention: canmap.<vehicle>.txt --"));
    {
        char v[CAN_MAP_VEHICLE_MAX];
        check("canmap.brio.txt",  canMapVehicleFromName("canmap.brio.txt", v, sizeof(v)) ? 1 : 0, 1);
        check("  vehicle=brio",   strcmp(v, "brio") == 0 ? 1 : 0, 1);
        // FAT short names come back upper-case whatever was typed.
        check("CANMAP.BRIO.TXT",  canMapVehicleFromName("CANMAP.BRIO.TXT", v, sizeof(v)) ? 1 : 0, 1);
        check("  vehicle=BRIO",   strcmp(v, "BRIO") == 0 ? 1 : 0, 1);
        check("long vehicle ok",  canMapVehicleFromName("canmap.civic_2016.txt", v, sizeof(v)) ? 1 : 0, 1);
        check("  vehicle kept",   strcmp(v, "civic_2016") == 0 ? 1 : 0, 1);
        // A bare canmap.txt is NOT the convention: one spelling, so there is
        // never a question of which wins when both are present.
        check("canmap.txt rejected",   canMapVehicleFromName("canmap.txt", v, sizeof(v)) ? 1 : 0, 0);
        // "canmap..txt" is exactly prefix+suffix, so the vehicle is empty.
        check("empty vehicle rejected",canMapVehicleFromName("canmap..txt", v, sizeof(v)) ? 1 : 0, 0);
        check("wrong prefix",          canMapVehicleFromName("vehmap.brio.txt", v, sizeof(v)) ? 1 : 0, 0);
        check("wrong suffix",          canMapVehicleFromName("canmap.brio.cfg", v, sizeof(v)) ? 1 : 0, 0);
        check("prefix only",           canMapVehicleFromName("canmap.", v, sizeof(v)) ? 1 : 0, 0);
        check("unrelated file",        canMapVehicleFromName("record.txt", v, sizeof(v)) ? 1 : 0, 0);
    }

    Serial.println(F("\n-- checksum must see scale and signedness, not just bit geometry --"));
    {
        CanSignalMap a, b2;
        canMapInitDefaults(a);
        (void)feed(a, "speed,0x158,7,16,u,0.01");
        (void)canMapFinalise(a);
        // Same ID, same bits, ten times the scale: identical telemetry frames
        // decode to numbers an order of magnitude apart, so the maps must not
        // share an identity on the wire.
        canMapInitDefaults(b2);
        (void)feed(b2, "speed,0x158,7,16,u,0.1");
        (void)canMapFinalise(b2);
        check("scale changes checksum", (a.checksum != b2.checksum) ? 1 : 0, 1);

        CanSignalMap c;
        canMapInitDefaults(c);
        (void)feed(c, "speed,0x158,7,16,s,0.01");
        (void)canMapFinalise(c);
        check("signedness changes checksum", (a.checksum != c.checksum) ? 1 : 0, 1);
    }

    Serial.println(F("\n-- >6 ids: priority decides who is dropped, not ID order --"));
    CanSignalMap p;
    canMapInitDefaults(p);
    (void)feed(p, "speed,0x158,7,16,u,0.01");        // pri 5
    (void)feed(p, "rpm,0x100,23,16,u,1");            // pri 2
    (void)feed(p, "pedal,0x101,7,8,u,1");            // pri 2
    (void)feed(p, "gear,0x102,47,8,u,1");            // pri 1  <- lowest
    (void)feed(p, "steer_torque,0x103,0,9,u,1");     // pri 1  <- lowest
    (void)feed(p, "brake_pressed,0x104,53,1,u,1");   // pri 3
    (void)feed(p, "wheel_rl,0x1D0,25,15,u,0.01");    // pri 4
    (void)feed(p, "turn_left,0x294,5,1,u,1");        // pri 4, HIGHEST id
    check("finalise",    (long)canMapFinalise(p), (long)CanMapStatus::OK);
    check("8 ids",       p.idCount, 8);
    check("ids dropped", (p.statusFlags & CAN_MAP_F_IDS_DROPPED) ? 1 : 0, 1);
    check("6 slots used", p.filterCount, 6);
    // The point of the whole exercise: 0x294 sorts LAST by ID and would have
    // been the first casualty under the old ordering.
    check("turn signal kept",  canMapIdIsFiltered(p, 0x294) ? 1 : 0, 1);
    check("speed kept",        canMapIdIsFiltered(p, 0x158) ? 1 : 0, 1);
    check("wheels kept",       canMapIdIsFiltered(p, 0x1D0) ? 1 : 0, 1);
    check("brake kept",        canMapIdIsFiltered(p, 0x104) ? 1 : 0, 1);
    check("gear dropped",      canMapIdIsFiltered(p, 0x102) ? 1 : 0, 0);
    check("steer dropped",     canMapIdIsFiltered(p, 0x103) ? 1 : 0, 0);
    // Ascending, so the boot log and the filter registers read in one order.
    check("filter ids sorted",
          (p.filterId[0] < p.filterId[1] && p.filterId[1] < p.filterId[2] &&
           p.filterId[2] < p.filterId[3] && p.filterId[3] < p.filterId[4] &&
           p.filterId[4] < p.filterId[5]) ? 1 : 0, 1);
}

void setup()
{
    Serial.begin(115200);
    const uint32_t t0 = millis();
    while (!Serial && (millis() - t0) < 3000) { }

    Serial.println();
    Serial.println(F("================ CanMap self-test ================"));
    Serial.println(F("No SD card, no CAN hardware, no vehicle required."));

    testExtractor();
    testMinDlc();
    testParser();

    Serial.println();
    Serial.print(F("=== "));
    Serial.print(gPass);
    Serial.print(F(" passed, "));
    Serial.print(gFail);
    Serial.println(gFail == 0 ? F(" failed - ALL GOOD ===") : F(" FAILED ==="));
}

void loop() { delay(1000); }
