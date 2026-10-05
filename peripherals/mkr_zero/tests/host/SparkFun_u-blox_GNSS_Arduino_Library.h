#ifndef HOST_SPARKFUN_GNSS_STUB_H
#define HOST_SPARKFUN_GNSS_STUB_H 1

/**
 * @file SparkFun_u-blox_GNSS_Arduino_Library.h
 * @brief Host stand-in for the u-blox driver — a name, nothing more.
 *
 * lib/CommunicationFunctions.cpp includes GPSFunctions.h for the GPSData struct
 * the telemetry builder copies, and GPSFunctions.h names the driver's class in
 * its prototypes. Only buildTelemetry() is under test (can_drain_tests), so the
 * class need only exist; nothing here talks to a receiver.
 */

class SFE_UBLOX_GNSS {};

#endif // HOST_SPARKFUN_GNSS_STUB_H
