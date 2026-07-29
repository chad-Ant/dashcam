/*
 * ESP_Sentinel telemetry bridge — arduino-cli / Arduino IDE entry point.
 *
 * Deliberately empty.  arduino-cli requires a .ino named after the sketch
 * directory, but it also compiles everything under src/ recursively, and that
 * is where the bridge lives:
 *
 *   src/main.cpp          setup() / loop()
 *   lib/commLink/         MKR Zero UART link   (pass with --library)
 *   lib/hostLink/         Jetson USB-C link    (pass with --library)
 *
 * Build with BuildAndUpload.cmd, which sets the board flags this firmware
 * depends on (CDCOnBoot=cdc in particular — hostLink.h refuses to compile
 * without it).  PlatformIO users can ignore this file: platformio.ini builds
 * the same sources and needs no .ino.
 */
