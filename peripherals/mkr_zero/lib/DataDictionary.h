#ifndef DATA_DICTIONARY
#define DATA_DICTIONARY 1

#define SERVO_XAXIS_PIN                             20U
#define SERVO_YAXIS_PIN                             19U
#define SEGLED_ADDRESS                              112U

#define SERIAL_BAUDRATE                             115200U
#define STATUS_INDICATOR                            LED_BUILTIN

#define WIFI_SSID                                   "TP-Link_2890"
#define WIFI_PASS                                   "52000393"
#define WIFI_SSID_BACKUP_1                          "Pixel_5101"
#define WIFI_PASS_BACKUP_1                          "AndroidPixel6"
#define WIFI_SSID_BACKUP_2                          "401 402 403"
#define WIFI_PASS_BACKUP_2                          "0938690720kien"

#define GPS_DEFAULT_I2C_ADDRESS                      66U
#define GPS_REFRESH_RATE                             4U
#define GPS_BAUDRATE_DEFAULT                         9600U
#define GPS_BAUDRATE_CUSTOM                          115200UL
#define USE_DEFAULT_LOCATION 1
#ifdef USE_HOCHIMINH    //10.81532915851147, 106.6573371901137
#define GPS_DEFAULT_POSITION                         "lat=10.81532915851147;lon=106.6573371901137;alt=10.000000;pacc=50000.000000"
#endif
#ifdef USE_VUNGTAU      //10.367506732991703, 107.08795322622392
#define GPS_DEFAULT_POSITION                         "lat=10.367506732991703;lon=107.08795322622392;alt=7.000000;pacc=50000.000000"
#endif
#ifdef USE_DEFAULT_LOCATION //10.81532915851147, 106.6573371901137, 100km radius
#define GPS_DEFAULT_POSITION                         "lat=10.81532915851147;lon=106.6573371901137;alt=10.000000;pacc=100000.000000"
#endif
#define LOCAL_TIMEZONE                               7

#define CAN_BAUDRATE_DEFAULT                         500000UL
/// MCP2515 crystal frequency — MUST match the physical module (16 MHz on the MKR CAN
/// shield; many bare red/blue MCP2515 boards use 8 MHz — set 8000000UL for those).
#define MCP2515_OSC_FREQ                             16000000UL
//SPI interface pins, avoid using these for other purposes
#define MCP2515_DEFAULT_CS_PIN                       3
#define MCP2515_DEFAULT_INT_PIN                      7

/// Chip-select pin for the external SD card module (SPI).
#define SD_CS_PIN                                    4U

/// Default GPS position floats — match the @c USE_DEFAULT_LOCATION string in @c GPS_DEFAULT_POSITION.
#define GPS_DEFAULT_LAT                              10.81532915851147f
#define GPS_DEFAULT_LON                              106.6573371901137f
#define GPS_DEFAULT_ALT                              10.0f
#define GPS_DEFAULT_PACC                             100000.0f

/// ESP32-C3 telemetry link (MKR Zero @c Serial1 UART).
#define ESP32_UART_BAUD                              115200UL
/// Streaming push period — 100 ms = 10 Hz.
#define COMM_STREAM_INTERVAL_MS                      100UL
/// Max inbound command frames serviced per @c tickCommMaster() call (RX budget).
#define COMM_MAX_CMDS_PER_TICK                       8U

#endif