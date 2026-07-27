#ifndef GLOBAL_VARIABLES
#define GLOBAL_VARIABLES 1

/// @c true after @c Wire.begin() has been called at least once.
extern bool i2cInitialized;

/// @c true after @c SPI.begin() has been called at least once.
extern bool spiInitialized;

/// @c true after the RTCZero has been successfully set.
extern bool RTCset;

#endif
