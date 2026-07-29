#ifndef GLOBAL_VARIABLES
#define GLOBAL_VARIABLES 1

// i2cInitialized was removed: I2CBus.h owns the bus lifecycle now, and a bare
// bool could not represent the state that matters (Stuck), nor enforce that
// recovery happens before the first transaction.  Use i2cBusBegin().

/// @c true after @c SPI.begin() has been called at least once.
extern bool spiInitialized;

/// @c true after the RTCZero has been successfully set.
extern bool RTCset;

#endif
