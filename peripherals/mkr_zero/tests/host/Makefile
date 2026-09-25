# Host tests for the MKR Zero firmware's pure logic.
#
#   make check      build and run every suite
#   make            build only
#   make clean
#
# Suites:
#   switch_tests      lib/SwitchFunctions.cpp, against a 74HC165 chain model
#   imu_tests         lib/IMUFunctions.cpp lifecycle and fault accounting,
#                     against a faked bring-up and burst transport
#   can_probe_tests   lib/CANSniffFunctions.cpp map probe, against an MCP2515
#                     model that filters frames the way the silicon does
#
# Needs any hosted C++11 compiler. There is none on the Windows box this
# firmware is normally built from — arduino-cli ships only arm-none-eabi — so
# the usual place to run these is the Orin Nano, which already has g++ for the
# dashcam application. Nothing here is Linux-specific; it is plain C++ and
# stdio.
#
# THIS DIRECTORY IS FIRST ON THE INCLUDE PATH ON PURPOSE. The modules under
# test include <Arduino.h>, <Wire.h>, <SPI.h>, <CAN.h> and <SdFat.h> with angle
# brackets, so the stand-ins here shadow the real ones and every module
# compiles unmodified — no #ifdef, no test build of the firmware, no second
# copy of the logic.

CXX      ?= g++
CXXFLAGS ?= -std=c++11 -O1 -g -Wall -Wextra -Werror
LIB      := ../../lib
INC      := -I. -I$(LIB)

STUB_HDRS := Arduino.h Wire.h SPI.h CAN.h SdFat.h mcp2515_model.h

SWITCH_SRCS := arduino_stub.cpp switch_tests.cpp $(LIB)/SwitchFunctions.cpp
IMU_SRCS    := arduino_stub.cpp imu_tests.cpp $(LIB)/IMUFunctions.cpp
CAN_SRCS    := arduino_stub.cpp mcp2515_model.cpp can_probe_tests.cpp \
               $(LIB)/CANSniffFunctions.cpp $(LIB)/CANMap.cpp $(LIB)/VehicleSignals.cpp

BINS := switch_tests imu_tests can_probe_tests

.PHONY: all check clean

all: $(BINS)

switch_tests: $(SWITCH_SRCS) $(STUB_HDRS) $(LIB)/SwitchFunctions.h $(LIB)/DataDictionary.h
	$(CXX) $(CXXFLAGS) $(INC) $(SWITCH_SRCS) -o $@

imu_tests: $(IMU_SRCS) $(STUB_HDRS) $(LIB)/IMUFunctions.h $(LIB)/BNO055Init.h $(LIB)/DataDictionary.h
	$(CXX) $(CXXFLAGS) $(INC) $(IMU_SRCS) -o $@

can_probe_tests: $(CAN_SRCS) $(STUB_HDRS) $(LIB)/CANSniffFunctions.h $(LIB)/CANMap.h $(LIB)/DataDictionary.h
	$(CXX) $(CXXFLAGS) $(INC) $(CAN_SRCS) -o $@

# Every suite runs even after one fails, so a single run reports everything.
# Each binary's exit status is its failure count.
check: $(BINS)
	@rc=0; for t in $(BINS); do ./$$t || rc=1; done; exit $$rc

clean:
	rm -f $(BINS)
