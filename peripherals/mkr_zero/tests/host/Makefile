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
#   can_ring_tests    lib/CANFrameRing.h: the ISR -> loop() SPSC ring and its
#                     16-byte frame packing
#   can_drain_tests   lib/CANSniffFunctions.cpp drain ISR, mode arming and SPI
#                     masking, SNIFF decode from the ring
#   can_telemetry_tests  the telemetry the C3 gets in DISCOVER, through the
#                     real builder (lib/CommunicationFunctions.cpp)
#   can_stream_tests  lib/CANRawStream.cpp: the F/FS lines, byte for byte, and
#                     the USB gating that keeps loop() from ever blocking
#   can_selftest_tests / can_selftest_max_tests
#                     the bench self-test generator (DASHCAM_CAN_STREAM_SELFTEST)
#                     at 2400 frames/s and at the 10000 cap: the same sources as
#                     can_stream_tests, built with that one define
#   sketch_policy_tests.py
#                     mkr_zero.ino itself, which no hosted compiler can build:
#                     boot and recovery modes, masks, liveness, stream seams
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

STUB_HDRS := Arduino.h Wire.h SPI.h CAN.h SdFat.h mcp2515_model.h can_stream_hw_stub.h \
             SparkFun_u-blox_GNSS_Arduino_Library.h

SWITCH_SRCS := arduino_stub.cpp switch_tests.cpp $(LIB)/SwitchFunctions.cpp
IMU_SRCS    := arduino_stub.cpp imu_tests.cpp $(LIB)/IMUFunctions.cpp
# Everything that links CANSniffFunctions.cpp needs the MCP2515 model and the
# drain/USB hardware model that stands in for lib/CANStreamHw.cpp.
CAN_CORE    := arduino_stub.cpp mcp2515_model.cpp can_stream_hw_stub.cpp \
               $(LIB)/CANSniffFunctions.cpp $(LIB)/CANMap.cpp $(LIB)/VehicleSignals.cpp
CAN_HDRS    := $(LIB)/CANSniffFunctions.h $(LIB)/CANMap.h $(LIB)/CANFrameRing.h \
               $(LIB)/CANStreamHw.h $(LIB)/DataDictionary.h
CAN_SRCS    := $(CAN_CORE) can_probe_tests.cpp
RING_SRCS   := can_ring_tests.cpp
DRAIN_SRCS  := $(CAN_CORE) can_drain_tests.cpp
TELEM_SRCS  := $(CAN_CORE) can_telemetry_tests.cpp $(LIB)/CommunicationFunctions.cpp \
               $(LIB)/CommProtocol.cpp $(LIB)/SwitchFunctions.cpp
STREAM_SRCS := $(CAN_CORE) can_stream_tests.cpp $(LIB)/CANRawStream.cpp
SELFTEST_SRCS := $(CAN_CORE) can_selftest_tests.cpp $(LIB)/CANRawStream.cpp

BINS := switch_tests imu_tests can_probe_tests bno_init_tests can_ring_tests can_drain_tests \
        can_telemetry_tests can_stream_tests can_selftest_tests can_selftest_max_tests

.PHONY: all check clean check-ports check-sketch mutations

all: $(BINS)

switch_tests: $(SWITCH_SRCS) $(STUB_HDRS) $(LIB)/SwitchFunctions.h $(LIB)/DataDictionary.h
	$(CXX) $(CXXFLAGS) $(INC) $(SWITCH_SRCS) -o $@

imu_tests: $(IMU_SRCS) $(STUB_HDRS) $(LIB)/IMUFunctions.h $(LIB)/BNO055Init.h $(LIB)/BNO055Regs.h $(LIB)/DataDictionary.h
	$(CXX) $(CXXFLAGS) $(INC) $(IMU_SRCS) -o $@

can_probe_tests: $(CAN_SRCS) $(STUB_HDRS) $(CAN_HDRS)
	$(CXX) $(CXXFLAGS) $(INC) $(CAN_SRCS) -o $@

can_ring_tests: $(RING_SRCS) $(LIB)/CANFrameRing.h
	$(CXX) $(CXXFLAGS) $(INC) $(RING_SRCS) -o $@

can_drain_tests: $(DRAIN_SRCS) $(STUB_HDRS) $(CAN_HDRS)
	$(CXX) $(CXXFLAGS) $(INC) $(DRAIN_SRCS) -o $@

can_telemetry_tests: $(TELEM_SRCS) $(STUB_HDRS) $(CAN_HDRS) $(LIB)/CommunicationFunctions.h $(LIB)/CommProtocol.h
	$(CXX) $(CXXFLAGS) $(INC) $(TELEM_SRCS) -o $@

can_stream_tests: $(STREAM_SRCS) $(STUB_HDRS) $(CAN_HDRS) $(LIB)/CANRawStream.h
	$(CXX) $(CXXFLAGS) $(INC) $(STREAM_SRCS) -o $@

# The production sources with ONE define added; the build script's default rate
# and the cap (one synthetic frame per drain tick).
can_selftest_tests: $(SELFTEST_SRCS) $(STUB_HDRS) $(CAN_HDRS) $(LIB)/CANRawStream.h
	$(CXX) $(CXXFLAGS) -DDASHCAM_CAN_STREAM_SELFTEST=2400 $(INC) $(SELFTEST_SRCS) -o $@

can_selftest_max_tests: $(SELFTEST_SRCS) $(STUB_HDRS) $(CAN_HDRS) $(LIB)/CANRawStream.h
	$(CXX) $(CXXFLAGS) -DDASHCAM_CAN_STREAM_SELFTEST=10000 $(INC) $(SELFTEST_SRCS) -o $@

bno_init_tests: arduino_stub.cpp bno_init_tests.cpp $(LIB)/BNO055Init.cpp $(LIB)/BNO055Init.h $(LIB)/BNO055Regs.h $(LIB)/DataDictionary.h $(LIB)/BNO055Transport.h $(LIB)/BNO055Calib.h $(STUB_HDRS)
	$(CXX) $(CXXFLAGS) $(INC) arduino_stub.cpp bno_init_tests.cpp $(LIB)/BNO055Init.cpp -o $@

# Every suite runs even after one fails, so a single run reports everything.
# Each binary's exit status is its failure count. The sketch's policy checks
# need Python 3; where there is none they are reported as skipped, not passed.
check: $(BINS)
	@rc=0; for t in $(BINS); do ./$$t || rc=1; done; \
	if command -v python3 >/dev/null 2>&1; then python3 sketch_policy_tests.py || rc=1; \
	else echo "sketch_policy_tests: SKIPPED (no python3)"; fi; exit $$rc

check-sketch:
	python3 sketch_policy_tests.py

check-ports:
	python3 port_tests.py

mutations:
	python3 mutation_check.py

clean:
	rm -f $(BINS)
