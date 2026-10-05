# Orin-side tools for the MKR Zero's raw CAN stream.
#
#   make            build can_decode and can_contract_emit
#   make check      build, then run every test (pty-driven logger scenarios,
#                   converter, decoder, end-to-end contract) — about 2 minutes,
#                   most of it real-time pty playback
#   make perf       the logger's CPU at 2500 lines/s for 60 s
#   make clean
#
# Run it where python3 and a hosted C++11 compiler live: on the Jetson that is
# the l4t-ml-gpio container (see README.md).
#
# can_decode and can_contract_emit are compiled from the UNMODIFIED firmware
# sources in ../../lib, against the host stand-ins in ../../tests/host — the
# same objects, include order and stubs the firmware's own host suites use. That
# directory goes FIRST on the include path so its <Arduino.h>, <SPI.h>, <CAN.h>
# and <SdFat.h> shadow the real ones; nothing is copied from it and nothing in
# it is edited for this build.

CXX      ?= g++
CXXFLAGS ?= -std=c++11 -O2 -g -Wall -Wextra -Werror
FW       := ../..
LIB      := $(FW)/lib
HOST     := $(FW)/tests/host
INC      := -I$(HOST) -I$(LIB)

# What linking lib/CANSniffFunctions.cpp needs on the host: the Arduino stand-in,
# the MCP2515 model behind SPI/CAN, and the model of lib/CANStreamHw.cpp.
CORE := $(HOST)/arduino_stub.cpp $(HOST)/mcp2515_model.cpp $(HOST)/can_stream_hw_stub.cpp \
        $(LIB)/CANSniffFunctions.cpp $(LIB)/CANMap.cpp $(LIB)/VehicleSignals.cpp
HDRS := $(LIB)/CANSniffFunctions.h $(LIB)/CANMap.h $(LIB)/CANFrameRing.h $(LIB)/CANStreamHw.h \
        $(LIB)/VehicleSignals.h $(LIB)/DataDictionary.h $(LIB)/SDFunctions.h \
        $(HOST)/Arduino.h $(HOST)/SPI.h $(HOST)/CAN.h $(HOST)/SdFat.h $(HOST)/mcp2515_model.h \
        $(HOST)/can_stream_hw_stub.h

BINS := can_decode can_contract_emit

.PHONY: all check perf clean

all: $(BINS)

can_decode: can_decode.cpp $(CORE) $(HDRS)
	$(CXX) $(CXXFLAGS) $(INC) can_decode.cpp $(CORE) -o $@

can_contract_emit: can_contract_emit.cpp $(CORE) $(LIB)/CANRawStream.cpp $(LIB)/CANRawStream.h $(HDRS)
	$(CXX) $(CXXFLAGS) $(INC) can_contract_emit.cpp $(CORE) $(LIB)/CANRawStream.cpp -o $@

check: $(BINS)
	PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tests -p 'test_*.py' -v

perf:
	PYTHONDONTWRITEBYTECODE=1 python3 tests/perf_stream_log.py

clean:
	rm -f $(BINS)
	rm -rf __pycache__ tests/__pycache__
