# Host tests for lib/SwitchFunctions.cpp.
#
#   make check      build and run
#   make            build only
#   make clean
#
# Needs any hosted C++11 compiler. There is none on the Windows box this
# firmware is normally built from — arduino-cli ships only arm-none-eabi — so
# the usual place to run these is the Orin Nano, which already has g++ for the
# dashcam application. Nothing here is Linux-specific; it is plain C++ and
# stdio.
#
# THIS DIRECTORY IS FIRST ON THE INCLUDE PATH ON PURPOSE. lib/SwitchFunctions.h
# includes <Arduino.h> with angle brackets, so ./Arduino.h shadows the real one
# and the module under test compiles unmodified — no #ifdef, no test build of
# the firmware, no second copy of the logic.

CXX      ?= g++
CXXFLAGS ?= -std=c++11 -O1 -g -Wall -Wextra -Werror
LIB      := ../../lib
INC      := -I. -I$(LIB)

SRCS := arduino_stub.cpp switch_tests.cpp $(LIB)/SwitchFunctions.cpp
BIN  := switch_tests

.PHONY: all check clean

all: $(BIN)

$(BIN): $(SRCS) Arduino.h $(LIB)/SwitchFunctions.h $(LIB)/DataDictionary.h
	$(CXX) $(CXXFLAGS) $(INC) $(SRCS) -o $@

# Exit status is the failure count, so this fails loudly in a script.
check: $(BIN)
	./$(BIN)

clean:
	rm -f $(BIN)
