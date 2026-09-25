# Host tests for the ESP32-C3 bridge's session handling.
#
#   make check      build and run
#   make            build only
#   make clean
#
# bridge_tests runs src/main.cpp's loop() against lib/commLink over a
# scripted MKR Zero (real frames through the real decoder) and a scripted
# Jetson (hostLink.h here). Plain C++11 and stdio; runs on the Orin Nano's g++.
#
# THIS DIRECTORY IS FIRST ON THE INCLUDE PATH ON PURPOSE. It shadows the ESP32
# core's <Arduino.h>, <esp_log.h> and <esp_system.h>, and lib/hostLink's
# hostLink.h, so the modules under test compile unmodified. lib/hostLink stays
# on the path, after this directory, for the real HostProtocol.h.

CXX      ?= g++
CXXFLAGS ?= -std=c++11 -O1 -g -Wall -Wextra -Werror
C3       := ../..
INC      := -I. -I$(C3)/lib/commLink -I$(C3)/lib/hostLink

SRCS := arduino_stub.cpp bridge_tests.cpp \
        $(C3)/lib/commLink/commLink.cpp $(C3)/lib/commLink/CommProtocol.cpp
HDRS := Arduino.h esp_log.h esp_system.h hostLink.h \
        $(C3)/src/main.cpp $(C3)/lib/commLink/commLink.h $(C3)/lib/commLink/CommProtocol.h \
        $(C3)/lib/hostLink/HostProtocol.h
BIN  := bridge_tests

.PHONY: all check clean

all: $(BIN)

$(BIN): $(SRCS) $(HDRS)
	$(CXX) $(CXXFLAGS) $(INC) $(SRCS) -o $@

# Exit status is the failure count, so this fails loudly in a script.
check: $(BIN)
	./$(BIN)

clean:
	rm -f $(BIN)
