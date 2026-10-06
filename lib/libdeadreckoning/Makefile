CXX ?= g++
AR ?= ar
CXXFLAGS ?= -O2 -g -std=c++17 -Wall -Wextra -Wpedantic -Werror
MKR := ../../peripherals/mkr_zero/lib
BUILD ?= build
CPPFLAGS += -I. -I$(MKR)
OBJS := $(BUILD)/deadreckoning.o $(BUILD)/moving_average.o $(BUILD)/c_api.o

.PHONY: all check sanitize
all: $(BUILD)/libdeadreckoning.a $(BUILD)/libdeadreckoning.so
$(BUILD):
	mkdir -p $@
$(BUILD)/deadreckoning.o: libdeadreckoning.cpp libdeadreckoning.h $(MKR)/SignalProcessingFunctions.h $(MKR)/MathFunctions.h | $(BUILD)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -fPIC -c $< -o $@
$(BUILD)/moving_average.o: $(MKR)/SimpleMovingAverage.cpp $(MKR)/SignalProcessingFunctions.h $(MKR)/MathFunctions.h | $(BUILD)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -fPIC -c $< -o $@
$(BUILD)/c_api.o: c_api.cpp c_api.h libdeadreckoning.h $(MKR)/SignalProcessingFunctions.h $(MKR)/MathFunctions.h | $(BUILD)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -fPIC -c $< -o $@
$(BUILD)/libdeadreckoning.a: $(OBJS)
	$(AR) rcs $@ $^
$(BUILD)/libdeadreckoning.so: $(OBJS)
	$(CXX) $(CXXFLAGS) -shared -Wl,--no-undefined $^ -o $@
$(BUILD)/test_deadreckoning: tests.cpp $(BUILD)/libdeadreckoning.a
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $^ -o $@
check: all $(BUILD)/test_deadreckoning
	$(BUILD)/test_deadreckoning
sanitize:
	$(MAKE) BUILD=build-sanitize CXXFLAGS='-O1 -g -std=c++17 -Wall -Wextra -Wpedantic -Werror -fsanitize=address,undefined -fno-omit-frame-pointer' check
