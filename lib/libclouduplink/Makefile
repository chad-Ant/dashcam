CXX ?= c++
AR ?= ar
CXXFLAGS ?= -O2 -g -std=c++17 -Wall -Wextra -Wpedantic -Werror
BUILD ?= build

.PHONY: all check sanitize
all: $(BUILD)/libclouduplink.a

$(BUILD):
	mkdir -p $@

$(BUILD)/libclouduplink.o: libclouduplink.cpp libclouduplink.h | $(BUILD)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -fPIC -c $< -o $@

$(BUILD)/libclouduplink.a: $(BUILD)/libclouduplink.o
	$(AR) rcs $@ $^

$(BUILD)/test_clouduplink: tests.cpp $(BUILD)/libclouduplink.a
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(LDFLAGS) $^ $(LDLIBS) -o $@

check: $(BUILD)/test_clouduplink
	$(BUILD)/test_clouduplink

sanitize:
	$(MAKE) BUILD=$(BUILD)-sanitize CXXFLAGS='-O1 -g -std=c++17 -Wall -Wextra -Wpedantic -Werror -fsanitize=address,undefined -fno-omit-frame-pointer' check
