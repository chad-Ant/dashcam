CXX ?= c++
CXXFLAGS ?= -O2 -g -std=c++17 -Wall -Wextra -Wpedantic -Werror
BUILD ?= /tmp/dashcam-lidar-tests
MKR_LIB := ../../peripherals/mkr_zero/libraries/GRF250/src
.PHONY: test sanitize demo
test:
	mkdir -p $(BUILD)
	$(CXX) $(CXXFLAGS) -I$(MKR_LIB) tests/test_protocol.cpp -o $(BUILD)/protocol_test
	$(BUILD)/protocol_test
	PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tests -v
sanitize:
	$(MAKE) test BUILD=$(BUILD)-sanitize CXXFLAGS='-O1 -g -std=c++17 -Wall -Wextra -Wpedantic -Werror -fsanitize=address,undefined -fno-omit-frame-pointer'
demo:
	PYTHONDONTWRITEBYTECODE=1 python3 -m lidartracking --synthetic
