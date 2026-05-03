# Respect environment CXX for cross-compilation
CXX      ?= g++

# GStreamer flags, C++17, debug symbols, and the library include path
CXXFLAGS := -std=c++17 -Wall -Wextra -O2 -g \
            -Ilib/libcamera \
            $(shell pkg-config --cflags gstreamer-1.0 gstreamer-app-1.0 gstreamer-video-1.0 cairo)

# GStreamer libs, Cairo, and POSIX threads
LDFLAGS  := $(shell pkg-config --libs gstreamer-1.0 gstreamer-app-1.0 gstreamer-video-1.0 cairo) -pthread

# Each invocation gets its own timestamped output directory.
# TIMESTAMP is evaluated once at parse time (:=), so all rules share the same value.
TIMESTAMP   := $(shell date +%Y%m%d_%H%M%S)
BUILD_DIR   := bin/build_$(TIMESTAMP)

TARGET_NAME := csi_test
TARGET      := $(BUILD_DIR)/$(TARGET_NAME)

SRCS := src/main.cpp \
        lib/libcamera/libcamera.cpp \
        lib/libcamera/libcamera_gst.cpp \
        lib/libcamera/libcamera_csi.cpp \
        lib/libcamera/libcamera_usb.cpp

# Mirror the source tree under BUILD_DIR (e.g. src/main.cpp → BUILD_DIR/src/main.o)
OBJS := $(addprefix $(BUILD_DIR)/,$(SRCS:.cpp=.o))
DEPS := $(OBJS:.o=.d)

.PHONY: all clean run

all: $(TARGET)
	@echo "Built: $(TARGET)"

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS)

# mkdir -p on the object's directory so nested source paths (lib/libcamera/*)
# are created automatically without a separate directory rule.
$(BUILD_DIR)/%.o: %.cpp
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) -MMD -MP -c $< -o $@

-include $(DEPS)

run: $(TARGET)
	./$(TARGET)

# Remove all timestamped build directories.
clean:
	rm -rf bin/
