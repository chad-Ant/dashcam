# Respect environment CXX for cross-compilation
CXX      ?= g++

# GStreamer flags, C++17, debug symbols, and the library include path
CXXFLAGS := -std=c++17 -Wall -Wextra -O2 -g \
            -Ilib/libcamera \
            $(shell pkg-config --cflags gstreamer-1.0 gstreamer-app-1.0)

# GStreamer libs and POSIX threads
LDFLAGS  := $(shell pkg-config --libs gstreamer-1.0 gstreamer-app-1.0) -pthread

TARGET   := csi_test

SRCS     := src/main.cpp \
            lib/libcamera/libcamera.cpp \
            lib/libcamera/libcamera_gst.cpp \
            lib/libcamera/libcamera_csi.cpp \
            lib/libcamera/libcamera_usb.cpp

OBJS     := $(SRCS:.cpp=.o)
DEPS     := $(OBJS:.o=.d)

.PHONY: all clean run

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -MMD -MP -c $< -o $@

-include $(DEPS)

run: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(TARGET) $(OBJS) $(DEPS)
