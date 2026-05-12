# Respect environment CXX for cross-compilation
CXX ?= g++

# ─── flags ────────────────────────────────────────────────────────────────────

CXXFLAGS := -std=c++17 -Wall -Wextra -O2 -g \
            -Ilib/libcan \
            -Ilib/libcamera \
            -Ilib/libconfig \
            -Ilib/libgpio \
            -Ilib/libi2c \
            -Ilib/liblog \
            -Ilib/libmidi \
            -Ilib/librecord \
            -Ilib/libspi \
            -Ilib/libstereocam \
            -Ilib/libuart \
            $(shell pkg-config --cflags \
                gstreamer-1.0 gstreamer-app-1.0 gstreamer-video-1.0 \
                cairo pugixml opencv4 spdlog)

LD_BASE := $(shell pkg-config --libs \
               gstreamer-1.0 gstreamer-app-1.0 gstreamer-video-1.0 \
               cairo pugixml spdlog) \
           -pthread

LD_CV   := $(shell pkg-config --libs opencv4)
LD_VPI  := -lvpi
LD_GPIO  := -lgpiod
LD_ALSA  := -lasound

# ─── VPI guard ────────────────────────────────────────────────────────────────
# libstereocam and the production dashcam binary require VPI 3.x headers
# (libnvvpi-dev on JetPack 6.x).  Skip them when headers are absent so that
# tests and demos build in environments where VPI is not installed.

VPI_HDRS := $(wildcard /usr/include/vpi/Image.h)

# ─── library source groups ────────────────────────────────────────────────────

LIBCAM_CORE_SRCS := lib/libcamera/libcamera.cpp

LIBCAM_SRCS := lib/libcamera/libcamera.cpp \
               lib/libcamera/libcamera_gst.cpp \
               lib/libcamera/libcamera_csi.cpp \
               lib/libcamera/libcamera_usb.cpp

LIBCAN_SRCS    := lib/libcan/libcan.cpp
LIBGPIO_SRCS   := lib/libgpio/libgpio.cpp
LIBI2C_SRCS    := lib/libi2c/libi2c.cpp
LIBMIDI_SRCS   := lib/libmidi/libmidi.cpp
LIBSPI_SRCS    := lib/libspi/libspi.cpp
LIBUART_SRCS   := lib/libuart/libuart.cpp
LIBLOG_SRCS    := lib/liblog/liblog.cpp
LIBCFG_SRCS    := lib/libconfig/libconfig.cpp
LIBREC_SRCS    := lib/librecord/librecord.cpp
LIBSTEREO_SRCS := lib/libstereocam/libstereocam.cpp

# Convenience group: all peripheral interface libs (no GStreamer / OpenCV dependency)
LIBPERIPH_SRCS := $(LIBCAN_SRCS) $(LIBGPIO_SRCS) $(LIBI2C_SRCS) $(LIBMIDI_SRCS) $(LIBSPI_SRCS) $(LIBUART_SRCS)

# ─── build directory (timestamped so parallel invocations don't collide) ──────

TIMESTAMP := $(shell date +%Y%m%d_%H%M%S)
BUILD_DIR := bin/build_$(TIMESTAMP)

# ─── per-target object lists ──────────────────────────────────────────────────

define make_objs
$(addprefix $(BUILD_DIR)/,$(1:.cpp=.o))
endef

# csi_test: raw Argus pipeline tests, no library wrappers
CSI_OBJS := $(call make_objs, src/tests/test_suite_1.cpp)

# usb_test: USB camera lifecycle and frame-rate tests
USB_OBJS := $(call make_objs, $(LIBLOG_SRCS) $(LIBCAM_SRCS) $(LIBCFG_SRCS) src/tests/test_usb_cameras.cpp)

# recording_test: overlay + MKV recording + graceful shutdown
REC_OBJS := $(call make_objs, $(LIBLOG_SRCS) $(LIBCAM_SRCS) $(LIBCFG_SRCS) $(LIBREC_SRCS) \
                src/tests/recording_and_safe_shutdown.cpp)

# scan_cameras: enumerate all V4L2 devices
SCAN_OBJS := $(call make_objs, $(LIBCAM_CORE_SRCS) src/tests/scan_cameras.cpp)

# config_test: XML config round-trip tests
CFG_OBJS := $(call make_objs, $(LIBLOG_SRCS) $(LIBCFG_SRCS) $(LIBCAM_CORE_SRCS) src/tests/test_libconfig.cpp)

# midi_test: WAV playback smoke test
MIDI_OBJS := $(call make_objs, $(LIBLOG_SRCS) $(LIBMIDI_SRCS) src/tests/midi_test.cpp)

# can_test: SocketCAN send/receive loopback test
CAN_OBJS := $(call make_objs, $(LIBLOG_SRCS) $(LIBCAN_SRCS) src/tests/can_test.cpp)

# gpio_test: GPIO / UART / I2C / SPI hardware interface test
GPIO_OBJS := $(call make_objs, $(LIBLOG_SRCS) $(LIBGPIO_SRCS) $(LIBUART_SRCS) \
                 $(LIBI2C_SRCS) $(LIBSPI_SRCS) src/tests/gpio_test.cpp)

# dashcam: production application (1 CSI + 3 USB + stereo + recording) — needs VPI
DASHCAM_OBJS := $(call make_objs, $(LIBLOG_SRCS) $(LIBPERIPH_SRCS) $(LIBCAM_SRCS) \
                    $(LIBCFG_SRCS) $(LIBREC_SRCS) $(LIBSTEREO_SRCS) src/main.cpp)

# demo_graphical: interactive preview window showcasing all four libraries
DEMO_OBJS := $(call make_objs, $(LIBLOG_SRCS) $(LIBCAM_SRCS) $(LIBCFG_SRCS) $(LIBREC_SRCS) \
                 src/tests/demo_graphical.cpp)

# demo_terminal: headless walkthrough of all four libraries; no display required
DEMO_TERM_OBJS := $(call make_objs, $(LIBLOG_SRCS) $(LIBCAM_SRCS) $(LIBCFG_SRCS) $(LIBREC_SRCS) \
                     src/tests/demo_terminal.cpp)

# Always compile these; no VPI dependency.
BASE_OBJS := $(sort $(CSI_OBJS) $(USB_OBJS) $(REC_OBJS) $(SCAN_OBJS) $(CFG_OBJS) \
                    $(CAN_OBJS) $(GPIO_OBJS) $(MIDI_OBJS) $(DEMO_OBJS) $(DEMO_TERM_OBJS))

ifneq ($(VPI_HDRS),)
ALL_OBJS := $(sort $(BASE_OBJS) $(DASHCAM_OBJS))
else
ALL_OBJS := $(BASE_OBJS)
endif

DEPS := $(ALL_OBJS:.o=.d)

# ─── targets ──────────────────────────────────────────────────────────────────

# Targets that never need VPI.
TARGETS := $(BUILD_DIR)/csi_test \
           $(BUILD_DIR)/usb_test \
           $(BUILD_DIR)/recording_test \
           $(BUILD_DIR)/scan_cameras \
           $(BUILD_DIR)/config_test \
           $(BUILD_DIR)/can_test \
           $(BUILD_DIR)/gpio_test \
           $(BUILD_DIR)/midi_test \
           $(BUILD_DIR)/demo_graphical \
           $(BUILD_DIR)/demo_terminal

# dashcam requires libstereocam which requires VPI headers.
ifneq ($(VPI_HDRS),)
TARGETS += $(BUILD_DIR)/dashcam
endif

.PHONY: all clean run

all: $(TARGETS)
ifeq ($(VPI_HDRS),)
	@echo "NOTE: VPI headers not found — dashcam target skipped (install libnvvpi-dev)"
endif
	@echo "Built all targets in $(BUILD_DIR)/"

ifneq ($(VPI_HDRS),)
$(BUILD_DIR)/dashcam: $(DASHCAM_OBJS)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LD_BASE) $(LD_CV) $(LD_VPI) $(LD_GPIO) $(LD_ALSA)
endif

$(BUILD_DIR)/csi_test: $(CSI_OBJS)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LD_BASE)

$(BUILD_DIR)/usb_test: $(USB_OBJS)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LD_BASE)

$(BUILD_DIR)/recording_test: $(REC_OBJS)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LD_BASE)

$(BUILD_DIR)/scan_cameras: $(SCAN_OBJS)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LD_BASE)

$(BUILD_DIR)/config_test: $(CFG_OBJS)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LD_BASE)

$(BUILD_DIR)/demo_graphical: $(DEMO_OBJS)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LD_BASE) $(LD_CV)

$(BUILD_DIR)/can_test: $(CAN_OBJS)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LD_BASE)

$(BUILD_DIR)/gpio_test: $(GPIO_OBJS)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LD_BASE) $(LD_GPIO)

$(BUILD_DIR)/midi_test: $(MIDI_OBJS)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LD_BASE) $(LD_ALSA)

$(BUILD_DIR)/demo_terminal: $(DEMO_TERM_OBJS)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LD_BASE)

# Pattern rule: mirror source tree under BUILD_DIR.
$(BUILD_DIR)/%.o: %.cpp
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) -MMD -MP -c $< -o $@

-include $(DEPS)

# Remove all timestamped build directories.
clean:
	rm -rf bin/

# ─── notes ────────────────────────────────────────────────────────────────────
# To add a stereo standalone test binary, link:
#   $(LIBCAM_SRCS) $(LIBCFG_SRCS) $(LIBSTEREO_SRCS) + LD_BASE + LD_CV + LD_VPI
# No stereo test binary exists yet; add src/tests/test_stereocam.cpp to activate.
