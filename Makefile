# Respect environment CXX / NVCC for cross-compilation
CXX  ?= g++
NVCC ?= /usr/local/cuda/bin/nvcc
# sm_87 = Ampere — used by all Orin-family Jetson (Nano / NX / AGX Orin).
NVCCFLAGS := -O2 -std=c++17 -arch=sm_87 \
             -I/usr/local/cuda/include \
             -Ilib/libdriverstate \
             -Ilib/liblanedetector \
             -Ilib/libsigndetector \
             $(filter-out -pthread,$(shell pkg-config --cflags gstreamer-1.0 gstreamer-app-1.0))

# ─── flags ────────────────────────────────────────────────────────────────────

CXXFLAGS := -std=c++17 -Wall -Wextra -O2 -g \
            -I/usr/local/cuda/include \
            -Ilib/libcan \
            -Ilib/libdriverstate \
            -Ilib/libsigndetector \
            -Ilib/libcamera \
            -Ilib/libconfig \
            -Ilib/libgpio \
            -Ilib/libi2c \
            -Ilib/liblanedetector \
            -Ilib/liblog \
            -Ilib/libmidi \
            -Ilib/libnetwork \
            -Ilib/librecord \
            -Ilib/libspi \
            -Ilib/libstereocam \
            -Ilib/libuart \
            $(shell pkg-config --cflags \
                gstreamer-1.0 gstreamer-app-1.0 gstreamer-video-1.0 \
                pugixml opencv4 spdlog)

LD_BASE := $(shell pkg-config --libs \
               gstreamer-1.0 gstreamer-app-1.0 gstreamer-video-1.0 \
               pugixml spdlog) \
           -pthread

LD_CV   := $(shell pkg-config --libs opencv4)
LD_TRT  := -L/usr/local/cuda/lib -L/usr/local/cuda/lib64 -lnvinfer -lcudart
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
LIBNET_SRCS    := lib/libnetwork/libnetwork.cpp lib/libnetwork/libnetwork_ntp.cpp \
                  lib/libnetwork/libnetwork_stream.cpp lib/libnetwork/libnetwork_rtp.cpp \
                  lib/libnetwork/libnetwork_control.cpp lib/libnetwork/libnetwork_wifi.cpp
LIBSPI_SRCS    := lib/libspi/libspi.cpp
LIBUART_SRCS   := lib/libuart/libuart.cpp
LIBLOG_SRCS    := lib/liblog/liblog.cpp
LIBCFG_SRCS      := lib/libconfig/libconfig.cpp
LIBLANE_SRCS     := lib/liblanedetector/liblanedetector.cpp
LIBLANE_CU_SRCS  := lib/liblanedetector/liblanedetector_preprocess.cu
LIBDSTATE_SRCS    := lib/libdriverstate/libdriverstate.cpp
LIBDSTATE_CU_SRCS := lib/libdriverstate/libdriverstate_preprocess.cu
LIBSIGN_SRCS      := lib/libsigndetector/libsigndetector.cpp
LIBSIGN_CU_SRCS   := lib/libsigndetector/libsigndetector_preprocess.cu
LIBREC_SRCS      := lib/librecord/librecord.cpp
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

define make_cu_objs
$(addprefix $(BUILD_DIR)/,$(1:.cu=.o))
endef

# csi_test: raw Argus pipeline tests, no library wrappers
CSI_OBJS := $(call make_objs, src/tests/test_suite_1.cpp)

# usb_test: USB camera lifecycle and frame-rate tests
USB_OBJS := $(call make_objs, $(LIBLOG_SRCS) $(LIBCAM_SRCS) $(LIBCFG_SRCS) src/tests/test_usb_cameras.cpp)

# record_test: UVC compressed-passthrough recording + ASS telemetry sidecar
REC_OBJS := $(call make_objs, $(LIBLOG_SRCS) $(LIBCAM_SRCS) $(LIBCFG_SRCS) $(LIBREC_SRCS) \
                $(LIBNET_SRCS) src/tests/test_librecord.cpp)

# RETIRED with the librecord redesign (Cairo overlay + x264 branch recording
# removed): recording_and_safe_shutdown, test_dual_recording,
# test_single_record, dashcam_v0_1, demo_graphical, demo_terminal.  Sources
# remain in-tree for reference but no longer compile against the new API.

# dashcam_v0_2: v0.2 app — v0.1 + UFLD v2 lane detection on the IMX296 (needs TRT, no VPI)
V02_OBJS := $(call make_objs, $(LIBLOG_SRCS) $(LIBCAM_SRCS) $(LIBCFG_SRCS) $(LIBREC_SRCS) \
                $(LIBLANE_SRCS) src/dashcam_v0_2.cpp) \
            $(call make_cu_objs, $(LIBLANE_CU_SRCS))

# dashcam_v0_3: v0.3 app — v0.2 + driver drowsiness monitoring on a UVC camera
# (libdriverstate: YuNet DNN face crop + TRT classifier; needs TRT + OpenCV)
V03_OBJS := $(call make_objs, $(LIBLOG_SRCS) $(LIBCAM_SRCS) $(LIBCFG_SRCS) $(LIBREC_SRCS) \
                $(LIBLANE_SRCS) $(LIBDSTATE_SRCS) $(LIBNET_SRCS) src/dashcam_v0_3.cpp) \
            $(call make_cu_objs, $(LIBLANE_CU_SRCS) $(LIBDSTATE_CU_SRCS))

# scan_cameras: enumerate all V4L2 devices
SCAN_OBJS := $(call make_objs, $(LIBCAM_CORE_SRCS) src/tests/scan_cameras.cpp)

# config_test: XML config round-trip tests
CFG_OBJS := $(call make_objs, $(LIBLOG_SRCS) $(LIBCFG_SRCS) $(LIBCAM_CORE_SRCS) src/tests/test_libconfig.cpp)

# midi_test: WAV playback smoke test
MIDI_OBJS := $(call make_objs, $(LIBLOG_SRCS) $(LIBMIDI_SRCS) src/tests/midi_test.cpp)

# liblog_test: async file + console logger smoke test
LIBLOG_TEST_OBJS := $(call make_objs, $(LIBLOG_SRCS) src/tests/test_liblog.cpp)

# write_default_config: build tool that emits a default-valued dashcam.xml,
# used to seed each build's config/ directory (see the `all` target).
WRITECFG_OBJS := $(call make_objs, $(LIBCFG_SRCS) src/tools/write_default_config.cpp)

# can_test: SocketCAN send/receive loopback test
CAN_OBJS := $(call make_objs, $(LIBLOG_SRCS) $(LIBCAN_SRCS) src/tests/can_test.cpp)

# network_test: TCP/UDP socket loopback + SNTP internet-time smoke test
NET_TEST_OBJS := $(call make_objs, $(LIBLOG_SRCS) $(LIBNET_SRCS) src/tests/test_libnetwork.cpp)

# csi_rtp_test: CSI→Argus→x264→RTP hardware smoke test (in-process udpsrc receiver)
CSI_RTP_OBJS := $(call make_objs, $(LIBLOG_SRCS) $(LIBCAM_SRCS) $(LIBCFG_SRCS) \
                    $(LIBNET_SRCS) src/tests/test_csi_rtp.cpp)

# gpio_test: GPIO / UART / I2C / SPI hardware interface test
GPIO_OBJS := $(call make_objs, $(LIBLOG_SRCS) $(LIBGPIO_SRCS) $(LIBUART_SRCS) \
                 $(LIBI2C_SRCS) $(LIBSPI_SRCS) src/tests/gpio_test.cpp)

# dashcam: production application (1 CSI + 3 USB + stereo + recording) — needs VPI
DASHCAM_OBJS := $(call make_objs, $(LIBLOG_SRCS) $(LIBPERIPH_SRCS) $(LIBCAM_SRCS) \
                    $(LIBCFG_SRCS) $(LIBREC_SRCS) $(LIBSTEREO_SRCS) src/main.cpp)

# lane_test: live-camera lane detection smoke test (10 s run)
LANE_TEST_OBJS := $(call make_objs,    $(LIBLOG_SRCS) $(LIBCAM_SRCS) $(LIBCFG_SRCS) \
                                        $(LIBLANE_SRCS) src/tests/test_lanedetector.cpp) \
                  $(call make_cu_objs,  $(LIBLANE_CU_SRCS))

# driverstate_test: live-UVC drowsiness classification smoke test (2 fps branch)
DSTATE_TEST_OBJS := $(call make_objs,   $(LIBLOG_SRCS) $(LIBCAM_SRCS) $(LIBCFG_SRCS) \
                                         $(LIBDSTATE_SRCS) src/tests/test_libdriverstate.cpp) \
                    $(call make_cu_objs, $(LIBDSTATE_CU_SRCS))

# Always compile these; no VPI dependency.
BASE_OBJS := $(sort $(CSI_OBJS) $(USB_OBJS) $(REC_OBJS) $(V02_OBJS) $(V03_OBJS) $(SCAN_OBJS) \
                    $(CFG_OBJS) $(CAN_OBJS) $(GPIO_OBJS) $(MIDI_OBJS) $(LIBLOG_TEST_OBJS) \
                    $(WRITECFG_OBJS) $(LANE_TEST_OBJS) $(DSTATE_TEST_OBJS) $(NET_TEST_OBJS) \
                    $(CSI_RTP_OBJS))

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
           $(BUILD_DIR)/record_test \
           $(BUILD_DIR)/dashcam_v0_2 \
           $(BUILD_DIR)/dashcam_v0_3 \
           $(BUILD_DIR)/scan_cameras \
           $(BUILD_DIR)/config_test \
           $(BUILD_DIR)/can_test \
           $(BUILD_DIR)/network_test \
           $(BUILD_DIR)/csi_rtp_test \
           $(BUILD_DIR)/gpio_test \
           $(BUILD_DIR)/midi_test \
           $(BUILD_DIR)/liblog_test \
           $(BUILD_DIR)/lane_test \
           $(BUILD_DIR)/driverstate_test

# dashcam requires libstereocam which requires VPI headers.
ifneq ($(VPI_HDRS),)
TARGETS += $(BUILD_DIR)/dashcam
endif

.PHONY: all clean run dashcam_v0_3

# Each build gets its own logs/ and config/ directories.  Binaries resolve
# <exe_dir>/logs and <exe_dir>/config at runtime (dashcam::log::init() /
# dashcam::config::configDir()), so a build's binaries use their own tree.  The
# config/ dir is seeded with a default-valued dashcam.xml (and the attribute
# dictionary); libconfig also recreates the defaults on demand at runtime.
all: $(TARGETS) | $(BUILD_DIR)/logs $(BUILD_DIR)/config
ifeq ($(VPI_HDRS),)
	@echo "NOTE: VPI headers not found — dashcam target skipped (install libnvvpi-dev)"
endif
	@echo "Built all targets in $(BUILD_DIR)/ (logs -> $(BUILD_DIR)/logs/, config -> $(BUILD_DIR)/config/)"

# Production launcher target: compile only v0.3 and its libraries, while still
# creating/seeding the per-build runtime directories.  The broad `all` target
# remains available to development/test workflows.
dashcam_v0_3: $(BUILD_DIR)/dashcam_v0_3 | $(BUILD_DIR)/logs $(BUILD_DIR)/config
	@echo "Built dashcam_v0_3 in $(BUILD_DIR)/"

$(BUILD_DIR)/logs:
	@mkdir -p $@

$(BUILD_DIR)/write_default_config: $(WRITECFG_OBJS)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LD_BASE)

$(BUILD_DIR)/config: $(BUILD_DIR)/write_default_config
	@mkdir -p $@
	@test -f $@/dashcam.xml || $(BUILD_DIR)/write_default_config $@/dashcam.xml
	@cp -n config/camera_attributes.xml $@/ 2>/dev/null || true
	@echo "Seeded $@/ (default dashcam.xml + camera_attributes.xml)"

ifneq ($(VPI_HDRS),)
$(BUILD_DIR)/dashcam: $(DASHCAM_OBJS)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LD_BASE) $(LD_CV) $(LD_VPI) $(LD_GPIO) $(LD_ALSA)
endif

$(BUILD_DIR)/csi_test: $(CSI_OBJS)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LD_BASE)

$(BUILD_DIR)/usb_test: $(USB_OBJS)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LD_BASE)

$(BUILD_DIR)/record_test: $(REC_OBJS)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LD_BASE)

$(BUILD_DIR)/dashcam_v0_2: $(V02_OBJS)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LD_BASE) $(LD_TRT)

$(BUILD_DIR)/dashcam_v0_3: $(V03_OBJS)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LD_BASE) $(LD_TRT) $(LD_CV)

$(BUILD_DIR)/scan_cameras: $(SCAN_OBJS)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LD_BASE)

$(BUILD_DIR)/config_test: $(CFG_OBJS)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LD_BASE)

$(BUILD_DIR)/can_test: $(CAN_OBJS)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LD_BASE)

$(BUILD_DIR)/network_test: $(NET_TEST_OBJS)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LD_BASE)

$(BUILD_DIR)/csi_rtp_test: $(CSI_RTP_OBJS)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LD_BASE)

$(BUILD_DIR)/gpio_test: $(GPIO_OBJS)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LD_BASE) $(LD_GPIO)

$(BUILD_DIR)/midi_test: $(MIDI_OBJS)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LD_BASE) $(LD_ALSA)

$(BUILD_DIR)/liblog_test: $(LIBLOG_TEST_OBJS)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LD_BASE)

$(BUILD_DIR)/lane_test: $(LANE_TEST_OBJS)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LD_BASE) $(LD_TRT)

$(BUILD_DIR)/driverstate_test: $(DSTATE_TEST_OBJS)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LD_BASE) $(LD_TRT) $(LD_CV)

# Pattern rules: mirror source tree under BUILD_DIR.
$(BUILD_DIR)/%.o: %.cpp
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) -MMD -MP -c $< -o $@

$(BUILD_DIR)/%.o: %.cu
	@mkdir -p $(@D)
	$(NVCC) $(NVCCFLAGS) -MMD -MP -c $< -o $@

-include $(DEPS)

# Remove all timestamped build directories.
clean:
	rm -rf bin/

# ─── notes ────────────────────────────────────────────────────────────────────
# To add a stereo standalone test binary, link:
#   $(LIBCAM_SRCS) $(LIBCFG_SRCS) $(LIBSTEREO_SRCS) + LD_BASE + LD_CV + LD_VPI
# No stereo test binary exists yet; add src/tests/test_stereocam.cpp to activate.
