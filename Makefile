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
            -Ilib/libbus \
            -Ilib/libcan \
            -Ilib/libdriverstate \
            -Ilib/libsigndetector \
            -Ilib/libcamera \
            -Ilib/libcommlink \
            -Ilib/libconfig \
            -Ilib/libes3display \
            -Ilib/libgpio \
            -Ilib/libi2c \
            -Ilib/liblanedetector \
            -Ilib/liblog \
            -Ilib/libmidi \
            -Ilib/libnavigation \
            -Ilib/libnetwork \
            -Ilib/librecord \
            -Ilib/libspi \
            -Ilib/libstereocam \
            -Ilib/libtimesync \
            -Ilib/libuart \
            -Ilib/libvideopreview \
            $(shell pkg-config --cflags \
                gstreamer-1.0 gstreamer-app-1.0 gstreamer-video-1.0 \
                pugixml opencv4 spdlog libcurl libpng sqlite3)

LD_BASE := $(shell pkg-config --libs \
               gstreamer-1.0 gstreamer-app-1.0 gstreamer-video-1.0 \
               pugixml spdlog) \
           -pthread

LD_CV   := $(shell pkg-config --libs opencv4)
LD_TRT  := -L/usr/local/cuda/lib -L/usr/local/cuda/lib64 -lnvinfer -lcudart
LD_VPI  := -lvpi
LD_GPIO  := -lgpiod
LD_ALSA  := -lasound
LD_NAV   := $(shell pkg-config --libs libcurl libpng sqlite3)
LD_JPEG  := $(shell pkg-config --libs libjpeg)

# ─── library source groups ────────────────────────────────────────────────────

LIBCAM_CORE_SRCS := lib/libcamera/libcamera.cpp
LIBCAM_EXPO_SRCS := lib/libcamera/libcamera_exposure.cpp
LIBCAM_FOCUS_SRCS := lib/libcamera/libcamera_focus.cpp
LIBCAM_ATTR_SRCS := lib/libcamera/libcamera_attributes.cpp

LIBCAM_SRCS := lib/libcamera/libcamera.cpp \
               lib/libcamera/libcamera_attributes.cpp \
               lib/libcamera/libcamera_gst.cpp \
               lib/libcamera/libcamera_csi.cpp \
               lib/libcamera/libcamera_usb.cpp

LIBCAN_SRCS    := lib/libcan/libcan.cpp
LIBCOMM_SRCS   := lib/libcommlink/libcommlink.cpp
LIBTIME_SRCS   := lib/libtimesync/libtimesync.cpp
LIBGPIO_SRCS   := lib/libgpio/libgpio.cpp lib/libgpio/libgpio_pwm.cpp
LIBI2C_SRCS    := lib/libi2c/libi2c.cpp
LIBMIDI_SRCS   := lib/libmidi/libmidi.cpp
LIBNET_SRCS    := lib/libnetwork/libnetwork.cpp lib/libnetwork/libnetwork_ntp.cpp \
                  lib/libnetwork/libnetwork_stream.cpp lib/libnetwork/libnetwork_rtp.cpp \
                  lib/libnetwork/libnetwork_control.cpp lib/libnetwork/libnetwork_hmac.cpp \
                  lib/libnetwork/libnetwork_wifi.cpp
LIBSPI_SRCS    := lib/libspi/libspi.cpp
LIBUART_SRCS   := lib/libuart/libuart.cpp
LIBLOG_SRCS    := lib/liblog/liblog.cpp
LIBCFG_SRCS      := lib/libconfig/libconfig.cpp
LIBLANE_SRCS     := lib/liblanedetector/liblanedetector.cpp \
                    lib/liblanedetector/lane_geometry.cpp \
                    lib/liblanedetector/ufld_decode.cpp
LIBLANE_CU_SRCS  := lib/liblanedetector/liblanedetector_preprocess.cu
LIBDSTATE_SRCS    := lib/libdriverstate/libdriverstate.cpp
LIBDSTATE_CU_SRCS := lib/libdriverstate/libdriverstate_preprocess.cu
LIBSIGN_SRCS      := lib/libsigndetector/libsigndetector.cpp
LIBSIGN_CU_SRCS   := lib/libsigndetector/libsigndetector_preprocess.cu
LIBREC_SRCS      := lib/librecord/librecord.cpp lib/librecord/libsegment.cpp \
                    lib/librecord/libretention.cpp
LIBSTEREO_SRCS := lib/libstereocam/libstereocam.cpp
LIBPREVIEW_SRCS := lib/libvideopreview/libvideopreview.cpp \
                   lib/libvideopreview/libvideopreview_convert.cpp
LIBES3_SRCS     := lib/libes3display/libes3display.cpp
LIBNAV_SRCS     := lib/libnavigation/libnavigation.cpp lib/libnavigation/nav_geo.cpp \
                   lib/libnavigation/nav_tiles.cpp lib/libnavigation/nav_compose.cpp \
                   lib/libnavigation/nav_pan.cpp lib/libnavigation/nav_fetch.cpp \
                   lib/libnavigation/nav_render.cpp

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

# dashcam_v0_2: v0.2 app — v0.1 + UFLD v2 lane detection on the IMX296 (needs TRT, no VPI)
V02_OBJS := $(call make_objs, $(LIBLOG_SRCS) $(LIBCAM_SRCS) $(LIBCFG_SRCS) $(LIBREC_SRCS) \
                $(LIBLANE_SRCS) src/dashcam_v0_2.cpp) \
            $(call make_cu_objs, $(LIBLANE_CU_SRCS))

# dashcam_v0_3: v0.3 app — v0.2 + driver drowsiness monitoring on a UVC camera
# (libdriverstate: YuNet DNN face crop + TRT classifier; needs TRT + OpenCV)
V03_OBJS := $(call make_objs, $(LIBLOG_SRCS) $(LIBCAM_SRCS) $(LIBCFG_SRCS) $(LIBREC_SRCS) \
                $(LIBLANE_SRCS) $(LIBDSTATE_SRCS) $(LIBNET_SRCS) \
                $(LIBUART_SRCS) $(LIBCOMM_SRCS) src/dashcam_v0_3.cpp) \
            $(call make_cu_objs, $(LIBLANE_CU_SRCS) $(LIBDSTATE_CU_SRCS))

# dashcam_v0_4: recording-only app — one UVC camera, compressed passthrough into
# gapless segments with loop overwrite, plus the optional ES3C28P display preview
# and map (libvideopreview + libes3display + libnavigation: libjpeg, libcurl, libpng, sqlite3);
# no inference (no TRT/OpenCV)
V04_OBJS := $(call make_objs, $(LIBLOG_SRCS) $(LIBCAM_CORE_SRCS) $(LIBCAM_EXPO_SRCS) $(LIBCAM_FOCUS_SRCS) $(LIBCFG_SRCS) \
                $(LIBREC_SRCS) $(LIBNET_SRCS) $(LIBUART_SRCS) $(LIBCOMM_SRCS) $(LIBTIME_SRCS) \
                $(LIBPREVIEW_SRCS) $(LIBES3_SRCS) $(LIBNAV_SRCS) src/dashcam_v0_4.cpp)

# timesync_test: NTP-first / GPS-fallback clock decisions + clock-jump detection (no privileges)
TIMESYNC_TEST_OBJS := $(call make_objs, $(LIBLOG_SRCS) $(LIBTIME_SRCS) src/tests/test_timesync.cpp)

# bridge_overlay_test: ESP32-C3 telemetry -> recording overlay and map position mapping (src/bridge_overlay.h,
# no hardware; nav_geo.cpp for the map's usablePosition rules)
BRIDGE_OVL_OBJS := $(call make_objs, $(LIBLOG_SRCS) lib/libnavigation/nav_geo.cpp src/tests/test_bridge_overlay.cpp)
MONITOR_OBJS := $(call make_objs, src/tests/test_monitoring.cpp)

# videopreview_test: libvideopreview MJPEG -> letterboxed preview JPEG (DCT-scaled decode, resize,
# capped encode) + never-blocking submission, worker and lease pool (synthesized JPEGs, mock sink)
PREVIEW_TEST_OBJS := $(call make_objs, $(LIBLOG_SRCS) $(LIBPREVIEW_SRCS) src/tests/test_libvideopreview.cpp)

# navigation_test: libnavigation (GPS state, Web Mercator, tile PNG/HTTP caching rules, SQLite tile
# cache, map compositor, NavigationService against a fake HTTP tile server on 127.0.0.1) — no network
NAV_TEST_OBJS := $(call make_objs, $(LIBLOG_SRCS) $(LIBPREVIEW_SRCS) $(LIBNAV_SRCS) src/tests/test_libnavigation.cpp)

# es3display_test: libes3display (ES3C28P USB tty owner: handshake, CAPABILITIES/START_VIDEO, credited
# E3 29 video with control priority + TIOCOUTQ gating, reconnect, map + board-side panning) against a
# fake display on a pty; also NavigationService -> link -> fake board end to end (libnavigation)
ES3_TEST_OBJS := $(call make_objs, $(LIBLOG_SRCS) $(LIBUART_SRCS) $(LIBPREVIEW_SRCS) $(LIBES3_SRCS) \
                     $(LIBNAV_SRCS) src/tests/test_libes3display.cpp)

# exposure_test: frame-rate-priority auto-exposure loop (pure logic, no camera)
EXPO_TEST_OBJS := $(call make_objs, $(LIBLOG_SRCS) $(LIBCAM_EXPO_SRCS) src/tests/test_exposure.cpp)

# focus_test: the fixed-focus hold against a simulated UVC camera (its own ioctl(), no camera)
FOCUS_TEST_OBJS := $(call make_objs, $(LIBLOG_SRCS) $(LIBCAM_FOCUS_SRCS) src/tests/test_focus.cpp)

# camera_gst_test: Camera_GST over videotestsrc — lifecycle, capture, branches,
# attributes, start/runtime errors, USB/CSI pipeline strings (no camera needed)
CAMGST_TEST_OBJS := $(call make_objs, $(LIBLOG_SRCS) $(LIBCAM_SRCS) src/tests/test_camera_gst.cpp)

# scan_cameras: enumerate all V4L2 devices
SCAN_OBJS := $(call make_objs, $(LIBCAM_CORE_SRCS) src/tests/scan_cameras.cpp)

# config_test: XML config round-trip tests
CFG_OBJS := $(call make_objs, $(LIBLOG_SRCS) $(LIBCFG_SRCS) $(LIBCAM_ATTR_SRCS) src/tests/test_libconfig.cpp)

# midi_test: WAV playback smoke test
MIDI_OBJS := $(call make_objs, $(LIBLOG_SRCS) $(LIBMIDI_SRCS) src/tests/midi_test.cpp)

# liblog_test: async file + console logger smoke test
LIBLOG_TEST_OBJS := $(call make_objs, $(LIBLOG_SRCS) src/tests/test_liblog.cpp)

# write_default_config: build tool that emits a default-valued dashcam.xml,
# used to seed each build's config/ directory (see the `all` target).
WRITECFG_OBJS := $(call make_objs, $(LIBCFG_SRCS) src/tools/write_default_config.cpp)

# can_test: SocketCAN send/receive loopback test
CAN_OBJS := $(call make_objs, $(LIBLOG_SRCS) $(LIBCAN_SRCS) src/tests/can_test.cpp)

# commlink_test: ESP32-C3 USB bridge link test (discovery, streaming, hot-replug)
COMMLINK_OBJS := $(call make_objs, $(LIBLOG_SRCS) $(LIBUART_SRCS) $(LIBCOMM_SRCS) \
                     src/tests/test_commlink.cpp)

# commlink_sim_test: CommLink + libuart against a simulated ESP32-C3 bridge on a pty
# (handshake, frames, commands, watchdog, hot-plug, discovery) — no hardware needed
COMMLINK_SIM_OBJS := $(call make_objs, $(LIBLOG_SRCS) $(LIBUART_SRCS) $(LIBCOMM_SRCS) \
                         src/tests/test_commlink_sim.cpp)

# network_test: TCP/UDP socket loopback + SNTP internet-time smoke test
NET_TEST_OBJS := $(call make_objs, $(LIBLOG_SRCS) $(LIBNET_SRCS) src/tests/test_libnetwork.cpp)

# csi_rtp_test: CSI→Argus→x264→RTP hardware smoke test (in-process udpsrc receiver)
CSI_RTP_OBJS := $(call make_objs, $(LIBLOG_SRCS) $(LIBCAM_SRCS) $(LIBCFG_SRCS) \
                    $(LIBNET_SRCS) src/tests/test_csi_rtp.cpp)

# gpio_test: GPIO / UART / I2C / SPI hardware interface test
GPIO_OBJS := $(call make_objs, $(LIBLOG_SRCS) $(LIBGPIO_SRCS) $(LIBUART_SRCS) \
                 $(LIBI2C_SRCS) $(LIBSPI_SRCS) src/tests/gpio_test.cpp)

# lane_test: lane detection smoke test (10 s live, --video) and --replay (R1 golden)
LANE_TEST_OBJS := $(call make_objs,    $(LIBLOG_SRCS) $(LIBCAM_SRCS) $(LIBCFG_SRCS) \
                                        $(LIBLANE_SRCS) src/tests/test_lanedetector.cpp \
                                        src/tests/lane_test_unit.cpp src/tests/lane_test_gpu.cpp) \
                  $(call make_cu_objs,  $(LIBLANE_CU_SRCS))

# lane_drive: in-car / bench harness of the lane test suite (one camera per process).
# No libjpeg / OpenCV in it: a libjpeg in the global symbol scope breaks nvjpegenc.
LANE_DRIVE_OBJS := $(call make_objs,   $(LIBLOG_SRCS) $(LIBCAM_SRCS) $(LIBCAM_EXPO_SRCS) $(LIBCAM_FOCUS_SRCS) \
                                        $(LIBCFG_SRCS) $(LIBLANE_SRCS) src/tests/test_lanedetector_drive.cpp) \
                   $(call make_cu_objs, $(LIBLANE_CU_SRCS))

# lane_status: the lane test's status pages + alerts on the CYD / ES3C28P (libes3display)
LANE_STATUS_OBJS := $(call make_objs, $(LIBLOG_SRCS) $(LIBUART_SRCS) $(LIBPREVIEW_SRCS) $(LIBES3_SRCS) \
                                       $(LIBNAV_SRCS) src/tests/lane_status_display.cpp)

# driverstate_test: live-UVC drowsiness classification smoke test (2 fps branch)
DSTATE_TEST_OBJS := $(call make_objs,   $(LIBLOG_SRCS) $(LIBCAM_SRCS) $(LIBCFG_SRCS) \
                                         $(LIBDSTATE_SRCS) src/tests/test_libdriverstate.cpp) \
                    $(call make_cu_objs, $(LIBDSTATE_CU_SRCS))

ALL_OBJS := $(sort $(CSI_OBJS) $(USB_OBJS) $(REC_OBJS) $(V02_OBJS) $(V03_OBJS) $(V04_OBJS) $(EXPO_TEST_OBJS) $(FOCUS_TEST_OBJS) $(CAMGST_TEST_OBJS) $(TIMESYNC_TEST_OBJS) $(BRIDGE_OVL_OBJS) $(SCAN_OBJS) \
                   $(CFG_OBJS) $(CAN_OBJS) $(GPIO_OBJS) $(MIDI_OBJS) $(LIBLOG_TEST_OBJS) \
                   $(WRITECFG_OBJS) $(LANE_TEST_OBJS) $(LANE_DRIVE_OBJS) $(LANE_STATUS_OBJS) $(DSTATE_TEST_OBJS) $(NET_TEST_OBJS) \
                   $(CSI_RTP_OBJS) $(COMMLINK_OBJS) $(COMMLINK_SIM_OBJS) \
                   $(PREVIEW_TEST_OBJS) $(ES3_TEST_OBJS) $(NAV_TEST_OBJS))

DEPS := $(ALL_OBJS:.o=.d)

# ─── targets ──────────────────────────────────────────────────────────────────

TARGETS := $(BUILD_DIR)/csi_test \
           $(BUILD_DIR)/usb_test \
           $(BUILD_DIR)/record_test \
           $(BUILD_DIR)/dashcam_v0_2 \
           $(BUILD_DIR)/dashcam_v0_3 \
           $(BUILD_DIR)/dashcam_v0_4 \
           $(BUILD_DIR)/exposure_test \
           $(BUILD_DIR)/focus_test \
           $(BUILD_DIR)/camera_gst_test \
           $(BUILD_DIR)/timesync_test \
           $(BUILD_DIR)/bridge_overlay_test \
           $(BUILD_DIR)/monitoring_test \
           $(BUILD_DIR)/videopreview_test \
           $(BUILD_DIR)/navigation_test \
           $(BUILD_DIR)/es3display_test \
           $(BUILD_DIR)/scan_cameras \
           $(BUILD_DIR)/config_test \
           $(BUILD_DIR)/can_test \
           $(BUILD_DIR)/commlink_test \
           $(BUILD_DIR)/commlink_sim_test \
           $(BUILD_DIR)/network_test \
           $(BUILD_DIR)/csi_rtp_test \
           $(BUILD_DIR)/gpio_test \
           $(BUILD_DIR)/midi_test \
           $(BUILD_DIR)/liblog_test \
           $(BUILD_DIR)/lane_test \
           $(BUILD_DIR)/lane_drive \
           $(BUILD_DIR)/lane_status \
           $(BUILD_DIR)/driverstate_test

.PHONY: all clean dashcam_v0_3 dashcam_v0_4 commlink_test

# `all` stays the default goal even though convenience aliases precede it.
.DEFAULT_GOAL := all

# No timestamped app build: do not affect the service's newest-build selection.
.PHONY: deadreckoning deadreckoning_test
deadreckoning:
	$(MAKE) -C lib/libdeadreckoning all
deadreckoning_test:
	$(MAKE) -C lib/libdeadreckoning check

# Convenience alias so `make commlink_test` works without the build-dir prefix.
commlink_test: $(BUILD_DIR)/commlink_test | $(BUILD_DIR)/logs
	@echo "Built commlink_test in $(BUILD_DIR)/"

# Same for the library suites the display preview work touches.
.PHONY: record_test config_test
.PHONY: monitoring_test
monitoring_test: $(BUILD_DIR)/monitoring_test | $(BUILD_DIR)/logs
	@echo "Built monitoring_test in $(BUILD_DIR)/"
record_test: $(BUILD_DIR)/record_test | $(BUILD_DIR)/logs
	@echo "Built record_test in $(BUILD_DIR)/"
config_test: $(BUILD_DIR)/config_test | $(BUILD_DIR)/logs
	@echo "Built config_test in $(BUILD_DIR)/"
.PHONY: videopreview_test
videopreview_test: $(BUILD_DIR)/videopreview_test | $(BUILD_DIR)/logs
	@echo "Built videopreview_test in $(BUILD_DIR)/"

.PHONY: navigation_test
navigation_test: $(BUILD_DIR)/navigation_test | $(BUILD_DIR)/logs
	@echo "Built navigation_test in $(BUILD_DIR)/"
.PHONY: es3display_test
es3display_test: $(BUILD_DIR)/es3display_test | $(BUILD_DIR)/logs
	@echo "Built es3display_test in $(BUILD_DIR)/"

# Each build gets its own logs/ and config/ directories.  Binaries resolve
# <exe_dir>/logs and <exe_dir>/config at runtime (dashcam::log::init() /
# dashcam::config::configDir()), so a build's binaries use their own tree.  The
# config/ dir is seeded with a default-valued dashcam.xml (and the attribute
# dictionary); libconfig also recreates the defaults on demand at runtime.
all: $(TARGETS) | $(BUILD_DIR)/logs $(BUILD_DIR)/config
	@echo "Built all targets in $(BUILD_DIR)/ (logs -> $(BUILD_DIR)/logs/, config -> $(BUILD_DIR)/config/)"

# Production launcher target: compile only v0.3 and its libraries, while still
# creating/seeding the per-build runtime directories.  The broad `all` target
# remains available to development/test workflows.
dashcam_v0_3: $(BUILD_DIR)/dashcam_v0_3 | $(BUILD_DIR)/logs $(BUILD_DIR)/config
	@echo "Built dashcam_v0_3 in $(BUILD_DIR)/"

# v0.4 launcher target: the recording-only binary plus its runtime directories.
dashcam_v0_4: $(BUILD_DIR)/dashcam_v0_4 | $(BUILD_DIR)/logs $(BUILD_DIR)/config
	@echo "Built dashcam_v0_4 in $(BUILD_DIR)/"

$(BUILD_DIR)/logs:
	@mkdir -p $@

$(BUILD_DIR)/write_default_config: $(WRITECFG_OBJS)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LD_BASE)

$(BUILD_DIR)/config: $(BUILD_DIR)/write_default_config
	@mkdir -p $@
	@test -f $@/dashcam.xml || $(BUILD_DIR)/write_default_config $@/dashcam.xml
	@cp -n config/camera_attributes.xml $@/ 2>/dev/null || true
	@echo "Seeded $@/ (default dashcam.xml + camera_attributes.xml)"

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

$(BUILD_DIR)/dashcam_v0_4: $(V04_OBJS)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LD_BASE) $(LD_JPEG) $(LD_NAV)

$(BUILD_DIR)/exposure_test: $(EXPO_TEST_OBJS)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LD_BASE)

$(BUILD_DIR)/focus_test: $(FOCUS_TEST_OBJS)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LD_BASE)

$(BUILD_DIR)/camera_gst_test: $(CAMGST_TEST_OBJS)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LD_BASE)

$(BUILD_DIR)/timesync_test: $(TIMESYNC_TEST_OBJS)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LD_BASE)

$(BUILD_DIR)/bridge_overlay_test: $(BRIDGE_OVL_OBJS)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LD_BASE)

$(BUILD_DIR)/videopreview_test: $(PREVIEW_TEST_OBJS)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LD_BASE) $(LD_JPEG)

$(BUILD_DIR)/navigation_test: $(NAV_TEST_OBJS)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LD_BASE) $(LD_JPEG) $(LD_NAV)

$(BUILD_DIR)/es3display_test: $(ES3_TEST_OBJS)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LD_BASE) $(LD_JPEG) $(LD_NAV)

$(BUILD_DIR)/scan_cameras: $(SCAN_OBJS)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LD_BASE)

$(BUILD_DIR)/config_test: $(CFG_OBJS)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LD_BASE)

$(BUILD_DIR)/monitoring_test: $(MONITOR_OBJS)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LD_BASE)

$(BUILD_DIR)/can_test: $(CAN_OBJS)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LD_BASE)

$(BUILD_DIR)/commlink_test: $(COMMLINK_OBJS)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LD_BASE)

$(BUILD_DIR)/commlink_sim_test: $(COMMLINK_SIM_OBJS)
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
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LD_BASE) $(LD_TRT) $(LD_CV)

$(BUILD_DIR)/lane_drive: $(LANE_DRIVE_OBJS)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LD_BASE) $(LD_TRT)

$(BUILD_DIR)/lane_status: $(LANE_STATUS_OBJS)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LD_BASE) $(LD_JPEG) $(LD_NAV)

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
# libstereocam needs VPI 3.x (vpi3-dev) and has no target since the old
# `dashcam` app (src/main.cpp) was removed.  To add a stereo test binary, link:
#   $(LIBCAM_SRCS) $(LIBCFG_SRCS) $(LIBSTEREO_SRCS) + LD_BASE + LD_CV + LD_VPI
# No stereo test binary exists yet; add src/tests/test_stereocam.cpp to activate.
