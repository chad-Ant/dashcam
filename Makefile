# Respect environment CXX for cross-compilation
CXX ?= g++

# ─── flags ────────────────────────────────────────────────────────────────────

CXXFLAGS := -std=c++17 -Wall -Wextra -O2 -g \
            -Ilib/libcamera \
            -Ilib/libconfig \
            -Ilib/liblog \
            -Ilib/librecord \
            -Ilib/libstereocam \
            $(shell pkg-config --cflags \
                gstreamer-1.0 gstreamer-app-1.0 gstreamer-video-1.0 \
                cairo pugixml opencv4 spdlog)

LD_BASE := $(shell pkg-config --libs \
               gstreamer-1.0 gstreamer-app-1.0 gstreamer-video-1.0 \
               cairo pugixml spdlog) \
           -pthread

LD_CV  := $(shell pkg-config --libs opencv4)
LD_VPI := -lvpi

# ─── library source groups ────────────────────────────────────────────────────

LIBCAM_CORE_SRCS := lib/libcamera/libcamera.cpp

LIBCAM_SRCS := lib/libcamera/libcamera.cpp \
               lib/libcamera/libcamera_gst.cpp \
               lib/libcamera/libcamera_csi.cpp \
               lib/libcamera/libcamera_usb.cpp

LIBLOG_SRCS    := lib/liblog/liblog.cpp
LIBCFG_SRCS    := lib/libconfig/libconfig.cpp
LIBREC_SRCS    := lib/librecord/librecord.cpp
LIBSTEREO_SRCS := lib/libstereocam/libstereocam.cpp

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

# dashcam: production application (1 CSI + 3 USB + stereo + recording)
DASHCAM_OBJS := $(call make_objs, $(LIBLOG_SRCS) $(LIBCAM_SRCS) $(LIBCFG_SRCS) $(LIBREC_SRCS) \
                    $(LIBSTEREO_SRCS) src/main.cpp)

ALL_OBJS := $(sort $(CSI_OBJS) $(USB_OBJS) $(REC_OBJS) $(SCAN_OBJS) $(CFG_OBJS) $(DASHCAM_OBJS))
DEPS     := $(ALL_OBJS:.o=.d)

# ─── targets ──────────────────────────────────────────────────────────────────

TARGETS := $(BUILD_DIR)/dashcam \
           $(BUILD_DIR)/csi_test \
           $(BUILD_DIR)/usb_test \
           $(BUILD_DIR)/recording_test \
           $(BUILD_DIR)/scan_cameras \
           $(BUILD_DIR)/config_test

.PHONY: all clean run

all: $(TARGETS)
	@echo "Built all targets in $(BUILD_DIR)/"

$(BUILD_DIR)/dashcam: $(DASHCAM_OBJS)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LD_BASE) $(LD_CV) $(LD_VPI)

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
