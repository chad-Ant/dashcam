CORE = ../../lib/libclouduplink
CXXFLAGS ?= -O2 -std=c++17 -Wall -Wextra -Werror
libdashcam_uplink.so: native_bridge.cpp $(CORE)/libclouduplink.h $(CORE)/libclouduplink.cpp
	$(CXX) $(CXXFLAGS) -shared -fPIC native_bridge.cpp $(CORE)/libclouduplink.cpp -o $@
