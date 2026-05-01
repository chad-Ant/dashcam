CXX      := g++
CXXFLAGS := -std=c++17 -Wall -Wextra -O2 \
             $(shell pkg-config --cflags gstreamer-1.0)
LDFLAGS  := $(shell pkg-config --libs gstreamer-1.0) -pthread

TARGET := csi_test
SRCS   := src/main.cpp

.PHONY: all clean run

all: $(TARGET)

$(TARGET): $(SRCS)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS)

run: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(TARGET)
