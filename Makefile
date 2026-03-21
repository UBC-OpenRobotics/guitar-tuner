# Compiler settings
CXX = g++
# Added -Isrc to support Arduino-style folder structure
CXXFLAGS = -std=c++17 -Wall -Wextra -g3 -Isrc -I.

# Source files (Host Build)
SRCS = test/main.cpp \
       src/audio_capture/audio_capture.cpp \
       src/control/motor_drive.cpp \
       src/control/pid_controller.cpp \
       src/pitch_detection/pitch_algorithm.cpp \
       src/pitch_detection/pitch_detector.cpp

# Object files (automatically generated from SRCS)
OBJS = $(SRCS:.cpp=.o)

# Executable name
TARGET = guitar_tuner.exe

# Arduino / ESP32 Settings
ARDUINO_CLI ?= arduino-cli
BOARD ?= esp32:esp32:esp32
PORT ?= COM3
BUILD_DIR ?= build
SKETCH ?= guitar-tuner.ino

# Default target (what runs when you just type 'make')
all: $(TARGET)

# Link the executable (Host Simulation)
$(TARGET): $(OBJS)
	$(CXX) $(OBJS) -o $(TARGET)

# Generic rule for compiling .cpp to .o
# $< is the source file, $@ is the object file
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Run Host Simulation
run_host: $(TARGET)
	@echo Running $(TARGET)...
	.\$(TARGET)

# --- ESP32 Commands ---

# Install ESP32 Core for Arduino CLI (Run once)
setup_arduino:
	$(ARDUINO_CLI) core update-index
	$(ARDUINO_CLI) core install esp32:esp32

# Compile for ESP32
compile_esp32:
	$(ARDUINO_CLI) compile --fqbn $(BOARD) --output-dir $(BUILD_DIR) .

# Upload to ESP32
upload: compile_esp32
	$(ARDUINO_CLI) upload -p $(PORT) --fqbn $(BOARD) --input-dir $(BUILD_DIR) .

# Monitor Serial Output
monitor:
	$(ARDUINO_CLI) monitor -p $(PORT) --config baudrate=115200

# Combined build, upload, monitor
deploy: upload monitor

# Clean target to remove object files and the executable
clean:
	del /Q /F $(subst /,\,$(OBJS)) $(TARGET)
	if exist $(BUILD_DIR) rmdir /S /Q $(BUILD_DIR)


.PHONY: all run_host clean setup_arduino compile_esp32 upload monitor deploy
