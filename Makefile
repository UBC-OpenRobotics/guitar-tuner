# Compiler settings
CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -g3 -I.

# Source files
# We list them explicitly to avoid wildcard issues on Windows
SRCS = main.cpp \
       audio_capture/audio_capture.cpp \
       control/motor_drive.cpp \
       control/pid_controller.cpp \
       pitch_detection/pitch_algorithm.cpp \
       pitch_detection/pitch_detector.cpp

# Object files (automatically generated from SRCS)
OBJS = $(SRCS:.cpp=.o)

# Executable name
TARGET = guitar_tuner.exe

# Default target (what runs when you just type 'make')
all: $(TARGET)

# Link the executable
$(TARGET): $(OBJS)
	$(CXX) $(OBJS) -o $(TARGET)

# Generic rule for compiling .cpp to .o
# $< is the source file, $@ is the object file
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Flash target to run the executable after building
flash: $(TARGET)
	@echo Running $(TARGET)...
	.\$(TARGET)

# Clean target to remove object files and the executable
clean:
	del /Q /F $(subst /,\,$(OBJS)) $(TARGET)

.PHONY: all flash clean
