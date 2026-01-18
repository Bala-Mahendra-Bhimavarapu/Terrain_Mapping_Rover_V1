# PROS Makefile for Lunar Rover V5 Project

# Set this to the name of your main project file (without . cpp extension)
PROJNAME = lunar_rover_v5

# List all source files here
SRC = src/main.cpp \
      src/serial_protocol.cpp \
      src/motor_controller.cpp \
      src/odometry.cpp

# Include directories
INCLUDE = -Iinclude

# Compiler flags
CXXFLAGS = -std=gnu++17 -O2 -Wall -Wextra

# Default target
all: 
	pros make

# Build the project
build:
	pros build

# Upload to V5 brain
upload:
	pros upload

# Clean build artifacts
clean: 
	pros make clean

# Terminal for debugging
terminal:
	pros terminal

. PHONY: all build upload clean terminal
