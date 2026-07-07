# SIL build (Linux/macOS/WSL/MinGW make). Compiles the EXACT firmware + harness.
CXX      ?= g++
CXXFLAGS := -std=c++17 -O2 -I shims -include shims/Arduino.h
SRC      := sim_main.cpp -x c++ ../Sysiphus_Landing.ino
DEPS     := shims/Arduino.h shims/Wire.h shims/SPI.h shims/SD.h shims/PWMServo.h shims/MPU6050_tockn.h shims/Adafruit_BMP280.h ../Sysiphus_Landing.ino

sim: $(SRC) $(DEPS)
	$(CXX) $(CXXFLAGS) $(SRC) -o sim -lm

run: sim
	./sim > flight.csv

plot: run
	python plot_flight.py flight.csv

clean:
	rm -f sim sim.exe flight.csv flight.png

.PHONY: run plot clean
