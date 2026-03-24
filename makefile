# Nazwa pliku wynikowego
TARGET = RadiaCLI

# Kompilator i flagi
CXX = g++
# Dodajemy pkg-config dla dbus-1, ponieważ libsimpleble go wymaga na Linuxie
CXXFLAGS = -Wall -O2 -std=c++17 $(shell pkg-config --cflags dbus-1)
LDFLAGS = -lsimpleble $(shell pkg-config --libs dbus-1) -lpthread

# Pliki źródłowe i nagłówkowe
SRCS = main.cpp
DEPS = radiacode_lib.h radiacode_buffer.h

# Domyślna zasada (build) - używamy nice dla ochrony zasobów RPi
all: $(TARGET)

$(TARGET): $(SRCS) $(DEPS)
	@echo "Kompilacja z niskim priorytetem (nice) i obsługą D-Bus..."
	nice -n 15 $(CXX) $(CXXFLAGS) $(SRCS) -o $(TARGET) $(LDFLAGS)

# Czyszczenie binariów
clean:
	rm -f $(TARGET)

# Uruchamianie (wymaga sudo dla Bluetooth)
run: $(TARGET)
	sudo ./$(TARGET)