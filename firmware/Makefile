# redPitaLock firmware -- build on the Red Pitaya (ARM Linux)
#
# Usage:
#   ssh root@rp-XXXX.local
#   cd /root/redPitaLock/firmware
#   make
#   ./stabilizer_rp

CC       = gcc
CXX      = g++
CFLAGS   = -O2 -Wall -Wextra -std=gnu11 -Isrc -I/opt/redpitaya/include
CXXFLAGS = -O2 -Wall -Wextra -std=gnu++20 -Isrc -I/opt/redpitaya/include
LDFLAGS  = -L/opt/redpitaya/lib -Wl,-rpath,/opt/redpitaya/lib -lm -lpthread -lrp

# Red Pitaya OS 3.x ships librp as a C++ library (mangled symbols, std::span /
# std::vector in the public headers), so it cannot be used from C any more.
# analog_io.c is the only file that touches librp: on such an OS it is compiled
# as C++ (its API is exported with C linkage, see analog_io.h) and the final
# link is done with g++. OS 2.x still builds everything as plain C.
RP_CXX_API := $(shell grep -q 'std::' /opt/redpitaya/include/rp_gen.h 2>/dev/null && echo 1)
ifeq ($(RP_CXX_API),1)
LINK = $(CXX)
else
LINK = $(CC)
endif

SRC     = src/main.c src/pid.c src/aom_lut.c src/analog_io.c src/autotune.c src/psd.c src/tcp_server.c
OBJ     = $(SRC:.c=.o)
TARGET  = stabilizer_rp

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(OBJ)
	$(LINK) -o $@ $^ $(LDFLAGS)

ifeq ($(RP_CXX_API),1)
src/analog_io.o: src/analog_io.c
	$(CXX) $(CXXFLAGS) -x c++ -c -o $@ $<
endif

src/%.o: src/%.c
	$(CC) $(CFLAGS) -c -o $@ $<

clean:
	rm -f $(OBJ) $(TARGET)
