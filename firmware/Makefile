# redPitaLock firmware -- build on the Red Pitaya (ARM Linux)
#
# Usage:
#   ssh root@rp-XXXX.local
#   cd /root/redPitaLock/firmware
#   make
#   ./stabilizer_rp

CC      = gcc
CFLAGS  = -O2 -Wall -Wextra -std=gnu11 -Isrc -I/opt/redpitaya/include
LDFLAGS = -L/opt/redpitaya/lib -Wl,-rpath,/opt/redpitaya/lib -lm -lpthread -lrp

SRC     = src/main.c src/pid.c src/aom_lut.c src/analog_io.c src/tcp_server.c
OBJ     = $(SRC:.c=.o)
TARGET  = stabilizer_rp

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(OBJ)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

src/%.o: src/%.c
	$(CC) $(CFLAGS) -c -o $@ $<

clean:
	rm -f $(OBJ) $(TARGET)
