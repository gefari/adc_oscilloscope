CC      = gcc
CFLAGS  = -Wall -Wextra -O2
DEPFLAGS = -MMD -MP
LDLIBS  = -lgpiod -lpthread -lrt
TARGET  = adc_oscilloscope
SRCS    = main.c realTime.c sharedMemory.c ads1263.c rp1_dma.c
OBJS    = $(SRCS:.c=.o)
DEPS    = $(SRCS:.c=.d)

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDLIBS)

%.o: %.c
	$(CC) $(CFLAGS) $(DEPFLAGS) -c -o $@ $<

clean:
	rm -f $(OBJS) $(DEPS) $(TARGET)

-include $(DEPS)
