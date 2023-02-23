TARGET = demo

CC = gcc
SRCS = $(wildcard src/*.c)
OBJS = $(SRCS:src/%.c=src/%.o)
INCS = -I./deps/cglm/include -I../Fusion/Fusion
LIBS = -lm -lhidapi-libusb -lglfw -lGL -lGLEW -lpthread -L../Fusion/Fusion/ -lFusion

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) -o $@ $^ $(LIBS)

src/%.o: src/%.c
	$(CC) $(INCS) -c -o $@ $<

clean:
	$(RM) $(TARGET) src/*.o
