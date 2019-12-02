PROGRAM_NAME = particle_filter

CC = g++
LD = g++
CFLAGS = -I. -g -O0
LDFLAGS = -lglut -lGL
DEPS = $(wildcard *.h)
SRCS = $(wildcard *.cpp)

OBJS = $(patsubst %.cpp,%.o,$(SRCS))

all: $(PROGRAM_NAME)

%.o: %.cpp $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

$(PROGRAM_NAME): $(OBJS)
	$(LD) -o $@ $^ $(LDFLAGS)

clean:
	rm -f $(OBJS)
	rm -f $(PROGRAM_NAME)
