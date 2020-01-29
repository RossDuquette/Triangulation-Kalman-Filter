PROGRAM_NAME = particle_filter

CC = g++
LD = g++
CFLAGS = -I. -g -O0
LDFLAGS = -lglut -lGL -lncurses
DEPS = $(wildcard *.h)
SRCS = $(wildcard *.cpp)

OBJ_DIR = obj/
$(shell mkdir -p $(OBJ_DIR))
OBJS = $(patsubst %.cpp,$(OBJ_DIR)%.o,$(SRCS))

all: $(PROGRAM_NAME)

$(OBJ_DIR)%.o: %.cpp $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

$(PROGRAM_NAME): $(OBJS)
	$(LD) -o $@ $^ $(LDFLAGS)

clean:
	rm -rf $(OBJ_DIR)
	rm -f $(PROGRAM_NAME)
