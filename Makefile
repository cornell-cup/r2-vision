CC = g++
CFLAGS = -std=c++11 -O3 -Wall
LDFLAGS = -lpthread -lm -lopencv_world -lapriltag -lcurl

all: calibrate_cameras.x setup_tags.x find_location.x setup_tags_refactored.x

%.x: %.cc
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

.PHONY: all

clean:
	-rm *.x
