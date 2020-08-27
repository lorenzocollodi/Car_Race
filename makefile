CC = gcc
CFLAGS = -Wall -g -O2
CONF = -I.
ALLEG = `allegro-config --libs`
PTHREAD = -pthread -lrt 
MATH = -lm

.PHONY: all clean

all: bin/main

clean:
	rm -f bin/* build/*
	rmdir bin/ build/


build/main.o: src/main.c
	mkdir -p build/
	$(CC) -c $(CONF) $< -o $@

build/%.o: src/%.c src/%.h
	mkdir -p build/
	$(CC) -c $(CONF) $< -o $@

################
# Executables  #
################

bin/main: build/main.o build/car.o build/task_management.o
	mkdir -p bin/
	$(CC) $^ -o $@ $(ALLEG) $(PTHREAD) $(MATH)
