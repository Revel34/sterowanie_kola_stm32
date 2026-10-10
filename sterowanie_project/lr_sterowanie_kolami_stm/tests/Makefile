# Host unit tests for the HAL-free modules. Usage: make (builds and runs).
CC      ?= gcc
SRC_DIR := ../Core/Src
CFLAGS  := -std=c11 -Wall -Wextra -Werror -Wdouble-promotion -I../Core/Inc -I. -g -fsanitize=address,undefined
LDLIBS  := -lm

MODULES := can_proto kinematics pi_ctrl drive_ctrl encoder_math cmd_timeout ring_buf
TESTS   := test_main $(addprefix test_,$(MODULES))
OBJS    := $(addprefix build/,$(addsuffix .o,$(MODULES) $(TESTS)))

.PHONY: all test clean
all: test

test: build/run_tests
	./build/run_tests

build/run_tests: $(OBJS)
	$(CC) $(CFLAGS) $^ -o $@ $(LDLIBS)

build/%.o: $(SRC_DIR)/%.c | build
	$(CC) $(CFLAGS) -c $< -o $@

build/%.o: %.c test.h | build
	$(CC) $(CFLAGS) -c $< -o $@

build:
	mkdir -p build

clean:
	rm -rf build
