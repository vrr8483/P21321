
HEADERS = $(shell ls *.h) 

CPP_SRC = PCA9685.cpp ROV.cpp SBUS.cpp
C_SRC = sbus_driver.c
SRC = $(CPP_SRC) $(C_SRC)

BUILD_DIR = build

CPP_OBJS = $(patsubst %.cpp,$(BUILD_DIR)/%.opp, $(CPP_SRC))
C_OBJS = $(patsubst %.c,$(BUILD_DIR)/%.o, $(C_SRC))
OBJS = $(CPP_OBJS) $(C_OBJS)

PROG = ROV

CC = g++
CPPFLAGS = -Wall -pedantic -g
LIBS = -lwiringPi
COMPILE_CMD = $(CC) $(CPPFLAGS) $(LIBS)

.PHONY: all clean help
all: $(PROG)

$(BUILD_DIR)/%.o : %.c $(HEADERS)
	mkdir -p $(dir $@)
	$(COMPILE_CMD) $*.c -c -o $@

$(BUILD_DIR)/%.opp : %.cpp $(HEADERS)
	mkdir -p $(dir $@)
	$(COMPILE_CMD) $*.cpp -c -o $@

$(PROG) : $(OBJS)
	-./stop_running_ROV.sh
	@echo "If the line above this one is an ignored error, "
	@echo "it just means that there was no ROV process in the background to kill. Don't worry about it."
	$(COMPILE_CMD) $^ -o $@

clean:
	rm -rf $(BUILD_DIR) $(PROG)

help:
	echo "make options: all, clean, help"
