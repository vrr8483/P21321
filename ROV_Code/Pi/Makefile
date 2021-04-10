
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

$(BUILD_DIR) : 
	mkdir -p $(BUILD_DIR)

$(BUILD_DIR)/%.o : %.c $(HEADERS) $(BUILD_DIR)
	$(COMPILE_CMD) $*.c -c -o $@

$(BUILD_DIR)/%.opp : %.cpp $(HEADERS) $(BUILD_DIR)
	$(COMPILE_CMD) $*.cpp -c -o $@

$(PROG) : $(OBJS)
	$(COMPILE_CMD) $^ -o $@

clean:
	rm -rf $(OBJS) $(PROG)

help:
	echo "make options: all, clean, help"
