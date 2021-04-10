
HEADERS = $(shell ls *.h) 

CPP_SRC = PCA9685.cpp ROV.cpp SBUS.cpp
C_SRC = sbus_driver.c
SRC = $(CPP_SRC) $(C_SRC)

CPP_OBJS = $(patsubst %.cpp,%.opp, $(CPP_SRC))
C_OBJS = $(patsubst %.c,%.o, $(C_SRC))
OBJS = $(CPP_OBJS) $(C_OBJS)

PROG = ROV

CC = g++
CPPFLAGS = -Wall -pedantic -g
LIBS = -lwiringPi
COMPILE_CMD = $(CC) $(CPPFLAGS) $(LIBS)

%.o : %.c $(HEADERS)
	$(COMPILE_CMD) $*.c -c -o $@

%.opp : %.cpp $(HEADERS)
	$(COMPILE_CMD) $*.cpp -c -o $@

$(PROG) : $(OBJS)
	$(COMPILE_CMD) $^ -o $@

.PHONY: all clean help
all: $(PROG)

clean:
	rm -rf $(OBJS) $(PROG)

help:
	echo "make options: all, clean, help"
