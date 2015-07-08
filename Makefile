FILES = wifilib.h
MAIN = xbeerecv.cpp
CC = gcc
DEBUG = -g
CFLAGS = -Wall -c $(DEBUG)
LFLAGS = -Wall $(DEBUG) -lm -lpthread -lstdc++

all : $(FILES)
	$(CC) $(MAIN) $(LFLAGS) -o xbeerecv
