FILES = wifilib.h
MAIN = xbeerecv.cpp
CC = g++
DEBUG = -g
CFLAGS = -Wall -c $(DEBUG)
LFLAGS = -Wall $(DEBUG) -lm

all : $(FILES)
	$(CC) $(MAIN) $(LFLAGS) -o xbeerecv
