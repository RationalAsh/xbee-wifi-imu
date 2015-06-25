FILES = wifi.c wifi.h
MAIN = xbeerecv.c
CC = gcc
DEBUG = -g
CFLAGS = -Wall -c $(DEBUG)
LFLAGS = -Wall $(DEBUG) -lm

all : $(FILES)
	$(CC) $(MAIN) $(LFLAGS) -o xbeerecv
