CC=gcc

default: canusb

canusb: canusb.c
	$(CC) $< -o $@
