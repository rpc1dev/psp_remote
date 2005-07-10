CC          = gcc
TARGET      = psp_remote
CFLAGS      = -O4 -g -Wall
LDFLAGS     = -lncurses

psp_remote: psp_remote.c
	 $(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^

