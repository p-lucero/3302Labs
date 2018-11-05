IDIR=.
CC=g++
CFLAGS=-I$(IDIR) -x c++ -DSPARKI_HEADLESS_DEBUGGING

sparki_code = ../lab6/lab6.ino

driver:
	$(CC) $(CFLAGS) -o driver.o $(sparki_code) driver.cpp

run: driver
	./driver.o