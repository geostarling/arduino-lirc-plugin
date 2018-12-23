# ****************************************************************************
# * arduino.c ***********************************************************
# ***************************************************************************
#
# receive keycodes input via /dev/
#
# Copyright (C) 2016 Jiri Spacek <spaceji3@fit.cvut.cz>
#
# Distribute under GPL version 2 or later.

driver          = arduino

CFLAGS          += $(shell pkg-config --cflags lirc-driver) -D HAVE_KERNEL_LIRC_H=1
LDFLAGS         += $(shell pkg-config --libs lirc-driver)
PLUGINDIR       ?= $(shell pkg-config --variable=plugindir lirc-driver)
CONFIGDIR       ?= $(shell pkg-config --variable=configdir lirc-driver)
PLUGINDOCS      ?= $(shell pkg-config --variable=plugindocs lirc-driver)

CC=$(CROSS_COMPILE)gcc

all:  $(driver).so

$(driver).o: $(driver).c

$(driver).so: $(driver).o
	$(CC) --shared -fpic $(LDFLAGS) -o $@ $<

install: $(driver).so
	install $< $(PLUGINDIR)
	install $(driver)-serial.conf $(CONFIGDIR)
	install $(driver).html $(PLUGINDOCS)

clean:
	rm -f *.o *.so
