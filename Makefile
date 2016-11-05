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

CFLAGS          += $(shell pkg-config --cflags lirc-driver)
LDFLAGS         += $(shell pkg-config --libs lirc-driver)
PLUGINDIR       ?= $(shell pkg-config --variable=plugindir lirc-driver)
CONFIGDIR       ?= $(shell pkg-config --variable=configdir lirc-driver)
PLUGINDOCS      ?= $(shell pkg-config --variable=plugindocs lirc-driver)

all:  $(driver).so

$(driver).o: $(driver).c

$(driver).so: $(driver).o
	gcc --shared -fpic $(LDFLAGS) -o $@ $<

install: $(driver).so
	install $< $(PLUGINDIR)
	install $(driver).conf $(CONFIGDIR)
	install $(driver).html $(PLUGINDOCS)

clean:
	rm -f *.o *.so
