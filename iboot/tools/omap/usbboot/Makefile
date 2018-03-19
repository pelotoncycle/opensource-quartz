##
## Copyright (C) 2013 Texas Instruments, Inc.
## All rights reserved.
##
## Redistribution and use in source and binary forms, with or without
## modification, are permitted provided that the following conditions
## are met:
##  * Redistributions of source code must retain the above copyright
##    notice, this list of conditions and the following disclaimer.
##  * Redistributions in binary form must reproduce the above copyright
##    notice, this list of conditions and the following disclaimer in
##    the documentation and/or other materials provided with the
##    distribution.
##
## THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
## "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
## LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
## FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
## COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
## INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
## BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
## OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
## AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
## OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
## OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
## SUCH DAMAGE.
##

QUIET ?= @

HOST_CC ?= gcc
HOST_CFLAGS = -m32
HOST_CFLAGS += -g -O2 -Wall
HOST_CFLAGS += -I./
#HOST_CFLAGS += -DTRACE_USB

HOST_OBJS = usbboot.o usb_linux.o

M_NAME	= usbboot

%.o : %.c Makefile
	$(QUIET)gcc -c -o $@ $< $(HOST_CFLAGS)

$(M_NAME): $(HOST_OBJS)
	@echo "\n                        NOTICE"
	@echo "  This Makefile is external to project omapboot's Makefile"
	@echo "  and generates a stand-alone version of usbboot\n"
	$(QUIET)gcc -static -o $@ $^ $(HOST_CFLAGS)
	$(QUIET)rm -f $(HOST_OBJS)
	$(QUIET)chmod +x $@
	@echo "  $@ ready!\n"

PHONY: clean

clean:
	$(QUIET)rm -f $(M_NAME) $(HOST_OBJS)
