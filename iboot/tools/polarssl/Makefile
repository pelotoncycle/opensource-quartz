#
# (C) Copyright 2000-2006
# Wolfgang Denk, DENX Software Engineering, wd@denx.de.
#
# See file CREDITS for list of people who contributed to this
# project.
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License as
# published by the Free Software Foundation; either version 2 of
# the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place, Suite 330, Boston,
# MA 02111-1307 USA
#


#
# Include this after HOSTOS HOSTARCH check
# so that we can act intelligently.
#
include $(TOPDIR)/config.mk

#
# toolchains targeting win32 generate .exe files
#
ifneq (,$(findstring WIN32 ,$(shell $(HOSTCC) -E -dM -xc /dev/null)))
SFX = .exe
else
SFX =
endif

LIB	= $(obj)polarssl.o

# PolarSSL tree objects
POLARSSL_OBJ_FILES-y =	aes.o		arc4.o		asn1parse.o		\
						base64.o	bignum.o	camellia.o		\
						certs.o		cipher.o	cipher_wrap.o	\
						ctr_drbg.o	debug.o		des.o			\
						dhm.o		entropy.o	entropy_poll.o	\
						error.o		havege.o					\
						md.o		md_wrap.o	md2.o			\
						md4.o		md5.o		net.o			\
						padlock.o	pem.o		pkcs11.o		\
						rsa.o		sha1.o		sha2.o			\
						sha4.o		ssl_cli.o	ssl_srv.o		\
						ssl_tls.o	timing.o	version.o		\
						x509parse.o	xtea.o

# now $(obj) is defined
HOSTSRCS := $(addprefix $(SRCTREE)/tools/polarssl/library/,$(POLARSSL_OBJ_FILES-y:.o=.c))
POLARSSL_OBJS	:= $(addprefix $(obj),$(POLARSSL_OBJ_FILES-y))

HOSTOBJS := $(addprefix $(obj),$(OBJ_FILES-y))
NOPEDOBJS := $(addprefix $(obj),$(NOPED_OBJ_FILES-y))

#
# Use native tools and options
# Define __KERNEL_STRICT_NAMES to prevent typedef overlaps
#
HOSTCPPFLAGS = -I $(SRCTREE)/tools/polarssl/include \
		-D_FILE_OFFSET_BITS=64 -Wall -W -Wdeclaration-after-statement \
		-DUSE_HOSTCC \
		-D__KERNEL_STRICT_NAMES

cmd_link_o_host = $(if $(strip $1),\
		ld -r -o $@ $1,rm -f $@; $(AR) rcs $@)

all:	$(LIB)

$(LIB):	$(obj).depend $(POLARSSL_OBJS)
	$(call cmd_link_o_host, $(POLARSSL_OBJS))

# Some of the tool objects need to be accessed from outside the tools directory
$(obj)%.o: $(SRCTREE)/tools/polarssl/library/%.c
	$(HOSTCC) -g $(HOSTCFLAGS_NOPED) -c -o $@ $<

#########################################################################

# defines $(obj).depend target
include $(SRCTREE)/rules.mk

sinclude $(obj).depend

#########################################################################

