#
# Copyright (c) 2009 Wind River Systems, Inc.
# Tom Rix <Tom.Rix at windriver.com>
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
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place, Suite 330, Boston,
# MA 02111-1307 USA
#

include $(TOPDIR)/config.mk

LIB 	:= $(obj)libpower.o

COBJS-$(CONFIG_FTPMU010_POWER)	+= ftpmu010.o
COBJS-$(CONFIG_TPS6586X_POWER)	+= tps6586x.o
COBJS-$(CONFIG_TWL4030_POWER)	+= twl4030.o
COBJS-$(CONFIG_TWL6030_POWER)	+= twl6030.o
COBJS-$(CONFIG_TWL6035_POWER)	+= twl6035.o
ifndef CONFIG_SPL_BUILD
COBJS-$(CONFIG_TWL6032_CHARGER)		+= twl6032-gpadc.o
COBJS-$(CONFIG_TWL6032_CHARGER)		+= twl6032-charger.o
COBJS-$(CONFIG_CHARGER_BQ2416x)	+= bq2416x.o
COBJS-$(CONFIG_FUELGAUGE_BQ27541)	+= bq27541.o
COBJS-$(CONFIG_FUELGAUGE_BQ27520)	+= bq27520.o
ifndef CONFIG_BOARD_HAS_NO_TWL6040
COBJS-$(CONFIG_TWL6040_VIBRATOR)+= twl6040-vib.o
endif
COBJS-$(CONFIG_TWL5030_VIBRATOR)+= twl5030-vib.o
COBJS-$(CONFIG_TWL6030_PWM)+= twl6030-pwm.o
endif

COBJS	:= $(COBJS-y)
SRCS 	:= $(COBJS:.o=.c)
OBJS 	:= $(addprefix $(obj),$(COBJS))

all:	$(LIB)

$(LIB):	$(obj).depend $(OBJS)
	$(call cmd_link_o_target, $(OBJS))


#########################################################################

# defines $(obj).depend target
include $(SRCTREE)/rules.mk

sinclude $(obj).depend

########################################################################
