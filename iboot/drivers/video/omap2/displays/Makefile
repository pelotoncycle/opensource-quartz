#
# (C) Copyright 2000-2007
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
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place, Suite 330, Boston,
# MA 02111-1307 USA
#

include $(TOPDIR)/config.mk

LIB	:= $(obj)libpanel.o

COBJS-$(CONFIG_PANEL_DPI2LVDS) += panel-icom-dpi2lvds.o
COBJS-$(CONFIG_PANEL_HV070WX1) += panel-hv070wx1.o
COBJS-$(CONFIG_PANEL_TC358765) += panel-tc358765.o
COBJS-$(CONFIG_PANEL_LS047K1SX01F) += panel-ls047k1sx01f.o
COBJS-$(CONFIG_PANEL_LS047K1SX01) += panel-ls047k1sx01.o
COBJS-$(CONFIG_PANEL_GTN070WX1) += panel-gtn070wx1.o
COBJS-$(CONFIG_PANEL_GTN101WX1) += panel-gtn101wx1.o

COBJS	:= $(sort $(COBJS-y))
SRCS	:= $(COBJS:.o=.c)
OBJS	:= $(addprefix $(obj),$(COBJS))

all:	$(LIB)

$(LIB):	$(obj).depend $(OBJS)
	$(call cmd_link_o_target, $(OBJS))

#########################################################################

# defines $(obj).depend target
include $(SRCTREE)/rules.mk

sinclude $(obj).depend

#########################################################################
