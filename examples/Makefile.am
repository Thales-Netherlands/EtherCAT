#------------------------------------------------------------------------------
#
#  $Id$
#
#  Copyright (C) 2006-2012  Florian Pose, Ingenieurgemeinschaft IgH
#
#  This file is part of the IgH EtherCAT Master.
#
#  The IgH EtherCAT Master is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License version 2, as
#  published by the Free Software Foundation.
#
#  The IgH EtherCAT Master is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General
#  Public License for more details.
#
#  You should have received a copy of the GNU General Public License along
#  with the IgH EtherCAT Master; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
#
#  ---
#
#  The license mentioned above concerns the source code only. Using the
#  EtherCAT technology and brand is only permitted in compliance with the
#  industrial property and similar rights of Beckhoff Automation GmbH.
#
#------------------------------------------------------------------------------
include $(top_srcdir)/Makefile.kbuild

SUBDIRS =

if ENABLE_USERLIB
SUBDIRS += \
	dc_user \
	user
endif

if ENABLE_TTY
SUBDIRS += \
	tty
endif

if ENABLE_RTDM
if ENABLE_XENOMAI
SUBDIRS += \
	xenomai \
	xenomai_posix
endif

if ENABLE_RTAI
SUBDIRS += \
	rtai_rtdm \
	rtai_rtdm_dc
endif
endif

# Here DIST_SUBDIRS needs to be explicitely defined because
# dc_rtai, mini and rtai are never added to `SUBDIRS`
DIST_SUBDIRS = \
	dc_rtai \
	dc_user \
	mini \
	rtai \
	rtai_rtdm \
	rtai_rtdm_dc \
	tty \
	user \
	xenomai \
	xenomai_posix

#------------------------------------------------------------------------------
