# Makefile.am
#
# Copyright (C) 2005 Matthew Flood
# See file AUTHORS for contact information
#
# This file is part of RudeConfig.
#
# RudeConfig is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2, or (at your option)
# any later version.
# 
# RudeConfig is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with RudeConfig; (see COPYING) if not, write to the Free Software
# Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
# 02111-1307, USA.
#------------------------------------------------------------------------
AUTOMAKE_OPTIONS = gnits dist-bzip2 dist-zip subdir-objects

noinst_LTLIBRARIES = librudeconfig.la 
librudeconfig_la_SOURCES = src/AbstractData.cpp \
	src/Comment.cpp \
	src/File.cpp \
	src/Section.cpp \
	src/AbstractOrganiser.cpp \
	src/config.cpp \
	src/KeyValue.cpp \
	src/WhiteSpace.cpp \
	src/AbstractParser.cpp \
	src/ConfigImpl.cpp \
	src/Writer.cpp \
	src/SourceDest.cpp \
	src/AbstractWriter.cpp \
	src/DataLine.cpp \
	src/ParserJuly2004.cpp \
	src/Base64Encoder.cpp \
	src/RealOrganiser.cpp \
	src/AbstractData.h \
	src/AbstractWriter.h \
	src/config.h \
	src/Section.h \
	src/AbstractOrganiser.h \
	src/Base64Encoder.h \
	src/ConfigImpl.h \
	src/File.h \
	src/ParserJuly2004.h \
	src/WhiteSpace.h \
	src/AbstractParser.h \
	src/Comment.h \
	src/DataLine.h \
	src/KeyValue.h \
	src/RealOrganiser.h \
	src/Writer.h \
        src/SourceDest.h

librudeconfig_la_LDFLAGS = -version-info 5:1:2

#pkginclude_HEADERS = src/config.h
#pkgincludedir = $(includedir)/rude

#man_MANS = man3/rudeconfig.3

EXTRA_DIST = $(man_MANS)


