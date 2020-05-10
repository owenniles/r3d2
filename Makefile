# Makefile

# ESP-IDF project Makefile.

# Copyright 2020 Owen Niles <oniles@college.harvard.edu>

# This file is part of R3D2.

# R3D2 is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# R3D2 is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.

SHELL = /bin/sh

PROJECT_NAME := r3d2

VPATH = doc

ifdef IDF_PATH
include $(IDF_PATH)/make/project.mk
endif

.PHONY: all pdf clean-pdf
all: pdf

pdf: r3d2.pdf

r3d2.pdf: r3d2.tex
	TEXINPUTS=.:doc//:$(TEXINPUTS) latexmk -pdf $<

clean-pdf:
	latexmk -C -f r3d2.pdf
