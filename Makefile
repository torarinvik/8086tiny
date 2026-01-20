# 8086tiny: a tiny, highly functional, highly portable PC emulator/VM
# Copyright 2013-14, Adrian Cable (adrian.cable@gmail.com) - http://www.megalith.co.uk/8086tiny
#
# This work is licensed under the MIT License. See included LICENSE.TXT.

# 8086tiny builds with graphics and sound support
# 8086tiny_slowcpu improves graphics performance on slow platforms (e.g. Raspberry Pi)
# no_graphics compiles without SDL graphics/sound

CXX ?= c++

OPTS_ALL=-O3 -fsigned-char -std=c++20
SDL_CONFIG ?= sdl2-config
OPTS_SDL=$(shell $(SDL_CONFIG) --cflags --libs 2>/dev/null || sdl-config --cflags --libs)
OPTS_NOGFX=-DNO_GRAPHICS
OPTS_SLOWCPU=-DGRAPHICS_UPDATE_DELAY=25000

8086tiny: 8086tiny.cpp
	${CXX} ${OPTS_ALL} 8086tiny.cpp ${OPTS_SDL} -o 8086tiny
	strip 8086tiny

8086tiny_slowcpu: 8086tiny.cpp
	${CXX} ${OPTS_ALL} 8086tiny.cpp ${OPTS_SDL} ${OPTS_SLOWCPU} -o 8086tiny
	strip 8086tiny

no_graphics: 8086tiny.cpp
	${CXX} ${OPTS_ALL} 8086tiny.cpp ${OPTS_NOGFX} -o 8086tiny
	strip 8086tiny

clean:
	rm -f 8086tiny
