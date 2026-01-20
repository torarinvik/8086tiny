# 8086tiny: a tiny, highly functional, highly portable PC emulator/VM
# Copyright 2013-14, Adrian Cable (adrian.cable@gmail.com) - http://www.megalith.co.uk/8086tiny
#
# This work is licensed under the MIT License. See included LICENSE.TXT.

# 8086tiny builds with graphics and sound support
# 8086tiny_slowcpu improves graphics performance on slow platforms (e.g. Raspberry Pi)
# no_graphics compiles without SDL graphics/sound (legacy target; overwrites 8086tiny)
# headless builds a separate binary without SDL

CXX ?= c++

OPTS_ALL=-O3 -fsigned-char -std=c++20
SDL_CONFIG ?= sdl2-config
OPTS_SDL=$(shell $(SDL_CONFIG) --cflags --libs 2>/dev/null || sdl-config --cflags --libs)
OPTS_NOGFX=-DNO_GRAPHICS
OPTS_SLOWCPU=-DGRAPHICS_UPDATE_DELAY=25000


.PHONY: all bios headless sdl no_graphics clean

NASM ?= nasm
BIOS_SRC ?= bios_source/bios.asm

all: sdl

bios: ${BIOS_SRC}
	@command -v ${NASM} >/dev/null 2>&1 || (echo "Error: '${NASM}' not found. Install NASM (e.g. 'brew install nasm')." && exit 1)
	${NASM} -f bin ${BIOS_SRC} -o bios
	@size=$$(wc -c < bios); \
	if [ $$size -gt 65280 ]; then \
		echo "Error: built BIOS is too large ($$size bytes). Expected <= 65280 bytes."; \
		exit 1; \
	fi; \
	if [ $$size -lt 65280 ]; then \
		pad=$$((65280-$$size)); \
		echo "Padding BIOS with $$pad zero bytes to 65280..."; \
		dd if=/dev/zero bs=1 count=$$pad >> bios 2>/dev/null; \
	fi

sdl: 8086tiny

headless: 8086tiny_headless

8086tiny: 8086tiny.cpp
	${CXX} ${OPTS_ALL} 8086tiny.cpp ${OPTS_SDL} -o 8086tiny
	strip 8086tiny

8086tiny_slowcpu: 8086tiny.cpp
	${CXX} ${OPTS_ALL} 8086tiny.cpp ${OPTS_SDL} ${OPTS_SLOWCPU} -o 8086tiny
	strip 8086tiny


8086tiny_headless: 8086tiny.cpp
	${CXX} ${OPTS_ALL} 8086tiny.cpp ${OPTS_NOGFX} -o 8086tiny_headless
	strip 8086tiny_headless

no_graphics: 8086tiny.cpp
	${CXX} ${OPTS_ALL} 8086tiny.cpp ${OPTS_NOGFX} -o 8086tiny
	strip 8086tiny

clean:
	rm -f 8086tiny 8086tiny_headless
