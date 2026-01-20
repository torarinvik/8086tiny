#pragma once

#include <cstddef>
#include <cstdint>

#if defined(_WIN32)
	#include <conio.h>
	#include <io.h>
#else
	#include <unistd.h>
#endif

namespace platform
{
	// Poll a single key from stdin.
	// - On Windows uses _kbhit/_getch.
	// - On POSIX uses read(0, ...) (may block, matching the original behavior).
	//
	// Returns true if a key was read and the interrupt callback was invoked.
	template <typename InterruptFn>
	inline bool poll_keyboard(std::uint8_t *mem, std::uint8_t &int8_asap, InterruptFn &&interrupt)
	{
#if defined(_WIN32)
		if (!_kbhit())
			return false;
		const int ch = _getch();
		mem[0x4A6] = (std::uint8_t)ch;
		interrupt((unsigned char)7);
		return true;
#else
		const ssize_t n = ::read(0, mem + 0x4A6, 1);
		if (n <= 0)
			return false;
		int8_asap = (std::uint8_t)(mem[0x4A6] == 0x1B);
		interrupt((unsigned char)7);
		return true;
#endif
	}
}
