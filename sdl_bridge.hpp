#pragma once

// Small bridge layer so 8086tiny.cpp can call SDL-dependent helpers without
// sprinkling NO_GRAPHICS conditionals around the main loop.

namespace sdl_bridge
{
#if defined(NO_GRAPHICS)
	inline bool has_window() { return false; }
	inline int poll_keyboard() { return 0; }
#else
	bool has_window();
	int poll_keyboard();
#endif
}
