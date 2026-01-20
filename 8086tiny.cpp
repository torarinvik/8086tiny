// 8086tiny: a tiny, highly functional, highly portable PC emulator/VM
// Copyright 2013-14, Adrian Cable (adrian.cable@gmail.com) - http://www.megalith.co.uk/8086tiny
//
// Revision 1.25
//
// This work is licensed under the MIT License. See included LICENSE.TXT.

#include <ctime>
#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <cstdint>
#include <memory>
#include <type_traits>
#include <vector>
#include <sys/timeb.h>

#include "platform_io.hpp"
#include "platform_keyboard.hpp"
#include "sdl_bridge.hpp"

using u8 = std::uint8_t;
using u16 = std::uint16_t;
using u32 = std::uint32_t;
using s32 = std::int32_t;

#ifndef NO_GRAPHICS
#if __has_include(<SDL2/SDL.h>)
#include <SDL2/SDL.h>
#else
#include <SDL.h>
#endif
#endif

// Emulator system constants
constexpr std::size_t IO_PORT_COUNT = 0x10000;
constexpr std::size_t RAM_SIZE = 0x10FFF0;
constexpr std::size_t REGS_BASE = 0xF0000;
constexpr std::size_t VIDEO_RAM_SIZE = 0x10000;

// Graphics/timer/keyboard update delays (explained later)
#ifndef GRAPHICS_UPDATE_DELAY
#define GRAPHICS_UPDATE_DELAY 360000
#endif
constexpr u32 kGraphicsUpdateDelay = (u32)GRAPHICS_UPDATE_DELAY;
constexpr u32 kKeyboardTimerUpdateDelay = 20000;

template <typename E>
constexpr std::underlying_type_t<E> to_underlying(E e) noexcept
{
	return static_cast<std::underlying_type_t<E>>(e);
}

// 8-bit register decodes
enum class Reg8 : int
{
	AL = 0,
	AH = 1,
	CL = 2,
	CH = 3,
	DL = 4,
	DH = 5,
	BL = 6,
	BH = 7
};

// 16-bit register decodes
enum class Reg16 : int
{
	AX = 0,
	CX = 1,
	DX = 2,
	BX = 3,
	SP = 4,
	BP = 5,
	SI = 6,
	DI = 7,
	ES = 8,
	CS = 9,
	SS = 10,
	DS = 11,
	ZERO = 12,
	SCRATCH = 13
};

// FLAGS register decodes
enum class Flag : int
{
	CF = 40,
	PF = 41,
	AF = 42,
	ZF = 43,
	SF = 44,
	TF = 45,
	IF = 46,
	DF = 47,
	OF = 48
};

constexpr int idx(Reg8 r) noexcept { return to_underlying(r); }
constexpr int idx(Reg16 r) noexcept { return to_underlying(r); }
constexpr int idx(Flag f) noexcept { return to_underlying(f); }

// Lookup tables in the BIOS binary
enum class BiosTable : int
{
	XLAT_OPCODE = 8,
	XLAT_SUBFUNCTION = 9,
	STD_FLAGS = 10,
	PARITY_FLAG = 11,
	BASE_INST_SIZE = 12,
	I_W_SIZE = 13,
	I_MOD_SIZE = 14,
	COND_JUMP_DECODE_A = 15,
	COND_JUMP_DECODE_B = 16,
	COND_JUMP_DECODE_C = 17,
	COND_JUMP_DECODE_D = 18,
	FLAGS_BITFIELDS = 19,
};

constexpr int idx(BiosTable t) noexcept { return to_underlying(t); }

// Bitfields for TABLE_STD_FLAGS values
constexpr u32 FLAGS_UPDATE_SZP = 1;
constexpr u32 FLAGS_UPDATE_AO_ARITH = 2;
constexpr u32 FLAGS_UPDATE_OC_LOGIC = 4;

// Helper macros

// Decode mod, r_m and reg fields in instruction
static inline void decode_rm_reg();

// Return memory-mapped register location (offset into mem array) for register #reg_id
static inline u32 get_reg_addr(u32 reg_id) noexcept;

// Returns number of top bit in operand (i.e. 8 for 8-bit operands, 16 for 16-bit operands)
static inline u32 top_bit() noexcept;

template <typename T>
struct MemRef
{
	u8 *p;

	static inline T load(const u8 *ptr) noexcept
	{
		if constexpr (sizeof(T) == 1)
			return (T)*ptr;
		T v;
		std::memcpy(&v, ptr, sizeof(T));
		return v;
	}

	static inline void store(u8 *ptr, T v) noexcept
	{
		if constexpr (sizeof(T) == 1)
		{
			*ptr = (u8)v;
			return;
		}
		std::memcpy(ptr, &v, sizeof(T));
	}

	operator T() const noexcept
	{
		return load(p);
	}

	MemRef &operator=(T v) noexcept
	{
		store(p, v);
		return *this;
	}

	MemRef &operator+=(T rhs) noexcept
	{
		T v = (T)*this;
		v = (T)(v + rhs);
		return (*this = v);
	}
	MemRef &operator-=(T rhs) noexcept
	{
		T v = (T)*this;
		v = (T)(v - rhs);
		return (*this = v);
	}
	MemRef &operator^=(T rhs) noexcept
	{
		T v = (T)*this;
		v = (T)(v ^ rhs);
		return (*this = v);
	}
	MemRef &operator|=(T rhs) noexcept
	{
		T v = (T)*this;
		v = (T)(v | rhs);
		return (*this = v);
	}
	MemRef &operator&=(T rhs) noexcept
	{
		T v = (T)*this;
		v = (T)(v & rhs);
		return (*this = v);
	}
	MemRef &operator<<=(int rhs) noexcept
	{
		T v = (T)*this;
		v = (T)(v << rhs);
		return (*this = v);
	}
	MemRef &operator>>=(int rhs) noexcept
	{
		T v = (T)*this;
		v = (T)(v >> rhs);
		return (*this = v);
	}
};

template <typename T, typename U>
static inline MemRef<T> ref(U &x) noexcept
{
	return MemRef<T>{reinterpret_cast<u8 *>(&x)};
}

template <typename T, typename U>
static inline T val(U &&x) noexcept
{
	if constexpr (std::is_lvalue_reference_v<U &&>)
		return (T)MemRef<T>{reinterpret_cast<u8 *>(&x)};
	else
		return (T)x;
}

// [I]MUL/[I]DIV/DAA/DAS/ADC/SBB helpers
template <typename OpDataType, typename RegT>
static inline int mul_op(RegT *out_regs) noexcept;

template <typename OutDataType, typename InDataType, typename RegT>
static inline int div_op(RegT *out_regs) noexcept;

static inline int daa() noexcept;
static inline int das() noexcept;

static inline int adc_op() noexcept;
static inline int sbb_op() noexcept;

// Execute arithmetic/logic operations in emulator memory/registers
#define R_M_OP(dest,op,src) (i_w ? op_dest = ref<unsigned short>(dest), op_result = ref<unsigned short>(dest) op (op_source = val<unsigned short>(src)) \
									 : (op_dest = dest, op_result = dest op (op_source = val<unsigned char>(src))))
#define MEM_OP(dest,op,src) R_M_OP(mem[dest],op,mem[src])
#define OP(op) MEM_OP(op_to_addr,op,op_from_addr)

// Increment or decrement a register #reg_id (usually SI or DI), depending on direction flag and operand size (given by i_w)
static inline int index_inc(u32 reg_id) noexcept;

// Helpers for stack operations
static inline u32 segreg(u32 seg_reg, u16 offset) noexcept;
static inline u32 r_m_push(u32 value) noexcept;
template <typename Dest>
static inline u32 r_m_pop(Dest &dest) noexcept;

// Convert segment:offset to linear address in emulator memory space
static inline u32 segreg(u32 seg_reg, u16 offset) noexcept;

// Returns sign bit of an 8-bit or 16-bit operand
template <typename A>
static inline u32 sign_of(A &&a) noexcept;

char pc_interrupt(unsigned char interrupt_num);

extern u8 mem[RAM_SIZE];
extern u8 int8_asap;

static inline int poll_console_keyboard()
{
	return platform::poll_keyboard(mem, int8_asap, [](unsigned char n) { pc_interrupt(n); }) ? 1 : 0;
}

static inline int poll_active_keyboard()
{
	return sdl_bridge::has_window() ? sdl_bridge::poll_keyboard() : poll_console_keyboard();
}

// Keyboard driver for SDL
#ifndef NO_GRAPHICS
static unsigned int sdl_ascii_from_keysym(const SDL_Keysym &keysym)
{
	SDL_Keycode sym = keysym.sym;
	SDL_Keymod mod = (SDL_Keymod)keysym.mod;
	bool shift = (mod & KMOD_SHIFT) != 0;

	if (sym >= SDLK_a && sym <= SDLK_z)
		return (unsigned int)(shift ? (sym - SDLK_a + 'A') : (sym - SDLK_a + 'a'));
	if (sym >= SDLK_0 && sym <= SDLK_9)
	{
		static const char shifted_digits[] = ")!@#$%^&*(";
		return (unsigned int)(shift ? shifted_digits[sym - SDLK_0] : (sym - SDLK_0 + '0'));
	}

	switch (sym)
	{
		case SDLK_SPACE: return ' ';
		case SDLK_RETURN: return '\r';
		case SDLK_TAB: return '\t';
		case SDLK_BACKSPACE: return '\b';
		case SDLK_MINUS: return shift ? '_' : '-';
		case SDLK_EQUALS: return shift ? '+' : '=';
		case SDLK_LEFTBRACKET: return shift ? '{' : '[';
		case SDLK_RIGHTBRACKET: return shift ? '}' : ']';
		case SDLK_BACKSLASH: return shift ? '|' : '\\';
		case SDLK_SEMICOLON: return shift ? ':' : ';';
		case SDLK_QUOTE: return shift ? '"' : '\'';
		case SDLK_BACKQUOTE: return shift ? '~' : '`';
		case SDLK_COMMA: return shift ? '<' : ',';
		case SDLK_PERIOD: return shift ? '>' : '.';
		case SDLK_SLASH: return shift ? '?' : '/';
		default: return 0;
	}
}

static int sdl_keyboard_driver();
#endif

// Global variable definitions
alignas(16) u8 mem[RAM_SIZE];
alignas(16) u8 io_ports[IO_PORT_COUNT];
u8 *opcode_stream, *regs8, i_rm, i_w, i_reg, i_mod, i_mod_size, i_d, i_reg4bit, raw_opcode_id, xlat_opcode_id, extra, rep_mode, seg_override_en, rep_override_en, trap_flag, int8_asap, scratch_uchar, io_hi_lo, *vid_mem_base, spkr_en, bios_table_lookup[20][256];
u16 *regs16, reg_ip, seg_override, file_index, wave_counter;
u32 op_source, op_dest, rm_addr, op_to_addr, op_from_addr, i_data0, i_data1, i_data2, scratch_uint, scratch2_uint, inst_counter, set_flags_type, GRAPHICS_X, GRAPHICS_Y, pixel_colors[16], vmem_ctr;
s32 op_result, disk[3], scratch_int;
time_t clock_buf;
struct timeb ms_clock;

static inline void decode_rm_reg()
{
	scratch2_uint = 4u * !i_mod;

	if (i_mod < 3)
	{
		const u32 seg_reg = seg_override_en ? (u32)seg_override : (u32)bios_table_lookup[scratch2_uint + 3][i_rm];
		const u32 ofs_reg = (u32)bios_table_lookup[scratch2_uint][i_rm];
		const u32 base_reg = (u32)bios_table_lookup[scratch2_uint + 1][i_rm];
		const u32 disp_mul = (u32)bios_table_lookup[scratch2_uint + 2][i_rm];

		const u32 ofs_val = (u32)regs16[ofs_reg];
		const u32 base_val = (u32)regs16[base_reg];
		const u16 linear_ofs = (u16)(base_val + disp_mul * i_data1 + ofs_val);
		op_to_addr = rm_addr = 16u * (u32)regs16[seg_reg] + (u32)linear_ofs;
	}
	else
	{
		op_to_addr = rm_addr = get_reg_addr(i_rm);
	}

	op_from_addr = get_reg_addr(i_reg);

	if (i_d)
	{
		scratch_uint = op_from_addr;
		op_from_addr = rm_addr;
		op_to_addr = scratch_uint;
	}
}

static inline u32 get_reg_addr(u32 reg_id) noexcept
{
	// Preserve original precedence/behavior from the macro version.
	return (u32)REGS_BASE + (u32)(i_w ? (2u * reg_id) : ((2u * reg_id + reg_id / 4u) & 7u));
}

static inline u32 top_bit() noexcept
{
	return 8u * (u32)(i_w + 1u);
}

template <typename A>
static inline u32 sign_of(A &&a) noexcept
{
	const s32 v = i_w ? (s32)val<short>(a) : (s32)val<unsigned char>(a);
	return (u32)((v >> (top_bit() - 1u)) & 1);
}

static inline void set_opcode(unsigned char opcode);
static inline char set_CF(int new_CF);
static inline char set_OF(int new_OF);

template <typename OpDataType, typename RegT>
static inline int mul_op(RegT *out_regs) noexcept
{
	set_opcode(0x10); // Decode like ADC

	if constexpr (std::is_signed_v<OpDataType>)
	{
		const s32 lhs = (s32)val<OpDataType>(mem[rm_addr]);
		const s32 rhs = (s32)(OpDataType)(*out_regs);
		op_result = lhs * rhs;
	}
	else
	{
		const u32 lhs = (u32)val<OpDataType>(mem[rm_addr]);
		const u32 rhs = (u32)(OpDataType)(*out_regs);
		const u32 prod = lhs * rhs;
		std::memcpy(&op_result, &prod, sizeof(prod));
	}

	// Preserve the original write order (high part first, then AX).
	out_regs[i_w + 1] = (RegT)((u32)op_result >> 16);
	regs16[idx(Reg16::AX)] = (u16)(u32)op_result;

	set_OF(set_CF(op_result - (s32)(OpDataType)op_result));
	return 0;
}

template <typename OutDataType, typename InDataType, typename RegT>
static inline int div_op(RegT *out_regs) noexcept
{
	scratch_int = (s32)val<OutDataType>(mem[rm_addr]);
	if (!scratch_int)
	{
		pc_interrupt(0);
		return 0;
	}

	// Preserve the original behavior: stash the raw (DX:AX) / (AH:AL) dividend bits in scratch_uint.
	scratch_uint = ((u32)out_regs[i_w + 1] << 16) + (u32)regs16[idx(Reg16::AX)];

	const InDataType dividend = (InDataType)scratch_uint;
	const InDataType divisor = (InDataType)scratch_int;
	const InDataType quotient = dividend / divisor;
	const InDataType remainder = dividend - quotient * divisor;

	const OutDataType q_trunc = (OutDataType)quotient;
	if ((InDataType)q_trunc != quotient)
	{
		pc_interrupt(0);
		return 0;
	}

	*out_regs = (RegT)q_trunc;
	out_regs[i_w + 1] = (RegT)(OutDataType)remainder;
	return 0;
}

static inline int index_inc(u32 reg_id) noexcept
{
	const int df = regs8[idx(Flag::DF)] ? 1 : 0;
	const int delta = (2 * df - 1) * (int)(i_w + 1);
	regs16[reg_id] = (u16)(regs16[reg_id] - delta);
	return 0;
}

static inline u32 segreg(u32 seg_reg, u16 offset) noexcept
{
	return 16u * (u32)regs16[seg_reg] + (u32)offset;
}

static inline u32 r_m_push(u32 value) noexcept
{
	i_w = 1;
	R_M_OP(mem[segreg(idx(Reg16::SS), (u16)--regs16[idx(Reg16::SP)])], =, value);
	return (u32)op_result;
}

template <typename Dest>
static inline u32 r_m_pop(Dest &dest) noexcept
{
	i_w = 1;
	regs16[idx(Reg16::SP)] += 2;
	R_M_OP(dest, =, mem[segreg(idx(Reg16::SS), (u16)(regs16[idx(Reg16::SP)] - 2))]);
	return (u32)op_result;
}

#ifndef NO_GRAPHICS
SDL_AudioSpec sdl_audio;
SDL_AudioDeviceID sdl_audio_dev;
struct SdlWindowDeleter
{
	void operator()(SDL_Window *p) const noexcept
	{
		if (p)
			SDL_DestroyWindow(p);
	}
};
struct SdlRendererDeleter
{
	void operator()(SDL_Renderer *p) const noexcept
	{
		if (p)
			SDL_DestroyRenderer(p);
	}
};
struct SdlTextureDeleter
{
	void operator()(SDL_Texture *p) const noexcept
	{
		if (p)
			SDL_DestroyTexture(p);
	}
};

std::unique_ptr<SDL_Window, SdlWindowDeleter> sdl_window;
std::unique_ptr<SDL_Renderer, SdlRendererDeleter> sdl_renderer;
std::unique_ptr<SDL_Texture, SdlTextureDeleter> sdl_texture;
std::vector<std::uint32_t> sdl_pixels;
SDL_Event sdl_event;
u16 vid_addr_lookup[VIDEO_RAM_SIZE], cga_colors[4] = {0 /* Black */, 0x1F1F /* Cyan */, 0xE3E3 /* Magenta */, 0xFFFF /* White */};
#endif

#ifndef NO_GRAPHICS
char pc_interrupt(unsigned char interrupt_num);

static int sdl_keyboard_driver()
{
	while (SDL_PollEvent(&sdl_event))
	{
		if (sdl_event.type == SDL_QUIT)
			std::exit(0);

		if (sdl_event.type != SDL_KEYDOWN && sdl_event.type != SDL_KEYUP)
			continue;

		scratch2_uint = sdl_event.key.keysym.mod;
		scratch_uint = sdl_ascii_from_keysym(sdl_event.key.keysym);
		if (!scratch_uint || scratch_uint > 0x7F)
			scratch_uint = (unsigned int)sdl_event.key.keysym.sym;

		ref<short>(mem[0x4A6]) = 0x400 + 0x800*!!(scratch2_uint & KMOD_ALT) + 0x1000*!!(scratch2_uint & KMOD_SHIFT) + 0x2000*!!(scratch2_uint & KMOD_CTRL) + 0x4000*(sdl_event.type == SDL_KEYUP) + scratch_uint;
		pc_interrupt(7);
		return 1;
	}
	return 0;
}

namespace sdl_bridge
{
	bool has_window() { return (bool)sdl_window; }
	int poll_keyboard() { return sdl_keyboard_driver(); }
}
#endif

// Helper functions

// Set carry flag
static inline char set_CF(int new_CF)
{
	return regs8[idx(Flag::CF)] = !!new_CF;
}

// Set auxiliary flag
static inline char set_AF(int new_AF)
{
	return regs8[idx(Flag::AF)] = !!new_AF;
}

// Set overflow flag
static inline char set_OF(int new_OF)
{
	return regs8[idx(Flag::OF)] = !!new_OF;
}

// Set auxiliary and overflow flag after arithmetic operations
static inline char set_AF_OF_arith()
{
	set_AF((op_source ^= op_dest ^ op_result) & 0x10);
	if (op_result == op_dest)
		return set_OF(0);
	else
		return set_OF(1 & (regs8[idx(Flag::CF)] ^ (op_source >> (top_bit() - 1u))));
}

static inline int daa() noexcept
{
	scratch2_uint = regs8[idx(Reg8::AL)];
	const u8 old_al = (u8)scratch2_uint;
	const u8 old_cf = regs8[idx(Flag::CF)];

	set_AF(((old_al & 0x0F) > 9) || regs8[idx(Flag::AF)]);
	if (regs8[idx(Flag::AF)])
	{
		op_result = (regs8[idx(Reg8::AL)] += 6);
		set_CF(old_cf || (regs8[idx(Reg8::AL)] < old_al));
	}

	// Match original mask/min logic for DAA: mask=0xF0, min=0x90, and (mask&1)==0 -> uses current AL.
	const u8 mask_src = regs8[idx(Reg8::AL)];
	set_CF(((mask_src & 0xF0) > 0x90) || regs8[idx(Flag::CF)]);
	if (regs8[idx(Flag::CF)])	// set_CF just set it
		op_result = (regs8[idx(Reg8::AL)] += 0x60);

	return 0;
}

static inline int das() noexcept
{
	scratch2_uint = regs8[idx(Reg8::AL)];
	const u8 old_al = (u8)scratch2_uint;
	const u8 old_cf = regs8[idx(Flag::CF)];

	set_AF(((old_al & 0x0F) > 9) || regs8[idx(Flag::AF)]);
	if (regs8[idx(Flag::AF)])
	{
		op_result = (regs8[idx(Reg8::AL)] -= 6);
		set_CF(old_cf || (regs8[idx(Reg8::AL)] >= old_al));
	}

	// Match original mask/min logic for DAS: mask=0xFF, min=0x99, and (mask&1)==1 -> uses old AL.
	const u8 mask_src = old_al;
	set_CF(((mask_src & 0xFF) > 0x99) || regs8[idx(Flag::CF)]);
	if (regs8[idx(Flag::CF)])	// set_CF just set it
		op_result = (regs8[idx(Reg8::AL)] -= 0x60);

	return 0;
}

static inline int adc_op() noexcept
{
	OP(+= regs8[idx(Flag::CF)] +);
	set_CF(regs8[idx(Flag::CF)] && (op_result == op_dest) || (op_result < (int)op_dest));
	set_AF_OF_arith();
	return 0;
}

static inline int sbb_op() noexcept
{
	OP(-= regs8[idx(Flag::CF)] +);
	set_CF(regs8[idx(Flag::CF)] && (op_result == op_dest) || (-op_result < -(int)op_dest));
	set_AF_OF_arith();
	return 0;
}

// Assemble and return emulated CPU FLAGS register in scratch_uint
static inline void make_flags()
{
	scratch_uint = 0xF002; // 8086 has reserved and unused flags set to 1
	for (int i = 9; i--;)
		scratch_uint += regs8[idx(Flag::CF) + i] << bios_table_lookup[idx(BiosTable::FLAGS_BITFIELDS)][i];
}

// Set emulated CPU FLAGS register from regs8[FLAG_xx] values
static inline void set_flags(int new_flags)
{
	for (int i = 9; i--;)
		regs8[idx(Flag::CF) + i] = !!(1 << bios_table_lookup[idx(BiosTable::FLAGS_BITFIELDS)][i] & new_flags);
}

// Convert raw opcode to translated opcode index. This condenses a large number of different encodings of similar
// instructions into a much smaller number of distinct functions, which we then execute
static inline void set_opcode(unsigned char opcode)
{
	xlat_opcode_id = bios_table_lookup[idx(BiosTable::XLAT_OPCODE)][raw_opcode_id = opcode];
	extra = bios_table_lookup[idx(BiosTable::XLAT_SUBFUNCTION)][opcode];
	i_mod_size = bios_table_lookup[idx(BiosTable::I_MOD_SIZE)][opcode];
	set_flags_type = bios_table_lookup[idx(BiosTable::STD_FLAGS)][opcode];
}

// Execute INT #interrupt_num on the emulated machine
char pc_interrupt(unsigned char interrupt_num)
{
	set_opcode(0xCD); // Decode like INT

	make_flags();
	r_m_push(scratch_uint);
	r_m_push(regs16[idx(Reg16::CS)]);
	r_m_push(reg_ip);
	MEM_OP(REGS_BASE + 2 * idx(Reg16::CS), =, 4 * interrupt_num + 2);
	R_M_OP(reg_ip, =, mem[4 * interrupt_num]);

	return regs8[idx(Flag::TF)] = regs8[idx(Flag::IF)] = 0;
}

// AAA and AAS instructions - which_operation is +1 for AAA, and -1 for AAS
int AAA_AAS(char which_operation)
{
	return (regs16[idx(Reg16::AX)] += 262 * which_operation*set_AF(set_CF(((regs8[idx(Reg8::AL)] & 0x0F) > 9) || regs8[idx(Flag::AF)])), regs8[idx(Reg8::AL)] &= 0x0F);
}

#ifndef NO_GRAPHICS
void audio_callback(void *data, Uint8 *stream, int len)
{
	for (int i = 0; i < len; i++)
		stream[i] = (spkr_en == 3) && ref<unsigned short>(mem[0x4AA]) ? -((54 * wave_counter++ / ref<unsigned short>(mem[0x4AA])) & 1) : sdl_audio.silence;

	spkr_en = io_ports[0x61] & 3;
}
#endif

// Emulator entry point
int main(int argc, char **argv)
{
#ifndef NO_GRAPHICS
	// Initialise SDL (SDL2)
	SDL_Init(SDL_INIT_AUDIO | SDL_INIT_VIDEO);
	sdl_audio = SDL_AudioSpec{};
	sdl_audio.freq = 44100;
	sdl_audio.format = AUDIO_U8;
	sdl_audio.channels = 1;
	sdl_audio.samples = 512;
	sdl_audio.callback = audio_callback;
	sdl_audio_dev = SDL_OpenAudioDevice(nullptr, 0, &sdl_audio, nullptr, 0);
	if (sdl_audio_dev)
		SDL_PauseAudioDevice(sdl_audio_dev, 0);

	sdl_window.reset();
	sdl_renderer.reset();
	sdl_texture.reset();
#endif

	if (argc < 3)
	{
		std::fprintf(stderr, "Usage: %s bios fd.img [hd.img]\n", argv[0]);
		return 1;
	}

	// regs16 and reg8 point to F000:0, the start of memory-mapped registers. CS is initialised to F000
	regs8 = mem + REGS_BASE;
	regs16 = reinterpret_cast<u16 *>(regs8);
	regs16[idx(Reg16::CS)] = 0xF000;

	// Trap flag off
	regs8[idx(Flag::TF)] = 0;

	// Set DL equal to the boot device: 0 for the FD, or 0x80 for the HD. Normally, boot from the FD.
	// But, if the HD image file is prefixed with @, then boot from the HD
	regs8[idx(Reg8::DL)] = ((argc > 3) && (*argv[3] == '@')) ? argv[3]++, 0x80 : 0;

	// Open BIOS (file id disk[2]), floppy disk image (disk[1]), and hard disk image (disk[0]) if specified
	for (file_index = 3; file_index;)
	{
		const char *path = *++argv;
		if (!path)
		{
			disk[--file_index] = 0;
			continue;
		}

		int fd = platform::open_disk_image(path);
		disk[--file_index] = (fd < 0) ? 0 : fd;
	}

	// Set CX:AX equal to the hard disk image size, if present
	const std::int64_t disk_size_bytes = *disk ? platform::seek_end_bytes(*disk) : -1;
	ref<unsigned>(regs16[idx(Reg16::AX)]) = (disk_size_bytes > 0) ? (unsigned)(disk_size_bytes >> 9) : 0;

	// Load BIOS image into F000:0100, and set IP to 0100
	platform::read_fd(disk[2], regs8 + (reg_ip = 0x100), 0xFF00);

	// Load instruction decoding helper table
	for (int i = 0; i < 20; i++)
		for (int j = 0; j < 256; j++)
			bios_table_lookup[i][j] = regs8[regs16[0x81 + i] + j];

	// Instruction execution loop. Terminates if CS:IP = 0:0
	for (; opcode_stream = mem + 16 * regs16[idx(Reg16::CS)] + reg_ip, opcode_stream != mem;)
	{
		// Set up variables to prepare for decoding an opcode
		set_opcode(*opcode_stream);

		// Extract i_w and i_d fields from instruction
		i_w = (i_reg4bit = raw_opcode_id & 7) & 1;
		i_d = i_reg4bit / 2 & 1;

		// Extract instruction data fields
		i_data0 = ref<short>(opcode_stream[1]);
		i_data1 = ref<short>(opcode_stream[2]);
		i_data2 = ref<short>(opcode_stream[3]);

		// seg_override_en and rep_override_en contain number of instructions to hold segment override and REP prefix respectively
		if (seg_override_en)
			seg_override_en--;
		if (rep_override_en)
			rep_override_en--;

		// i_mod_size > 0 indicates that opcode uses i_mod/i_rm/i_reg, so decode them
		if (i_mod_size)
		{
			i_mod = (i_data0 & 0xFF) >> 6;
			i_rm = i_data0 & 7;
			i_reg = i_data0 / 8 & 7;

			if ((!i_mod && i_rm == 6) || (i_mod == 2))
				i_data2 = ref<short>(opcode_stream[4]);
			else if (i_mod != 1)
				i_data2 = i_data1;
			else // If i_mod is 1, operand is (usually) 8 bits rather than 16 bits
				i_data1 = (char)i_data1;

				decode_rm_reg();
		}

		// Instruction execution unit
		switch (xlat_opcode_id)
		{
			case 0: // Conditional jump (JAE, JNAE, etc.)
				// i_w is the invert flag, e.g. i_w == 1 means JNAE, whereas i_w == 0 means JAE 
				scratch_uchar = raw_opcode_id / 2 & 7;
				reg_ip += (char)i_data0 * (i_w ^ (regs8[bios_table_lookup[idx(BiosTable::COND_JUMP_DECODE_A)][scratch_uchar]] || regs8[bios_table_lookup[idx(BiosTable::COND_JUMP_DECODE_B)][scratch_uchar]] || regs8[bios_table_lookup[idx(BiosTable::COND_JUMP_DECODE_C)][scratch_uchar]] ^ regs8[bios_table_lookup[idx(BiosTable::COND_JUMP_DECODE_D)][scratch_uchar]]));
				break;
			case 1: // MOV reg, imm
				i_w = !!(raw_opcode_id & 8);
				R_M_OP(mem[get_reg_addr(i_reg4bit)], =, i_data0);
				break;
			case 3: // PUSH regs16
				r_m_push(regs16[i_reg4bit]);
				break;
			case 4: // POP regs16
				r_m_pop(regs16[i_reg4bit]);
				break;
			case 2: // INC|DEC regs16
				i_w = 1;
				i_d = 0;
				i_reg = i_reg4bit;
				decode_rm_reg();
				i_reg = extra;
					[[fallthrough]];
			case 5: // INC|DEC|JMP|CALL|PUSH
				if (i_reg < 2) // INC|DEC
					MEM_OP(op_from_addr, += 1 - 2 * i_reg +, REGS_BASE + 2 * idx(Reg16::ZERO)),
					op_source = 1,
					set_AF_OF_arith(),
					set_OF(op_dest + 1 - i_reg == (1u << (top_bit() - 1u))),
					(xlat_opcode_id == 5) && (set_opcode(0x10), 0); // Decode like ADC
				else if (i_reg != 6) // JMP|CALL
					i_reg - 3 || r_m_push(regs16[idx(Reg16::CS)]), // CALL (far)
					i_reg & 2 && r_m_push(reg_ip + 2 + i_mod*(i_mod != 3) + 2*(!i_mod && i_rm == 6)), // CALL (near or far)
					i_reg & 1 && (regs16[idx(Reg16::CS)] = ref<short>(mem[op_from_addr + 2])), // JMP|CALL (far)
					R_M_OP(reg_ip, =, mem[op_from_addr]),
					set_opcode(0x9A); // Decode like CALL
				else // PUSH
						r_m_push(mem[rm_addr]);
					break;
				case 6: // TEST r/m, imm16 / NOT|NEG|MUL|IMUL|DIV|IDIV reg
				op_to_addr = op_from_addr;

				switch (i_reg)
				{
						case 0: // TEST
						set_opcode(0x20); // Decode like AND
						reg_ip += i_w + 1;
							R_M_OP(mem[op_to_addr], &, i_data2);
							break;
						case 2: // NOT
							OP(=~);
							break;
						case 3: // NEG
						OP(=-);
						op_dest = 0;
						set_opcode(0x28); // Decode like SUB
							set_CF(op_result > op_dest);
							break;
						case 4: // MUL
							i_w ? mul_op<unsigned short>(regs16) : mul_op<unsigned char>(regs8);
							break;
						case 5: // IMUL
							i_w ? mul_op<short>(regs16) : mul_op<char>(regs8);
							break;
						case 6: // DIV
							i_w ? div_op<unsigned short, unsigned>(regs16) : div_op<unsigned char, unsigned short>(regs8);
							break;
						case 7: // IDIV
							i_w ? div_op<short, int>(regs16) : div_op<char, short>(regs8);
							break;
				}
					break;
				case 7: // ADD|OR|ADC|SBB|AND|SUB|XOR|CMP AL/AX, immed
					rm_addr = REGS_BASE;
					i_data2 = i_data0;
					i_mod = 3;
					i_reg = extra;
					reg_ip--;
					[[fallthrough]];
				case 8: // ADD|OR|ADC|SBB|AND|SUB|XOR|CMP reg, immed
				op_to_addr = rm_addr;
				regs16[idx(Reg16::SCRATCH)] = (i_d |= !i_w) ? (char)i_data2 : i_data2;
				op_from_addr = REGS_BASE + 2 * idx(Reg16::SCRATCH);
				reg_ip += !i_d + 1;
				set_opcode(0x08 * (extra = i_reg));
					[[fallthrough]];
				case 9: // ADD|OR|ADC|SBB|AND|SUB|XOR|CMP|MOV reg, r/m
				switch (extra)
				{
						case 0: // ADD
							OP(+=),
							set_CF(op_result < op_dest);
							break;
						case 1: // OR
							OP(|=);
							break;
						case 2: // ADC
							adc_op();
							break;
						case 3: // SBB
							sbb_op();
							break;
						case 4: // AND
							OP(&=);
							break;
						case 5: // SUB
							OP(-=),
							set_CF(op_result > op_dest);
							break;
						case 6: // XOR
							OP(^=);
							break;
						case 7: // CMP
							OP(-),
							set_CF(op_result > op_dest);
							break;
						case 8: // MOV
							OP(=);
							break;
				}
					break;
				case 10: // MOV sreg, r/m | POP r/m | LEA reg, r/m
				if (!i_w) // MOV
					i_w = 1,
					i_reg += 8,
					decode_rm_reg(),
					OP(=);
				else if (!i_d) // LEA
					seg_override_en = 1,
					seg_override = (u16)idx(Reg16::ZERO),
					decode_rm_reg(),
					R_M_OP(mem[op_from_addr], =, rm_addr);
				else // POP
						r_m_pop(mem[rm_addr]);
					break;
				case 11: // MOV AL/AX, [loc]
				i_mod = i_reg = 0;
				i_rm = 6;
				i_data1 = i_data0;
				decode_rm_reg();
					MEM_OP(op_from_addr, =, op_to_addr);
					break;
				case 12: // ROL|ROR|RCL|RCR|SHL|SHR|???|SAR reg/mem, 1/CL/imm (80186)
					scratch2_uint = sign_of(mem[rm_addr]);
					if (extra) // xxx reg/mem, imm
					{
						++reg_ip;
						scratch_uint = (char)i_data1;
					}
					else if (i_d) // xxx reg/mem, CL
						scratch_uint = 31 & regs8[idx(Reg8::CL)];
					else // xxx reg/mem, 1
						scratch_uint = 1;
				if (scratch_uint)
				{
					if (i_reg < 4) // Rotate operations
						scratch_uint %= i_reg / 2 + top_bit(),
						R_M_OP(scratch2_uint, =, mem[rm_addr]);
					if (i_reg & 1) // Rotate/shift right operations
						R_M_OP(mem[rm_addr], >>=, scratch_uint);
					else // Rotate/shift left operations
						R_M_OP(mem[rm_addr], <<=, scratch_uint);
					if (i_reg > 3) // Shift operations
						set_opcode(0x10); // Decode like ADC
					if (i_reg > 4) // SHR or SAR
						set_CF(op_dest >> (scratch_uint - 1) & 1);
				}

					switch (i_reg)
					{
						case 0: // ROL
							R_M_OP(mem[rm_addr], += , scratch2_uint >> (top_bit() - scratch_uint));
							set_OF(sign_of(op_result) ^ set_CF(op_result & 1));
							break;
						case 1: // ROR
							scratch2_uint &= (1 << scratch_uint) - 1;
							R_M_OP(mem[rm_addr], += , scratch2_uint << (top_bit() - scratch_uint));
							set_OF(sign_of(op_result * 2) ^ set_CF(sign_of(op_result)));
							break;
						case 2: // RCL
							R_M_OP(mem[rm_addr], += (regs8[idx(Flag::CF)] << (scratch_uint - 1)) + , scratch2_uint >> (1u + top_bit() - scratch_uint));
							set_OF(sign_of(op_result) ^ set_CF(scratch2_uint & (1u << (top_bit() - scratch_uint))));
							break;
						case 3: // RCR
							R_M_OP(mem[rm_addr], += (regs8[idx(Flag::CF)] << (top_bit() - scratch_uint)) + , scratch2_uint << (1u + top_bit() - scratch_uint));
							set_CF(scratch2_uint & 1 << (scratch_uint - 1));
							set_OF(sign_of(op_result) ^ sign_of(op_result * 2));
							break;
						case 4: // SHL
							set_OF(sign_of(op_result) ^ set_CF(sign_of(op_dest << (scratch_uint - 1))));
							break;
						case 5: // SHR
							set_OF(sign_of(op_dest));
							break;
						case 7: // SAR
							scratch_uint < top_bit() || set_CF(scratch2_uint);
							set_OF(0);
							R_M_OP(mem[rm_addr], +=, scratch2_uint *= ~(((1u << top_bit()) - 1u) >> scratch_uint));
							break;
					}
					break;
				case 13: // LOOPxx|JCZX
				scratch_uint = !!--regs16[idx(Reg16::CX)];

				switch(i_reg4bit)
				{
					case 0: // LOOPNZ
						scratch_uint &= !regs8[idx(Flag::ZF)];
						break;
					case 1: // LOOPZ
						scratch_uint &= regs8[idx(Flag::ZF)];
						break;
					case 3: // JCXXZ
						scratch_uint = !++regs16[idx(Reg16::CX)];
						break;
					default:
						break;
				}
				reg_ip += scratch_uint*(char)i_data0;
				break;
			case 14: // JMP | CALL short/near
				reg_ip += 3 - i_d;
				if (!i_w)
				{
					if (i_d) // JMP far
						reg_ip = 0,
						regs16[idx(Reg16::CS)] = i_data2;
					else // CALL
						r_m_push(reg_ip);
				}
				reg_ip += i_d && i_w ? (char)i_data0 : i_data0;
				break;
			case 15: // TEST reg, r/m
				MEM_OP(op_from_addr, &, op_to_addr);
				break;
			case 16: // XCHG AX, regs16
				i_w = 1;
				op_to_addr = REGS_BASE;
				op_from_addr = get_reg_addr(i_reg4bit);
				[[fallthrough]];
			case 24: // NOP|XCHG reg, r/m
				if (op_to_addr != op_from_addr)
					OP(^=),
					MEM_OP(op_from_addr, ^=, op_to_addr),
					OP(^=);
				break;
			case 17: // MOVSx (extra=0)|STOSx (extra=1)|LODSx (extra=2)
				scratch2_uint = seg_override_en ? seg_override : (u32)idx(Reg16::DS);

				for (scratch_uint = rep_override_en ? regs16[idx(Reg16::CX)] : 1; scratch_uint; scratch_uint--)
				{
					MEM_OP(extra < 2 ? segreg(idx(Reg16::ES), (u16)regs16[idx(Reg16::DI)]) : REGS_BASE, =, extra & 1 ? REGS_BASE : segreg(scratch2_uint, (u16)regs16[idx(Reg16::SI)])),
					extra & 1 || index_inc(idx(Reg16::SI)),
					extra & 2 || index_inc(idx(Reg16::DI));
				}

				if (rep_override_en)
					regs16[idx(Reg16::CX)] = 0;
				break;
			case 18: // CMPSx (extra=0)|SCASx (extra=1)
				scratch2_uint = seg_override_en ? seg_override : (u32)idx(Reg16::DS);

				if ((scratch_uint = rep_override_en ? regs16[idx(Reg16::CX)] : 1))
				{
					for (; scratch_uint; rep_override_en || scratch_uint--)
					{
						MEM_OP(extra ? REGS_BASE : segreg(scratch2_uint, (u16)regs16[idx(Reg16::SI)]), -, segreg(idx(Reg16::ES), (u16)regs16[idx(Reg16::DI)])),
						extra || index_inc(idx(Reg16::SI)),
						index_inc(idx(Reg16::DI)), rep_override_en && !(--regs16[idx(Reg16::CX)] && (op_result != rep_mode)) && (scratch_uint = 0);
					}

					set_flags_type = FLAGS_UPDATE_SZP | FLAGS_UPDATE_AO_ARITH; // Funge to set SZP/AO flags
					set_CF(op_result > op_dest);
				}
				break;
			case 19: // RET|RETF|IRET
				i_d = i_w;
				r_m_pop(reg_ip);
				if (extra) // IRET|RETF|RETF imm16
					r_m_pop(regs16[idx(Reg16::CS)]);
				if (extra & 2) // IRET
					set_flags(r_m_pop(scratch_uint));
				else if (!i_d) // RET|RETF imm16
					regs16[idx(Reg16::SP)] += i_data0;
				break;
			case 20: // MOV r/m, immed
				R_M_OP(mem[op_from_addr], =, i_data2);
				break;
			case 21: // IN AL/AX, DX/imm8
				io_ports[0x20] = 0; // PIC EOI
				io_ports[0x42] = --io_ports[0x40]; // PIT channel 0/2 read placeholder
				io_ports[0x3DA] ^= 9; // CGA refresh
				scratch_uint = extra ? regs16[idx(Reg16::DX)] : (unsigned char)i_data0;
				scratch_uint == 0x60 && (io_ports[0x64] = 0); // Scancode read flag
				scratch_uint == 0x3D5 && (io_ports[0x3D4] >> 1 == 7) && (io_ports[0x3D5] = ((mem[0x49E]*80 + mem[0x49D] + ref<short>(mem[0x4AD])) & (io_ports[0x3D4] & 1 ? 0xFF : 0xFF00)) >> (io_ports[0x3D4] & 1 ? 0 : 8)); // CRT cursor position
				R_M_OP(regs8[idx(Reg8::AL)], =, io_ports[scratch_uint]);
				break;
			case 22: // OUT DX/imm8, AL/AX
				scratch_uint = extra ? regs16[idx(Reg16::DX)] : (unsigned char)i_data0;
				R_M_OP(io_ports[scratch_uint], =, regs8[idx(Reg8::AL)]);
				scratch_uint == 0x61 && (io_hi_lo = 0, spkr_en |= regs8[idx(Reg8::AL)] & 3); // Speaker control
				(scratch_uint == 0x40 || scratch_uint == 0x42) && (io_ports[0x43] & 6) && (mem[0x469 + scratch_uint - (io_hi_lo ^= 1)] = regs8[idx(Reg8::AL)]); // PIT rate programming
#ifndef NO_GRAPHICS
				scratch_uint == 0x43 && (io_hi_lo = 0, regs8[idx(Reg8::AL)] >> 6 == 2) && (sdl_audio_dev ? (SDL_PauseAudioDevice(sdl_audio_dev, (regs8[idx(Reg8::AL)] & 0xF7) != 0xB6), 0) : 0); // Speaker enable
#endif
				scratch_uint == 0x3D5 && (io_ports[0x3D4] >> 1 == 6) && (mem[0x4AD + !(io_ports[0x3D4] & 1)] = regs8[idx(Reg8::AL)]); // CRT video RAM start offset
				scratch_uint == 0x3D5 && (io_ports[0x3D4] >> 1 == 7) && (scratch2_uint = ((mem[0x49E]*80 + mem[0x49D] + ref<short>(mem[0x4AD])) & (io_ports[0x3D4] & 1 ? 0xFF00 : 0xFF)) + (regs8[idx(Reg8::AL)] << (io_ports[0x3D4] & 1 ? 0 : 8)) - ref<short>(mem[0x4AD]), mem[0x49D] = scratch2_uint % 80, mem[0x49E] = scratch2_uint / 80); // CRT cursor position
				scratch_uint == 0x3B5 && io_ports[0x3B4] == 1 && (GRAPHICS_X = regs8[idx(Reg8::AL)] * 16); // Hercules resolution reprogramming. Defaults are set in the BIOS
				scratch_uint == 0x3B5 && io_ports[0x3B4] == 6 && (GRAPHICS_Y = regs8[idx(Reg8::AL)] * 4);
				break;
			case 23: // REPxx
				rep_override_en = 2;
				rep_mode = i_w;
				seg_override_en && seg_override_en++;
				break;
			case 25: // PUSH reg
				r_m_push(regs16[extra]);
				break;
			case 26: // POP reg
				r_m_pop(regs16[extra]);
				break;
			case 27: // xS: segment overrides
				seg_override_en = 2;
				seg_override = extra;
				rep_override_en && rep_override_en++;
				break;
			case 28: // DAA/DAS
				i_w = 0;
				extra ? das() : daa(); // extra = 0 for DAA, 1 for DAS
				break;
			case 29: // AAA/AAS
				op_result = AAA_AAS(extra - 1);
				break;
			case 30: // CBW
				regs8[idx(Reg8::AH)] = -sign_of(regs8[idx(Reg8::AL)]);
				break;
			case 31: // CWD
				regs16[idx(Reg16::DX)] = -sign_of(regs16[idx(Reg16::AX)]);
				break;
			case 32: // CALL FAR imm16:imm16
				r_m_push(regs16[idx(Reg16::CS)]);
				r_m_push(reg_ip + 5);
				regs16[idx(Reg16::CS)] = i_data2;
				reg_ip = i_data0;
				break;
			case 33: // PUSHF
				make_flags();
				r_m_push(scratch_uint);
				break;
			case 34: // POPF
				set_flags(r_m_pop(scratch_uint));
				break;
			case 35: // SAHF
				make_flags();
				set_flags((scratch_uint & 0xFF00) + regs8[idx(Reg8::AH)]);
				break;
			case 36: // LAHF
				make_flags(),
				regs8[idx(Reg8::AH)] = scratch_uint;
				break;
			case 37: // LES|LDS reg, r/m
				i_w = i_d = 1;
				decode_rm_reg();
				OP(=);
				MEM_OP(REGS_BASE + extra, =, rm_addr + 2);
				break;
			case 38: // INT 3
				++reg_ip;
				pc_interrupt(3);
				break;
			case 39: // INT imm8
				reg_ip += 2;
				pc_interrupt(i_data0);
				break;
			case 40: // INTO
				++reg_ip;
				regs8[idx(Flag::OF)] && pc_interrupt(4);
				break;
			case 41: // AAM
				if (i_data0 &= 0xFF)
					regs8[idx(Reg8::AH)] = regs8[idx(Reg8::AL)] / i_data0,
					op_result = regs8[idx(Reg8::AL)] %= i_data0;
				else // Divide by zero
					pc_interrupt(0);
				break;
			case 42: // AAD
				i_w = 0;
				regs16[idx(Reg16::AX)] = op_result = 0xFF & regs8[idx(Reg8::AL)] + i_data0 * regs8[idx(Reg8::AH)];
				break;
			case 43: // SALC
				regs8[idx(Reg8::AL)] = -regs8[idx(Flag::CF)];
				break;
			case 44: // XLAT
				regs8[idx(Reg8::AL)] = mem[segreg(seg_override_en ? (u32)seg_override : (u32)idx(Reg16::DS), (u16)(regs16[idx(Reg16::BX)] + regs8[idx(Reg8::AL)]))];
				break;
			case 45: // CMC
				regs8[idx(Flag::CF)] ^= 1;
				break;
			case 46: // CLC|STC|CLI|STI|CLD|STD
				regs8[extra / 2] = extra & 1;
				break;
			case 47: // TEST AL/AX, immed
				R_M_OP(regs8[idx(Reg8::AL)], &, i_data0);
				break;
			case 48: // Emulator-specific 0F xx opcodes
				switch ((char)i_data0)
				{
					case 0: // PUTCHAR_AL
						platform::write_fd(1, regs8, 1);
						break;
					case 1: // GET_RTC
						time(&clock_buf);
						ftime(&ms_clock);
						memcpy(mem + segreg(idx(Reg16::ES), (u16)regs16[idx(Reg16::BX)]), localtime(&clock_buf), sizeof(struct tm));
						ref<short>(mem[segreg(idx(Reg16::ES), (u16)(regs16[idx(Reg16::BX)] + 36))]) = ms_clock.millitm;
						break;
					case 2: // DISK_READ
					case 3: // DISK_WRITE
					{
						int fd = disk[regs8[idx(Reg8::DL)]];
						unsigned int lba_offset = (unsigned int)regs16[idx(Reg16::BP)] << 9;
						unsigned int byte_count = regs16[idx(Reg16::AX)];
						u8 *buf = mem + segreg(idx(Reg16::ES), (u16)regs16[idx(Reg16::BX)]);
						const bool is_write = ((char)i_data0 == 3);
						const std::int64_t rv = platform::rw_at_offset(fd, buf, (std::size_t)byte_count, (std::uint64_t)lba_offset, is_write);
						regs8[idx(Reg8::AL)] = (rv < 0) ? 0 : (u8)rv;
					}
						break;
					default:
						break;
				}
				break;
		}

		// Increment instruction pointer by computed instruction length. Tables in the BIOS binary
		// help us here.
		reg_ip += (i_mod*(i_mod != 3) + 2*(!i_mod && i_rm == 6))*i_mod_size + bios_table_lookup[idx(BiosTable::BASE_INST_SIZE)][raw_opcode_id] + bios_table_lookup[idx(BiosTable::I_W_SIZE)][raw_opcode_id]*(i_w + 1);

		// If instruction needs to update SF, ZF and PF, set them as appropriate
		if (set_flags_type & FLAGS_UPDATE_SZP)
		{
			regs8[idx(Flag::SF)] = sign_of(op_result);
			regs8[idx(Flag::ZF)] = !op_result;
			regs8[idx(Flag::PF)] = bios_table_lookup[idx(BiosTable::PARITY_FLAG)][(unsigned char)op_result];

			// If instruction is an arithmetic or logic operation, also set AF/OF/CF as appropriate.
			if (set_flags_type & FLAGS_UPDATE_AO_ARITH)
				set_AF_OF_arith();
			if (set_flags_type & FLAGS_UPDATE_OC_LOGIC)
				set_CF(0), set_OF(0);
		}

		// Poll timer/keyboard every kKeyboardTimerUpdateDelay instructions
		if (!(++inst_counter % kKeyboardTimerUpdateDelay))
			int8_asap = 1;

#ifndef NO_GRAPHICS
		// Update the video graphics display every kGraphicsUpdateDelay instructions
		if (!(inst_counter % kGraphicsUpdateDelay))
		{
			// Video card in graphics mode?
			if (io_ports[0x3B8] & 2)
			{
				// If we don't already have an SDL window open, set it up and compute color and video memory translation tables
						if (!sdl_window)
				{
					for (int i = 0; i < 16; i++)
						pixel_colors[i] = mem[0x4AC] ? // CGA?
							cga_colors[(i & 12) >> 2] + (cga_colors[i & 3] << 16) // CGA -> RGB332
							: 0xFF*(((i & 1) << 24) + ((i & 2) << 15) + ((i & 4) << 6) + ((i & 8) >> 3)); // Hercules -> RGB332

					for (int i = 0; i < GRAPHICS_X * GRAPHICS_Y / 4; i++)
						vid_addr_lookup[i] = i / GRAPHICS_X * (GRAPHICS_X / 8) + (i / 2) % (GRAPHICS_X / 8) + 0x2000*(mem[0x4AC] ? (2 * i / GRAPHICS_X) % 2 : (4 * i / GRAPHICS_X) % 4);

							sdl_window.reset(SDL_CreateWindow("8086tiny", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, GRAPHICS_X, GRAPHICS_Y, 0));
							sdl_renderer.reset(sdl_window ? SDL_CreateRenderer(sdl_window.get(), -1, SDL_RENDERER_ACCELERATED) : nullptr);
							sdl_texture.reset(sdl_renderer ? SDL_CreateTexture(sdl_renderer.get(), SDL_PIXELFORMAT_RGB332, SDL_TEXTUREACCESS_STREAMING, GRAPHICS_X, GRAPHICS_Y) : nullptr);
							sdl_pixels.assign((size_t)GRAPHICS_X * (size_t)GRAPHICS_Y / 4u, 0u);
				}

				// Refresh SDL display from emulated graphics card video RAM
				vid_mem_base = mem + 0xB0000 + 0x8000*(mem[0x4AC] ? 1 : io_ports[0x3B8] >> 7); // B800:0 for CGA/Hercules bank 2, B000:0 for Hercules bank 1
						if (sdl_texture && !sdl_pixels.empty())
				{
						auto *dst = sdl_pixels.data();
							const int quad_count = GRAPHICS_X * GRAPHICS_Y / 4;
							for (int i = 0; i < quad_count; i++)
						dst[i] = pixel_colors[15 & (vid_mem_base[vid_addr_lookup[i]] >> 4*!(i & 1))];

							SDL_UpdateTexture(sdl_texture.get(), nullptr, sdl_pixels.data(), GRAPHICS_X);
							SDL_RenderClear(sdl_renderer.get());
							SDL_RenderCopy(sdl_renderer.get(), sdl_texture.get(), nullptr, nullptr);
							SDL_RenderPresent(sdl_renderer.get());
				}
			}
			else if (sdl_window) // Application has gone back to text mode, so close the SDL window
			{
						sdl_texture.reset();
						sdl_renderer.reset();
						sdl_window.reset();
				sdl_pixels.clear();
			}
			SDL_PumpEvents();
		}
#endif

		// Application has set trap flag, so fire INT 1
		if (trap_flag)
			pc_interrupt(1);

		trap_flag = regs8[idx(Flag::TF)];

		// If a timer tick is pending, interrupts are enabled, and no overrides/REP are active,
		// then process the tick and check for new keystrokes
		if (int8_asap && !seg_override_en && !rep_override_en && regs8[idx(Flag::IF)] && !regs8[idx(Flag::TF)])
			pc_interrupt(0xA), int8_asap = 0, poll_active_keyboard();
	}

#ifndef NO_GRAPHICS
	if (sdl_audio_dev)
		SDL_CloseAudioDevice(sdl_audio_dev);
	sdl_texture.reset();
	sdl_renderer.reset();
	sdl_window.reset();
	SDL_Quit();
#endif

	for (int i = 0; i < 3; ++i)
		if (disk[i] > 0)
			platform::close_fd(disk[i]);
	return 0;
}
