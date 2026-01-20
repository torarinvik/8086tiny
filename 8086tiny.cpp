// 8086tiny: a tiny, highly functional, highly portable PC emulator/VM
// Copyright 2013-14, Adrian Cable (adrian.cable@gmail.com) - http://www.megalith.co.uk/8086tiny
//
// Revision 1.25
#include <ctime>
#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <cstdint>
#include <array>
#include <cctype>
#include <string>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <random>
#include <csignal>
#include <cerrno>
#include <filesystem>
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
constexpr std::size_t PHYS_MEM_SIZE = 0x100000; // 20-bit, 1MB physical address space

// Graphics/timer/keyboard update delays (explained later)
#ifndef GRAPHICS_UPDATE_DELAY
#define GRAPHICS_UPDATE_DELAY 360000
#endif
constexpr u32 kGraphicsUpdateDelay = (u32)GRAPHICS_UPDATE_DELAY;
constexpr u32 kKeyboardTimerUpdateDelay = 20000;
constexpr u32 kExploreWarmupKeyboardTimerUpdateDelay = 2000;

#ifndef SNAP_AT
#define SNAP_AT 0
#endif

#ifndef TRACE_FROM
#define TRACE_FROM 0u
#endif

#ifndef TRACE_LEN
#define TRACE_LEN 0u
#endif

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

// Opcode groups for the main execution switch.
// NOTE: The underlying values MUST match the BIOS XLAT table output (0..48).
enum class OpcodeGroup : u8
{
	COND_JUMP,
	MOV_REG_IMM,
	INC_DEC_REG16,
	PUSH_REG16,
	POP_REG16,
	GRP_INC_DEC_JMP_CALL_PUSH,
	GRP_TEST_NOT_NEG_MUL_DIV,
	ALU_AL_AX_IMM,
	ALU_REG_IMM,
	ALU_REG_RM,
	MOV_SREG_POP_LEA,
	MOV_AL_AX_LOC,
	GRP_BITWISE,
	GRP_LOOP,
	JMP_CALL_SHORT_NEAR,
	TEST_REG_RM,
	XCHG_AX_REG16,
	GRP_MOVS_STOS_LODS,
	GRP_CMPS_SCAS,
	RET_RETF_IRET,
	MOV_RM_IMM,
	IN_AL_AX,
	OUT_AL_AX,
	REPXX,
	NOP_XCHG_REG_RM,
	PUSH_REG,
	POP_REG,
	SEGMENT_OVERRIDE,
	DAA_DAS,
	AAA_AAS,
	CBW,
	CWD,
	CALL_FAR_IMM,
	PUSHF,
	POPF,
	SAHF,
	LAHF,
	LES_LDS_REG_RM,
	INT_3,
	INT_IMM8,
	INTO,
	AAM,
	AAD,
	SALC,
	XLAT,
	CMC,
	CLC_STC_CLI_STI_CLD_STD,
	TEST_AL_AX_IMM,
	EMULATOR_SPECIFIC
};

static_assert(to_underlying(OpcodeGroup::COND_JUMP) == 0);
// Guardrails: BIOS XLAT translates raw opcodes into an OpcodeGroup id; these asserts ensure
// the enum stays in sync with that 0..48 table mapping even if items are reordered/inserted.
static_assert(to_underlying(OpcodeGroup::ALU_REG_RM) == 9);
static_assert(to_underlying(OpcodeGroup::GRP_BITWISE) == 12);
static_assert(to_underlying(OpcodeGroup::REPXX) == 23);
static_assert(to_underlying(OpcodeGroup::INT_IMM8) == 39);
static_assert(to_underlying(OpcodeGroup::EMULATOR_SPECIFIC) == 48);

// ALU operations (for groups 7, 8, 9)
enum class AluOp : int
{
	ADD = 0,
	OR = 1,
	ADC = 2,
	SBB = 3,
	AND = 4,
	SUB = 5,
	XOR = 6,
	CMP = 7,
	MOV = 8
};

// Group 5 operations (INC, DEC, JMP, CALL, PUSH)
enum class Group5Op : int
{
	INC = 0,
	DEC = 1,
	CALL_NEAR = 2,
	CALL_FAR = 3,
	JMP_NEAR = 4,
	JMP_FAR = 5,
	PUSH = 6
};

// Group 6 operations (TEST, NOT, NEG, MUL, IMUL, DIV, IDIV)
enum class Group6Op : int
{
	TEST = 0,
	NOT = 2,
	NEG = 3,
	MUL = 4,
	IMUL = 5,
	DIV = 6,
	IDIV = 7
};

// Bitwise operations (Group 12)
enum class BitwiseOp : int
{
	ROL = 0,
	ROR = 1,
	RCL = 2,
	RCR = 3,
	SHL = 4,
	SHR = 5,
	SAR = 7
};

// Loop operations (Group 13)
enum class LoopOp : int
{
	LOOPNZ = 0,
	LOOPZ = 1,
	JCXZ = 3
};

// Emulator-specific operations (Group 48)
enum class EmulatorOp : int
{
	PUTCHAR_AL = 0,
	GET_RTC = 1,
	DISK_READ = 2,
	DISK_WRITE = 3
};

enum class BitSize : u8
{
	Bit8 = 8,
	Bit16 = 16
};

enum class OpDirection : u8
{
	ToRM = 0,
	FromRM = 1,
	None = 2
};

enum class OpForm : u8
{
	None = 0,
	RegRm = 1,
	ImmToRm = 2,
	AccImm = 3
};

struct OpView
{
	BitSize size;
	OpForm form;
	OpDirection direction;
};

constexpr BitSize bit_size_from_w(u8 w) noexcept
{
	return w ? BitSize::Bit16 : BitSize::Bit8;
}

constexpr OpDirection op_direction_from_d(u8 d) noexcept
{
	return d ? OpDirection::FromRM : OpDirection::ToRM;
}

constexpr OpView op_view_for(OpcodeGroup group, u8 w, u8 d) noexcept
{
	const BitSize size = bit_size_from_w(w);
	switch (group)
	{
		case OpcodeGroup::ALU_REG_RM:
			return OpView{size, OpForm::RegRm, op_direction_from_d(d)};
		case OpcodeGroup::ALU_REG_IMM:
			return OpView{size, OpForm::ImmToRm, OpDirection::None};
		case OpcodeGroup::ALU_AL_AX_IMM:
			return OpView{size, OpForm::AccImm, OpDirection::None};
		default:
			return OpView{size, OpForm::None, OpDirection::None};
	}
}

constexpr int idx(OpcodeGroup g) noexcept { return to_underlying(g); }
constexpr int idx(AluOp o) noexcept { return to_underlying(o); }
constexpr int idx(Group5Op o) noexcept { return to_underlying(o); }
constexpr int idx(Group6Op o) noexcept { return to_underlying(o); }
constexpr int idx(BitwiseOp o) noexcept { return to_underlying(o); }
constexpr int idx(LoopOp o) noexcept { return to_underlying(o); }
constexpr int idx(EmulatorOp o) noexcept { return to_underlying(o); }

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
extern bool g_scripted_input;

static inline int poll_console_keyboard()
{
	// In headless exploration runs with scripted input, never block on stdin.
	if (g_scripted_input)
		return 0;
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

// --- Exploration instrumentation (enabled only when flags are provided) ---

struct InstrInfo
{
	u32 exec_count = 0;
	u16 variant_mismatches = 0;
	u8 len = 0;
	std::array<u8, 15> bytes{};
};

struct DecisionPoint
{
	u32 inst_index = 0;
	u16 cs = 0;
	u16 ip = 0;
	u32 phys = 0;
	u8 ah = 0;
};

static std::vector<InstrInfo> g_instr_map; // indexed by 20-bit physical address
static std::vector<DecisionPoint> g_decision_points;
static const char *g_dump_instr_path = nullptr;
static const char *g_dump_decisions_path = nullptr;
static u32 g_max_instructions = 0; // 0 = unlimited

static bool g_track_coverage = false;
static std::vector<u8> g_global_coverage; // 1MB
static std::vector<u8> g_run_coverage;    // 1MB
static std::vector<u32> g_run_touched;
static u32 g_last_run_gain = 0;

static bool g_explore_mode = false;
static u32 g_explore_iters = 1000;
static u32 g_explore_seed = 1;
static u32 g_explore_max_keys = 64;
static u32 g_explore_corpus_max = 256;

static bool g_quiet_output = false;
static bool g_quiet_output_user_set = false;
static bool g_debug_diskio = false;
static u32 g_debug_diskio_left = 0;
static bool g_scripted_input_user_set = false;

// Diagnostics/progress
static u32 g_progress_every_inst = 0; // 0 disables periodic progress
static u32 g_tty_limit_bytes = 0;     // 0 = unlimited; applies when not quiet
static std::uint64_t g_tty_bytes_written = 0;
static bool g_tty_limit_tripped = false;
static std::uint64_t g_disk_reads = 0;
static std::uint64_t g_disk_writes = 0;
static u32 g_last_diskio_inst = 0;
static u16 g_progress_last_cs = 0xFFFFu;
static u16 g_progress_last_ip = 0xFFFFu;
static u32 g_progress_same_csip = 0;
static bool g_progress_stuck_dumped = false;
static bool g_explore_warmup_active = false;
static bool g_explore_stop_requested = false;
static bool g_explore_prompt_seen = false;
static u32 g_explore_warmup_inst = 2000000; // initial warmup budget; may auto-extend unless explicitly set
static bool g_explore_warmup_inst_user_set = false;
static std::string g_explore_tty_tail;

static const char *g_explore_best_keys_path = nullptr;
static u32 g_explore_save_every = 200; // iterations; 0 disables periodic save
static u32 g_explore_dump_every = 0;   // iterations; 0 disables periodic dump

static const char *g_run_exe_host_path = nullptr;
static std::string g_fd_prepared_path;
static bool g_audio_enabled = true;
static bool g_audio_user_set = false;

static void preparse_early_audio_flags(int argc, char **argv)
{
	bool want_audio = false;
	bool want_no_audio = false;
	bool saw_explore = false;
	for (int i = 1; i < argc; ++i)
	{
		const char *arg = argv[i];
		if (!arg)
			continue;
		if (!std::strcmp(arg, "--explore"))
			saw_explore = true;
		if (!std::strcmp(arg, "--no-audio") || !std::strcmp(arg, "--mute"))
			want_no_audio = true;
		if (!std::strcmp(arg, "--audio"))
			want_audio = true;
	}
	if (want_no_audio)
	{
		g_audio_enabled = false;
		g_audio_user_set = true;
		return;
	}
	if (want_audio)
	{
		g_audio_enabled = true;
		g_audio_user_set = true;
		return;
	}
	// Default to silent in explore mode unless user opted in.
	if (saw_explore)
		g_audio_enabled = false;
}

static volatile std::sig_atomic_t g_host_stop_signal = 0;

static void on_host_signal(int sig)
{
	g_host_stop_signal = sig;
}

bool g_scripted_input = false;
std::vector<u16> g_scripted_keys;
std::size_t g_scripted_key_pos = 0;

static std::string trim(std::string s)
{
	const auto not_space = [](unsigned char c) { return !std::isspace(c); };
	s.erase(s.begin(), std::find_if(s.begin(), s.end(), not_space));
	s.erase(std::find_if(s.rbegin(), s.rend(), not_space).base(), s.end());
	return s;
}

static std::string upper_ascii(std::string s)
{
	for (char &c : s)
		c = (char)std::toupper((unsigned char)c);
	return s;
}

static bool token_is_banned_gorilla_key(const std::string &tok_upper)
{
	// Hard bans (quit + emulator/UI hotkeys).
	if (tok_upper == "N" || tok_upper == "QUIT")
		return true;
	if (tok_upper.find("ALT") != std::string::npos)
		return true;
	return false;
}

static bool try_parse_gorilla_ascii_token(const std::string &token, u8 &out_ascii)
{
	std::string tok = upper_ascii(trim(token));
	if (tok.empty())
		return false;

	if (token_is_banned_gorilla_key(tok))
		return false;

	if (tok.size() == 1)
	{
		const char c = tok[0];
		if (c >= '0' && c <= '9')
		{
			out_ascii = (u8)c;
			return true;
		}
		if (c >= 'A' && c <= 'Z' && c != 'N')
		{
			out_ascii = (u8)c;
			return true;
		}
		if (c == ':' || c == '\\' || c == '.' || c == '_')
		{
			out_ascii = (u8)c;
			return true;
		}
		return false;
	}

	if (tok == "ENTER" || tok == "RETURN")
	{
		out_ascii = 0x0D;
		return true;
	}
	if (tok == "BACKSPACE" || tok == "BS")
	{
		out_ascii = 0x08;
		return true;
	}
	if (tok == "SPACE")
	{
		out_ascii = (u8)' ';
		return true;
	}
	if (tok == "Y")
	{
		out_ascii = (u8)'Y';
		return true;
	}

	return false;
}

static std::string token_from_ascii(u8 c)
{
	if (c == 0x0D)
		return "ENTER";
	if (c == 0x08)
		return "BS";
	if (c == (u8)' ')
		return "SPACE";
	if ((c >= (u8)'0' && c <= (u8)'9') || (c >= (u8)'A' && c <= (u8)'Z') || c == (u8)':' || c == (u8)'\\' || c == (u8)'.' || c == (u8)'_')
		return std::string(1, (char)c);
	// Fallback: hex byte.
	char buf[8] = {0};
	std::snprintf(buf, sizeof(buf), "0x%02X", (unsigned)c);
	return std::string(buf);
}

static bool write_text_file_atomic(const char *path, const std::string &contents)
{
	if (!path || !*path)
		return false;
	const std::string tmp = std::string(path) + ".tmp";
	{
		std::ofstream out(tmp, std::ios::binary);
		if (!out)
			return false;
		out.write(contents.data(), (std::streamsize)contents.size());
		if (!out)
			return false;
	}
	if (std::rename(tmp.c_str(), path) != 0)
	{
		std::remove(tmp.c_str());
		return false;
	}
	return true;
}

static bool save_keys_file(const char *path, const std::vector<u8> &ascii)
{
	std::string line;
	line.reserve(ascii.size() * 2u + 16u);
	for (std::size_t i = 0; i < ascii.size(); ++i)
	{
		if (i)
			line.push_back(',');
		line += token_from_ascii(ascii[i]);
	}
	line.push_back('\n');
	return write_text_file_atomic(path, line);
}

static bool copy_file_binary(const char *src, const char *dst)
{
	std::ifstream in(src, std::ios::binary);
	if (!in)
		return false;
	std::ofstream out(dst, std::ios::binary | std::ios::trunc);
	if (!out)
		return false;
	out << in.rdbuf();
	return (bool)out;
}

static inline u16 rd16(const u8 *p) noexcept { return (u16)p[0] | ((u16)p[1] << 8); }
static inline u32 rd32(const u8 *p) noexcept { return (u32)p[0] | ((u32)p[1] << 8) | ((u32)p[2] << 16) | ((u32)p[3] << 24); }

static u16 fat12_get(const std::vector<u8> &fat, u32 cluster)
{
	const u32 off = (cluster * 3u) / 2u;
	if (off + 1u >= fat.size())
		return 0xFFF;
	const u16 v = (u16)fat[off] | ((u16)fat[off + 1u] << 8);
	return (cluster & 1u) ? (u16)(v >> 4) : (u16)(v & 0x0FFFu);
}

static void fat12_set(std::vector<u8> &fat, u32 cluster, u16 value)
{
	value &= 0x0FFFu;
	const u32 off = (cluster * 3u) / 2u;
	if (off + 1u >= fat.size())
		return;
	const u16 cur = (u16)fat[off] | ((u16)fat[off + 1u] << 8);
	const u16 next = (cluster & 1u)
		? (u16)((cur & 0x000Fu) | (value << 4))
		: (u16)((cur & 0xF000u) | value);
	fat[off] = (u8)(next & 0xFFu);
	fat[off + 1u] = (u8)((next >> 8) & 0xFFu);
}

static bool make_dos_83_upper(const std::string &filename, char out_name[11])
{
	std::memset(out_name, ' ', 11);
	std::string base = filename;
	// strip any directory
	const std::size_t slash = base.find_last_of("/\\");
	if (slash != std::string::npos)
		base = base.substr(slash + 1);
	// split ext
	std::string stem, ext;
	const std::size_t dot = base.find_last_of('.');
	if (dot != std::string::npos)
	{
		stem = base.substr(0, dot);
		ext = base.substr(dot + 1);
	}
	else
	{
		stem = base;
	}
	if (stem.empty())
		return false;
	auto norm = [](char c) -> char {
		c = (char)std::toupper((unsigned char)c);
		if ((c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_' || c == '$' || c == '~')
			return c;
		return '_';
	};
	for (std::size_t i = 0; i < stem.size() && i < 8u; ++i)
		out_name[i] = norm(stem[i]);
	for (std::size_t i = 0; i < ext.size() && i < 3u; ++i)
		out_name[8u + i] = norm(ext[i]);
	return true;
}

// Inject a host file into the root directory of a FAT12 floppy image.
// Supports classic FreeDOS boot floppies (FAT12, root dir, no subdirs).
static bool fat12_inject_root_file(const char *img_path, const char *host_path, const char *dest_83)
{
	std::ifstream host(host_path, std::ios::binary);
	if (!host)
	{
		std::fprintf(stderr, "--run-exe: failed to open host file: %s\n", host_path);
		return false;
	}
	std::vector<u8> file((std::istreambuf_iterator<char>(host)), std::istreambuf_iterator<char>());
	if (file.empty())
	{
		std::fprintf(stderr, "--run-exe: host file is empty: %s\n", host_path);
		return false;
	}

	std::fstream img(img_path, std::ios::in | std::ios::out | std::ios::binary);
	if (!img)
	{
		std::fprintf(stderr, "--run-exe: failed to open disk image for writing: %s\n", img_path);
		return false;
	}

	// Read boot sector
	std::vector<u8> boot(512);
	img.seekg(0);
	img.read((char *)boot.data(), (std::streamsize)boot.size());
	if (!img)
		return false;

	const u16 bytes_per_sector = rd16(&boot[11]);
	const u8 sectors_per_cluster = boot[13];
	const u16 reserved_sectors = rd16(&boot[14]);
	const u8 num_fats = boot[16];
	const u16 root_entry_count = rd16(&boot[17]);
	const u16 total_sectors16 = rd16(&boot[19]);
	const u16 fat_size16 = rd16(&boot[22]);
	const u32 total_sectors = total_sectors16 ? (u32)total_sectors16 : rd32(&boot[32]);
	if (bytes_per_sector == 0 || sectors_per_cluster == 0 || reserved_sectors == 0 || num_fats == 0 || fat_size16 == 0 || root_entry_count == 0 || total_sectors == 0)
	{
		std::fprintf(stderr, "--run-exe: unsupported/invalid BPB in %s\n", img_path);
		return false;
	}
	const u32 root_dir_sectors = (u32)((root_entry_count * 32u + bytes_per_sector - 1u) / bytes_per_sector);
	const u32 fat_bytes = (u32)fat_size16 * (u32)bytes_per_sector;
	const u32 fat0_offset = (u32)reserved_sectors * (u32)bytes_per_sector;
	const u32 root_offset = (u32)(reserved_sectors + (u32)num_fats * (u32)fat_size16) * (u32)bytes_per_sector;
	const u32 data_offset = (u32)(reserved_sectors + (u32)num_fats * (u32)fat_size16 + root_dir_sectors) * (u32)bytes_per_sector;
	const u32 cluster_bytes = (u32)sectors_per_cluster * (u32)bytes_per_sector;
	const u32 data_sectors = total_sectors - (u32)reserved_sectors - (u32)num_fats * (u32)fat_size16 - root_dir_sectors;
	const u32 cluster_count = data_sectors / sectors_per_cluster;
	if (cluster_count < 4)
		return false;

	// Load FAT0
	std::vector<u8> fat(fat_bytes);
	img.seekg((std::streamoff)fat0_offset);
	img.read((char *)fat.data(), (std::streamsize)fat.size());
	if (!img)
		return false;

	// Load root directory
	std::vector<u8> root(root_dir_sectors * (u32)bytes_per_sector);
	img.seekg((std::streamoff)root_offset);
	img.read((char *)root.data(), (std::streamsize)root.size());
	if (!img)
		return false;

	char name83[11];
	if (dest_83 && *dest_83)
	{
		if (!make_dos_83_upper(dest_83, name83))
			return false;
	}
	else
	{
		if (!make_dos_83_upper(std::filesystem::path(host_path).filename().string(), name83))
			return false;
	}

	// Find existing file or a free directory entry.
	int free_entry = -1;
	for (u32 i = 0; i + 32u <= root.size(); i += 32u)
	{
		const u8 first = root[i];
		if (first == 0x00 || first == 0xE5)
		{
			if (free_entry < 0)
				free_entry = (int)i;
			if (first == 0x00)
				break;
			continue;
		}
		if (std::memcmp(&root[i], name83, 11) == 0)
		{
			std::fprintf(stderr, "--run-exe: file already exists in image: %.8s.%.3s\n", name83, name83 + 8);
			return false;
		}
	}
	if (free_entry < 0)
	{
		std::fprintf(stderr, "--run-exe: no free root dir entries\n");
		return false;
	}

	// Allocate clusters
	const u32 need_clusters = (u32)((file.size() + cluster_bytes - 1u) / cluster_bytes);
	std::vector<u16> chain;
	chain.reserve(need_clusters);
	for (u32 c = 2; c < 2 + cluster_count && chain.size() < need_clusters; ++c)
	{
		if (fat12_get(fat, c) == 0)
			chain.push_back((u16)c);
	}
	if (chain.size() != need_clusters)
	{
		std::fprintf(stderr, "--run-exe: not enough free clusters (%u needed)\n", (unsigned)need_clusters);
		return false;
	}
	for (u32 i = 0; i < chain.size(); ++i)
	{
		const u16 cur = chain[i];
		const u16 next = (i + 1u < chain.size()) ? chain[i + 1u] : (u16)0xFFFu;
		fat12_set(fat, cur, next);
	}

	// Write file data clusters
	std::size_t written = 0;
	for (u32 i = 0; i < chain.size(); ++i)
	{
		const u32 cluster = (u32)chain[i];
		const u32 off = data_offset + (cluster - 2u) * cluster_bytes;
		const std::size_t chunk = std::min<std::size_t>(cluster_bytes, file.size() - written);
		img.seekp((std::streamoff)off);
		img.write((const char *)(file.data() + written), (std::streamsize)chunk);
		if (!img)
			return false;
		if (chunk < cluster_bytes)
		{
			std::vector<u8> zeros(cluster_bytes - chunk, 0);
			img.write((const char *)zeros.data(), (std::streamsize)zeros.size());
			if (!img)
				return false;
		}
		written += chunk;
	}

	// Write FATs back
	for (u8 fi = 0; fi < num_fats; ++fi)
	{
		const u32 off = fat0_offset + (u32)fi * fat_bytes;
		img.seekp((std::streamoff)off);
		img.write((const char *)fat.data(), (std::streamsize)fat.size());
		if (!img)
			return false;
	}

	// Write directory entry
	u8 entry[32] = {0};
	std::memcpy(entry, name83, 11);
	entry[11] = 0x20; // archive
	// time/date left as 0
	const u16 first_cluster = chain.empty() ? 0 : chain.front();
	entry[26] = (u8)(first_cluster & 0xFFu);
	entry[27] = (u8)((first_cluster >> 8) & 0xFFu);
	const u32 file_size = (u32)file.size();
	entry[28] = (u8)(file_size & 0xFFu);
	entry[29] = (u8)((file_size >> 8) & 0xFFu);
	entry[30] = (u8)((file_size >> 16) & 0xFFu);
	entry[31] = (u8)((file_size >> 24) & 0xFFu);
	std::memcpy(&root[(std::size_t)free_entry], entry, 32);
	img.seekp((std::streamoff)root_offset);
	img.write((const char *)root.data(), (std::streamsize)root.size());
	if (!img)
		return false;

	return true;
}

static inline void explore_observe_tty_char(u8 ch)
{
	if (!g_explore_mode || !g_explore_warmup_active)
		return;
	if (!ch)
		return;
	g_explore_tty_tail.push_back((char)ch);
	if (g_explore_tty_tail.size() > 512u)
		g_explore_tty_tail.erase(0, g_explore_tty_tail.size() - 256u);

	// Heuristic: prompt often contains something like "C:\\...>" or "A:>".
	if (ch == (u8)'>')
	{
		const std::size_t n = g_explore_tty_tail.size();
		if (n >= 4)
		{
			const char a = g_explore_tty_tail[n - 4];
			const char b = g_explore_tty_tail[n - 3];
			const char c = g_explore_tty_tail[n - 2];
			// Matches "C:\\>"
			if (std::isalpha((unsigned char)a) && b == ':' && c == '\\')
			{
				g_explore_prompt_seen = true;
				g_explore_stop_requested = true;
				return;
			}
		}
		if (n >= 3)
		{
			const char a = g_explore_tty_tail[n - 3];
			const char b = g_explore_tty_tail[n - 2];
			// Matches "A:>" (some shells / minimal prompts)
			if (std::isalpha((unsigned char)a) && b == ':')
			{
				g_explore_prompt_seen = true;
				g_explore_stop_requested = true;
				return;
			}
		}
	}
}

static inline void maybe_print_progress(u32 inst_counter)
{
	if (!g_progress_every_inst)
		return;

	const u16 cs = regs16[idx(Reg16::CS)];
	const u16 ip = reg_ip;
	if (cs == g_progress_last_cs && ip == g_progress_last_ip)
		++g_progress_same_csip;
	else
	{
		g_progress_last_cs = cs;
		g_progress_last_ip = ip;
		g_progress_same_csip = 0;
		g_progress_stuck_dumped = false;
	}

	if (inst_counter && !(inst_counter % g_progress_every_inst))
	{
		std::fprintf(stderr,
			"PROGRESS inst=%u CS:IP=%04X:%04X IF=%u TF=%u disk_r=%llu disk_w=%llu last_disk_inst=%u same_csip=%u\n",
			(unsigned)inst_counter,
			(unsigned)cs,
			(unsigned)ip,
			(unsigned)regs8[idx(Flag::IF)],
			(unsigned)regs8[idx(Flag::TF)],
			(unsigned long long)g_disk_reads,
			(unsigned long long)g_disk_writes,
			(unsigned)g_last_diskio_inst,
			(unsigned)g_progress_same_csip);

		// If we're spinning at the same CS:IP for a while, dump some state once.
		if (!g_progress_stuck_dumped && g_progress_same_csip >= 10)
		{
			g_progress_stuck_dumped = true;
			const u32 phys = (((u32)cs << 4u) + (u32)ip) & 0xFFFFFu;
			std::fprintf(stderr,
				"STUCK? cs:ip=%04X:%04X phys=%05X IF=%u TF=%u raw=%02X xlat=%u disk_r=%llu disk_w=%llu io60=%02X io64=%02X io40=%02X io42=%02X io43=%02X io3DA=%02X\n",
				(unsigned)cs,
				(unsigned)ip,
				(unsigned)phys,
				(unsigned)regs8[idx(Flag::IF)],
				(unsigned)regs8[idx(Flag::TF)],
				(unsigned)raw_opcode_id,
				(unsigned)xlat_opcode_id,
				(unsigned long long)g_disk_reads,
				(unsigned long long)g_disk_writes,
				(unsigned)io_ports[0x60],
				(unsigned)io_ports[0x64],
				(unsigned)io_ports[0x40],
				(unsigned)io_ports[0x42],
				(unsigned)io_ports[0x43],
				(unsigned)io_ports[0x3DA]);
			std::fprintf(stderr, "STUCK? bytes:");
			for (u32 i = 0; i < 16; ++i)
				std::fprintf(stderr, " %02X", (unsigned)mem[(phys + i) & 0xFFFFFu]);
			std::fprintf(stderr, "\n");
		}
	}
}

static inline void explore_observe_textmode_prompt()
{
	if (!g_explore_mode || !g_explore_warmup_active)
		return;
	if (g_explore_prompt_seen)
		return;

	// Many DOSes write the prompt directly into text VRAM instead of going through the BIOS teletype.
	// Scan the text buffer for a prompt signature like "A:\\>" or "A:\\DOS>".
	constexpr u32 kTextCols = 80;
	constexpr u32 kTextRows = 25;
	constexpr u32 kTextBytes = kTextCols * kTextRows * 2;

	auto scan_base = [&](u32 base) -> bool {
		if (base + kTextBytes > PHYS_MEM_SIZE)
			return false;
		const u8 *vram = mem + base;
		char row[kTextCols];
		for (u32 r = 0; r < kTextRows; ++r)
		{
			for (u32 c = 0; c < kTextCols; ++c)
			{
				const u8 ch = vram[(r * kTextCols + c) * 2u];
				row[c] = ch ? (char)ch : ' ';
			}

			// Common DOS prompt form: optional spaces, then "X:", then some path, then ">".
			u32 start = 0;
			while (start < kTextCols && row[start] == ' ')
				++start;
			if (start + 1 < kTextCols && std::isalpha((unsigned char)row[start]) && row[start + 1] == ':')
			{
				for (u32 i = start + 2; i < kTextCols; ++i)
				{
					if (row[i] == '>')
						return true;
				}
			}

			// Fallback: look for a trailing ">" preceded by a drive like "A:".
			for (u32 i = 0; i < kTextCols; ++i)
			{
				if (row[i] != '>')
					continue;
				if (i >= 3 && row[i - 1] == '\\' && row[i - 2] == ':' && std::isalpha((unsigned char)row[i - 3]))
					return true;
				if (i >= 2 && row[i - 1] == ':' && std::isalpha((unsigned char)row[i - 2]))
					return true;
			}
		}
		return false;
	};

	// Probe both common text VRAM bases; the BIOS data-area heuristic is not reliable across boot images.
	if (scan_base(0xB8000u) || scan_base(0xB0000u))
	{
		g_explore_prompt_seen = true;
		g_explore_stop_requested = true;
		return;
	}
}

static bool load_scripted_keys_from_stream(std::istream &in, std::vector<u16> &out_keys)
{
	std::string line;
	while (std::getline(in, line))
	{
		line = trim(line);
		if (line.empty() || line[0] == '#')
			continue;

		// Support comma-separated tokens per line.
		std::stringstream ss(line);
		std::string token;
		while (std::getline(ss, token, ','))
		{
			u8 ascii = 0;
			if (!try_parse_gorilla_ascii_token(token, ascii))
				return false;
			// Key word format matches SDL path: 0x0400 marks SDL-origin key, low byte is ASCII.
			out_keys.push_back((u16)(0x0400u | (u16)ascii));
		}
	}
	return true;
}

static bool load_scripted_keys_from_file(const char *path, std::vector<u16> &out_keys)
{
	std::ifstream f(path);
	if (!f)
		return false;
	return load_scripted_keys_from_stream(f, out_keys);
}

static inline bool scripted_input_has_key() noexcept
{
	return g_scripted_input && g_scripted_key_pos < g_scripted_keys.size();
}

static inline u16 scripted_input_peek() noexcept
{
	return scripted_input_has_key() ? g_scripted_keys[g_scripted_key_pos] : 0;
}

static inline u16 scripted_input_pop() noexcept
{
	return scripted_input_has_key() ? g_scripted_keys[g_scripted_key_pos++] : 0;
}

static inline u32 phys20(u16 seg, u16 off) noexcept
{
	return (16u * (u32)seg + (u32)off) & (u32)(PHYS_MEM_SIZE - 1u);
}

static inline u32 compute_inst_len() noexcept
{
	return (u32)((i_mod*(i_mod != 3) + 2*(!i_mod && i_rm == 6))*i_mod_size
		+ bios_table_lookup[idx(BiosTable::BASE_INST_SIZE)][raw_opcode_id]
		+ bios_table_lookup[idx(BiosTable::I_W_SIZE)][raw_opcode_id]*(i_w + 1));
}

static inline void record_instruction(u32 phys, const u8 *stream, u32 inst_len) noexcept
{
	if (g_instr_map.empty())
		return;
	if (phys >= g_instr_map.size())
		return;

	InstrInfo &info = g_instr_map[phys];
	++info.exec_count;

	const u8 stored_len = info.len;
	const u8 clamped_len = (u8)((inst_len > info.bytes.size()) ? info.bytes.size() : inst_len);

	if (info.exec_count == 1)
	{
		info.len = clamped_len;
		for (u32 i = 0; i < clamped_len; ++i)
			info.bytes[i] = stream[i];
		return;
	}

	bool mismatch = (stored_len != clamped_len);
	if (!mismatch)
		for (u32 i = 0; i < clamped_len; ++i)
			mismatch |= (info.bytes[i] != stream[i]);
	if (mismatch)
		++info.variant_mismatches;
}

static inline void record_coverage(u32 phys) noexcept
{
	if (!g_track_coverage)
		return;
	if (g_run_coverage.size() != PHYS_MEM_SIZE || g_global_coverage.size() != PHYS_MEM_SIZE)
		return;
	if (phys >= (u32)PHYS_MEM_SIZE)
		return;
	if (!g_run_coverage[phys])
	{
		g_run_coverage[phys] = 1;
		g_run_touched.push_back(phys);
		if (!g_global_coverage[phys])
			++g_last_run_gain;
	}
}

static inline void finalize_run_coverage() noexcept
{
	if (!g_track_coverage)
		return;
	if (g_run_coverage.size() != PHYS_MEM_SIZE || g_global_coverage.size() != PHYS_MEM_SIZE)
		return;
	for (u32 phys : g_run_touched)
	{
		g_run_coverage[phys] = 0;
		g_global_coverage[phys] = 1;
	}
	g_run_touched.clear();
}

static void dump_instr_map_to_file(const char *path)
{
	if (!path)
		return;
	std::FILE *f = std::fopen(path, "w");
	if (!f)
	{
		std::perror("fopen dump-instr");
		return;
	}
	std::fprintf(f, "phys20,len,bytes_hex,exec_count,variant_mismatches\n");
	if (g_instr_map.empty())
	{
		std::fclose(f);
		return;
	}
	for (u32 phys = 0; phys < (u32)g_instr_map.size(); ++phys)
	{
		const InstrInfo &info = g_instr_map[phys];
		if (!info.exec_count)
			continue;
		std::fprintf(f, "%05X,%u,", (unsigned)phys, (unsigned)info.len);
		for (u32 i = 0; i < (u32)info.len && i < (u32)info.bytes.size(); ++i)
			std::fprintf(f, "%02X", (unsigned)info.bytes[i]);
		std::fprintf(f, ",%u,%u\n", (unsigned)info.exec_count, (unsigned)info.variant_mismatches);
	}
	std::fclose(f);
}

static void dump_decisions_to_file(const char *path)
{
	if (!path)
		return;
	std::FILE *f = std::fopen(path, "w");
	if (!f)
	{
		std::perror("fopen dump-decisions");
		return;
	}
	std::fprintf(f, "inst_index,cs,ip,phys20,ah\n");
	if (g_decision_points.empty())
	{
		std::fclose(f);
		return;
	}
	for (const DecisionPoint &dp : g_decision_points)
		std::fprintf(f, "%u,%04X,%04X,%05X,%02X\n",
			(unsigned)dp.inst_index,
			(unsigned)dp.cs,
			(unsigned)dp.ip,
			(unsigned)dp.phys,
			(unsigned)dp.ah);
	std::fclose(f);
}

struct VmSnapshot
{
	std::vector<u8> mem_img;
	std::vector<u8> io_img;
	u16 reg_ip = 0;
	u16 seg_override = 0;
	u16 wave_counter = 0;
	u8 rep_mode = 0;
	u8 seg_override_en = 0;
	u8 rep_override_en = 0;
	u8 trap_flag = 0;
	u8 int8_asap = 0;
	u8 spkr_en = 0;
	u32 inst_counter = 0;
};

static VmSnapshot make_snapshot()
{
	VmSnapshot s;
	s.mem_img.assign(mem, mem + RAM_SIZE);
	s.io_img.assign(io_ports, io_ports + IO_PORT_COUNT);
	s.reg_ip = reg_ip;
	s.seg_override = seg_override;
	s.wave_counter = wave_counter;
	s.rep_mode = rep_mode;
	s.seg_override_en = seg_override_en;
	s.rep_override_en = rep_override_en;
	s.trap_flag = trap_flag;
	s.int8_asap = int8_asap;
	s.spkr_en = spkr_en;
	s.inst_counter = inst_counter;
	return s;
}

static void restore_snapshot(const VmSnapshot &s)
{
	if (s.mem_img.size() != RAM_SIZE || s.io_img.size() != IO_PORT_COUNT)
	{
		std::fprintf(stderr, "restore_snapshot: invalid snapshot sizes (mem=%zu io=%zu)\n", s.mem_img.size(), s.io_img.size());
		return;
	}
	std::memcpy(mem, s.mem_img.data(), RAM_SIZE);
	std::memcpy(io_ports, s.io_img.data(), IO_PORT_COUNT);
	reg_ip = s.reg_ip;
	seg_override = s.seg_override;
	wave_counter = s.wave_counter;
	rep_mode = s.rep_mode;
	seg_override_en = s.seg_override_en;
	rep_override_en = s.rep_override_en;
	trap_flag = s.trap_flag;
	int8_asap = s.int8_asap;
	spkr_en = s.spkr_en;
	inst_counter = s.inst_counter;
}

static std::vector<u8> default_explore_alphabet()
{
	// Enough to navigate DOS prompt and Gorilla, while hard-banning 'N'.
	std::vector<u8> a;
	for (u8 c = '0'; c <= '9'; ++c)
		a.push_back(c);
	for (u8 c = 'A'; c <= 'Z'; ++c)
		if (c != (u8)'N')
			a.push_back(c);
	a.push_back((u8)':');
	a.push_back((u8)'\\');
	a.push_back((u8)'.');
	a.push_back((u8)'_');
	a.push_back((u8)' ');
	a.push_back(0x0D); // Enter
	a.push_back(0x08); // Backspace
	return a;
}

static std::vector<u16> make_key_words_from_ascii(const std::vector<u8> &ascii)
{
	std::vector<u16> out;
	out.reserve(ascii.size());
	for (u8 c : ascii)
		out.push_back((u16)(0x0400u | (u16)c));
	return out;
}

static std::vector<u8> mutate_keys(std::mt19937 &rng, const std::vector<u8> &parent, const std::vector<u8> &alphabet, u32 max_len)
{
	if (alphabet.empty())
		return parent;
	std::vector<u8> child = parent;
	std::uniform_int_distribution<int> op_dist(0, 2); // 0=flip,1=insert,2=delete
	std::uniform_int_distribution<std::size_t> alpha_dist(0, alphabet.size() - 1);

	const int op = op_dist(rng);
	if (op == 0 && !child.empty())
	{
		std::uniform_int_distribution<std::size_t> pos_dist(0, child.size() - 1);
		child[pos_dist(rng)] = alphabet[alpha_dist(rng)];
	}
	else if (op == 1)
	{
		if (child.size() < (std::size_t)max_len)
		{
			std::uniform_int_distribution<std::size_t> pos_dist(0, child.size());
			child.insert(child.begin() + (std::ptrdiff_t)pos_dist(rng), alphabet[alpha_dist(rng)]);
		}
	}
	else if (op == 2 && !child.empty())
	{
		std::uniform_int_distribution<std::size_t> pos_dist(0, child.size() - 1);
		child.erase(child.begin() + (std::ptrdiff_t)pos_dist(rng));
	}

	return child;
}

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

static bool parse_u32_flag(const char *flag, const char *value, u32 &out)
{
	if (!value || !*value)
	{
		std::fprintf(stderr, "Missing value for %s\n", flag);
		return false;
	}
	char *end = nullptr;
	errno = 0;
	const unsigned long v = std::strtoul(value, &end, 10);
	if (errno != 0 || end == value || (end && *end != '\0'))
	{
		std::fprintf(stderr, "Invalid value for %s: %s\n", flag, value);
		return false;
	}
	if (v > 0xFFFFFFFFul)
	{
		std::fprintf(stderr, "Out-of-range value for %s: %s\n", flag, value);
		return false;
	}
	out = (u32)v;
	return true;
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

static inline u32 top_bit(BitSize size) noexcept
{
	return (u32)size;
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
	regs16[idx(Reg16::SP)] = (u16)(regs16[idx(Reg16::SP)] - 2);
	R_M_OP(mem[segreg(idx(Reg16::SS), (u16)regs16[idx(Reg16::SP)])], =, value);
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

		// Match the original behavior: write a 16-bit key word to BDA 0x4A6 with modulo-16 truncation.
		const u32 key_word = 0x400u + 0x800u * !!(scratch2_uint & KMOD_ALT)
			+ 0x1000u * !!(scratch2_uint & KMOD_SHIFT) + 0x2000u * !!(scratch2_uint & KMOD_CTRL)
			+ 0x4000u * (sdl_event.type == SDL_KEYUP) + (u32)scratch_uint;
		ref<u16>(mem[0x4A6]) = (u16)key_word;
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
	// Keep the original precedence: (oldCF && result==dest) || (result < (int)dest)
	set_CF((regs8[idx(Flag::CF)] && (op_result == op_dest)) || (op_result < (int)op_dest));
	set_AF_OF_arith();
	return 0;
}

static inline int sbb_op() noexcept
{
	OP(-= regs8[idx(Flag::CF)] +);
	// Keep the original precedence: (oldCF && result==dest) || (-result < -(int)dest)
	set_CF((regs8[idx(Flag::CF)] && (op_result == op_dest)) || (-op_result < -(int)op_dest));
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
	preparse_early_audio_flags(argc, argv);
#endif
#ifndef NO_GRAPHICS
	// Initialise SDL (SDL2)
	Uint32 sdl_flags = SDL_INIT_VIDEO;
	if (g_audio_enabled)
		sdl_flags |= SDL_INIT_AUDIO;
	SDL_Init(sdl_flags);
	sdl_audio = SDL_AudioSpec{};
	sdl_audio_dev = 0;
	if (g_audio_enabled)
	{
		sdl_audio.freq = 44100;
		sdl_audio.format = AUDIO_U8;
		sdl_audio.channels = 1;
		sdl_audio.samples = 512;
		sdl_audio.callback = audio_callback;
		sdl_audio_dev = SDL_OpenAudioDevice(nullptr, 0, &sdl_audio, nullptr, 0);
		if (sdl_audio_dev)
			SDL_PauseAudioDevice(sdl_audio_dev, 0);
	}

	sdl_window.reset();
	// Parse args early enough to know whether to init SDL audio.
	// (We still init SDL video for the interactive build unless NO_GRAPHICS is set.)
	sdl_renderer.reset();
	sdl_texture.reset();
#endif

	// Graceful termination for long headless runs.
	std::signal(SIGINT, on_host_signal);
	std::signal(SIGTERM, on_host_signal);
	std::signal(SIGQUIT, on_host_signal);
#ifdef SIGTSTP
	std::signal(SIGTSTP, on_host_signal);
#endif

	std::vector<const char *> positional;
	positional.reserve((std::size_t)argc);
	for (int i = 1; i < argc; ++i)
	{
		const char *arg = argv[i];
		if (!arg)
			continue;

		if (!std::strcmp(arg, "--dump-instr") && (i + 1) < argc)
		{
			g_dump_instr_path = argv[++i];
			continue;
		}
		if (!std::strncmp(arg, "--dump-instr=", 12))
		{
			g_dump_instr_path = arg + 12;
			continue;
		}
		if (!std::strcmp(arg, "--dump-decisions") && (i + 1) < argc)
		{
			g_dump_decisions_path = argv[++i];
			continue;
		}
		if (!std::strncmp(arg, "--dump-decisions=", 17))
		{
			g_dump_decisions_path = arg + 17;
			continue;
		}
		if (!std::strcmp(arg, "--max-inst") && (i + 1) < argc)
		{
			if (!parse_u32_flag("--max-inst", argv[++i], g_max_instructions))
				return 2;
			continue;
		}
		if (!std::strncmp(arg, "--max-inst=", 11))
		{
			if (!parse_u32_flag("--max-inst", arg + 11, g_max_instructions))
				return 2;
			continue;
		}
		if (!std::strcmp(arg, "--progress") && (i + 1) < argc)
		{
			if (!parse_u32_flag("--progress", argv[++i], g_progress_every_inst))
				return 2;
			continue;
		}
		if (!std::strncmp(arg, "--progress=", 11))
		{
			if (!parse_u32_flag("--progress", arg + 11, g_progress_every_inst))
				return 2;
			continue;
		}
		if (!std::strcmp(arg, "--tty-limit") && (i + 1) < argc)
		{
			if (!parse_u32_flag("--tty-limit", argv[++i], g_tty_limit_bytes))
				return 2;
			continue;
		}
		if (!std::strncmp(arg, "--tty-limit=", 12))
		{
			if (!parse_u32_flag("--tty-limit", arg + 12, g_tty_limit_bytes))
				return 2;
			continue;
		}
		if (!std::strcmp(arg, "--explore"))
		{
			g_explore_mode = true;
			continue;
		}
		if (!std::strcmp(arg, "--explore-save-best") && (i + 1) < argc)
		{
			g_explore_best_keys_path = argv[++i];
			continue;
		}
		if (!std::strncmp(arg, "--explore-save-best=", 19))
		{
			g_explore_best_keys_path = arg + 19;
			continue;
		}
		if (!std::strcmp(arg, "--explore-save-every") && (i + 1) < argc)
		{
			if (!parse_u32_flag("--explore-save-every", argv[++i], g_explore_save_every))
				return 2;
			continue;
		}
		if (!std::strncmp(arg, "--explore-save-every=", 20))
		{
			if (!parse_u32_flag("--explore-save-every", arg + 20, g_explore_save_every))
				return 2;
			continue;
		}
		if (!std::strcmp(arg, "--explore-dump-every") && (i + 1) < argc)
		{
			if (!parse_u32_flag("--explore-dump-every", argv[++i], g_explore_dump_every))
				return 2;
			continue;
		}
		if (!std::strncmp(arg, "--explore-dump-every=", 20))
		{
			if (!parse_u32_flag("--explore-dump-every", arg + 20, g_explore_dump_every))
				return 2;
			continue;
		}
		if (!std::strcmp(arg, "--run-exe") && (i + 1) < argc)
		{
			g_run_exe_host_path = argv[++i];
			continue;
		}
		if (!std::strcmp(arg, "--exe") && (i + 1) < argc)
		{
			g_run_exe_host_path = argv[++i];
			continue;
		}
		if (!std::strncmp(arg, "--run-exe=", 10))
		{
			g_run_exe_host_path = arg + 10;
			continue;
		}
		if (!std::strncmp(arg, "--exe=", 6))
		{
			g_run_exe_host_path = arg + 6;
			continue;
		}
		if (!std::strcmp(arg, "--quiet"))
		{
			g_quiet_output = true;
			g_quiet_output_user_set = true;
			continue;
		}
		if (!std::strcmp(arg, "--no-audio") || !std::strcmp(arg, "--mute"))
		{
			g_audio_enabled = false;
			g_audio_user_set = true;
			continue;
		}
		if (!std::strcmp(arg, "--audio"))
		{
			g_audio_enabled = true;
			g_audio_user_set = true;
			continue;
		}
		if (!std::strcmp(arg, "--loud"))
		{
			g_quiet_output = false;
			g_quiet_output_user_set = true;
			continue;
		}
		if (!std::strcmp(arg, "--debug-diskio"))
		{
			g_debug_diskio = true;
			g_debug_diskio_left = 200;
			continue;
		}
		if (!std::strcmp(arg, "--explore-iters") && (i + 1) < argc)
		{
			if (!parse_u32_flag("--explore-iters", argv[++i], g_explore_iters))
				return 2;
			continue;
		}
		if (!std::strncmp(arg, "--explore-iters=", 15))
		{
			if (!parse_u32_flag("--explore-iters", arg + 15, g_explore_iters))
				return 2;
			continue;
		}
		if (!std::strcmp(arg, "--explore-seed") && (i + 1) < argc)
		{
			if (!parse_u32_flag("--explore-seed", argv[++i], g_explore_seed))
				return 2;
			continue;
		}
		if (!std::strncmp(arg, "--explore-seed=", 15))
		{
			if (!parse_u32_flag("--explore-seed", arg + 15, g_explore_seed))
				return 2;
			continue;
		}
		if (!std::strcmp(arg, "--explore-max-keys") && (i + 1) < argc)
		{
			if (!parse_u32_flag("--explore-max-keys", argv[++i], g_explore_max_keys))
				return 2;
			continue;
		}
		if (!std::strncmp(arg, "--explore-max-keys=", 19))
		{
			if (!parse_u32_flag("--explore-max-keys", arg + 19, g_explore_max_keys))
				return 2;
			continue;
		}
		if (!std::strcmp(arg, "--explore-corpus") && (i + 1) < argc)
		{
			if (!parse_u32_flag("--explore-corpus", argv[++i], g_explore_corpus_max))
				return 2;
			continue;
		}
		if (!std::strncmp(arg, "--explore-corpus=", 17))
		{
			if (!parse_u32_flag("--explore-corpus", arg + 17, g_explore_corpus_max))
				return 2;
			continue;
		}
		if (!std::strcmp(arg, "--explore-warmup-inst") && (i + 1) < argc)
		{
			if (!parse_u32_flag("--explore-warmup-inst", argv[++i], g_explore_warmup_inst))
				return 2;
			g_explore_warmup_inst_user_set = true;
			continue;
		}
		if (!std::strncmp(arg, "--explore-warmup-inst=", 22))
		{
			if (!parse_u32_flag("--explore-warmup-inst", arg + 22, g_explore_warmup_inst))
				return 2;
			g_explore_warmup_inst_user_set = true;
			continue;
		}
		if (!std::strcmp(arg, "--keys-file") && (i + 1) < argc)
		{
			const char *path = argv[++i];
			std::vector<u16> tmp;
			if (!load_scripted_keys_from_file(path, tmp))
			{
				std::fprintf(stderr, "Failed to load --keys-file (or banned/unknown token): %s\n", path);
				return 2;
			}
			g_scripted_input = true;
			g_scripted_keys = std::move(tmp);
			g_scripted_key_pos = 0;
			g_scripted_input_user_set = true;
			continue;
		}
		if (!std::strncmp(arg, "--keys-file=", 12))
		{
			const char *path = arg + 12;
			std::vector<u16> tmp;
			if (!load_scripted_keys_from_file(path, tmp))
			{
				std::fprintf(stderr, "Failed to load --keys-file (or banned/unknown token): %s\n", path);
				return 2;
			}
			g_scripted_input = true;
			g_scripted_keys = std::move(tmp);
			g_scripted_key_pos = 0;
			g_scripted_input_user_set = true;
			continue;
		}
		if (!std::strcmp(arg, "--keys") && (i + 1) < argc)
		{
			std::stringstream ss(argv[++i]);
			std::vector<u16> tmp;
			if (!load_scripted_keys_from_stream(ss, tmp))
			{
				std::fprintf(stderr, "Failed to parse --keys (banned/unknown token).\n");
				return 2;
			}
			g_scripted_input = true;
			g_scripted_keys = std::move(tmp);
			g_scripted_key_pos = 0;
			g_scripted_input_user_set = true;
			continue;
		}
		if (!std::strncmp(arg, "--keys=", 7))
		{
			std::stringstream ss(arg + 7);
			std::vector<u16> tmp;
			if (!load_scripted_keys_from_stream(ss, tmp))
			{
				std::fprintf(stderr, "Failed to parse --keys (banned/unknown token).\n");
				return 2;
			}
			g_scripted_input = true;
			g_scripted_keys = std::move(tmp);
			g_scripted_key_pos = 0;
			g_scripted_input_user_set = true;
			continue;
		}

		positional.push_back(arg);
	}

	if (g_explore_mode)
	{
		// Exploration runs are most useful when they persist artifacts by default.
		if (!g_dump_instr_path)
			g_dump_instr_path = "instr.csv";
		if (!g_dump_decisions_path)
			g_dump_decisions_path = "decisions.csv";
		if (!g_explore_best_keys_path)
			g_explore_best_keys_path = "best_keys.txt";
		// Exploration is headless/high-volume; suppress BIOS PUTCHAR output unless the user opted in.
		if (!g_quiet_output_user_set)
			g_quiet_output = true;
		// Exploration runs shouldn't make noise by default.
		if (!g_audio_user_set)
			g_audio_enabled = false;
	}

	if (positional.size() < 2)
	{
		std::fprintf(stderr,
			"Usage: %s [--max-inst N] [--progress N] [--tty-limit N] [--dump-instr path.csv] [--dump-decisions path.csv] [--keys-file keys.txt | --keys tok,tok,...]\n"
			"          [--run-exe host.exe|--exe host.exe]\n"
			"          [--quiet|--loud] [--audio|--no-audio]\n"
			"          [--debug-diskio]\n"
			"          [--explore --explore-iters N --explore-seed N --explore-max-keys N --explore-corpus N --explore-warmup-inst N]\n"
			"          [--explore-save-best path.txt --explore-save-every N --explore-dump-every N]\n"
			"          bios fd.img [hd.img]\n",
			argv[0]);
		return 1;
	}

	const char *bios_path = positional[0];
	const char *fd_path = positional[1];
	const char *hd_path = (positional.size() >= 3) ? positional[2] : nullptr;

	// Optional: copy fd.img, inject the requested EXE into the FAT12 root directory,
	// and boot from the prepared image.
	if (g_run_exe_host_path)
	{
		g_fd_prepared_path = std::string(fd_path) + ".run.img";
		if (!copy_file_binary(fd_path, g_fd_prepared_path.c_str()))
		{
			std::fprintf(stderr, "--run-exe: failed to copy %s -> %s\n", fd_path, g_fd_prepared_path.c_str());
			return 1;
		}
		if (!fat12_inject_root_file(g_fd_prepared_path.c_str(), g_run_exe_host_path, nullptr))
		{
			std::fprintf(stderr, "--run-exe: failed to inject %s into %s\n", g_run_exe_host_path, g_fd_prepared_path.c_str());
			return 1;
		}
		fd_path = g_fd_prepared_path.c_str();
	}

	// Convenience: if an EXE was injected and the user didn't provide keys, auto-type the program name
	// at the first DOS prompt keyboard read.
	if (g_run_exe_host_path && !g_explore_mode && !g_scripted_input_user_set)
	{
		std::string base = std::filesystem::path(g_run_exe_host_path).filename().string();
		char name83[11];
		std::string prog = "";
		if (make_dos_83_upper(base, name83))
		{
			prog.assign(name83, name83 + 8);
			while (!prog.empty() && prog.back() == ' ')
				prog.pop_back();
		}
		else
		{
			// Fallback: just strip extension and uppercase.
			prog = std::filesystem::path(base).stem().string();
			for (char &c : prog)
				c = (char)std::toupper((unsigned char)c);
		}
		if (!prog.empty())
		{
			std::vector<u8> ascii;
			ascii.reserve(prog.size() + 1);
			for (char c : prog)
				ascii.push_back((u8)c);
			ascii.push_back(0x0D);
			g_scripted_input = true;
			g_scripted_keys = make_key_words_from_ascii(ascii);
			g_scripted_key_pos = 0;
			std::fprintf(stderr, "Auto-run: will type '%s' + Enter at prompt.\n", prog.c_str());
		}
	}

	if (g_dump_instr_path || g_explore_mode)
		g_instr_map.assign(PHYS_MEM_SIZE, InstrInfo{});

	// regs16 and reg8 point to F000:0, the start of memory-mapped registers. CS is initialised to F000
	regs8 = mem + REGS_BASE;
	regs16 = reinterpret_cast<u16 *>(regs8);
	regs16[idx(Reg16::CS)] = 0xF000;

	// Trap flag off
	regs8[idx(Flag::TF)] = 0;

	// Set DL equal to the boot device: 0 for the FD, or 0x80 for the HD. Normally, boot from the FD.
	// But, if the HD image file is prefixed with @, then boot from the HD
	regs8[idx(Reg8::DL)] = (hd_path && *hd_path == '@') ? 0x80 : 0;

	// Open BIOS (file id disk[2]), floppy disk image (disk[1]), and hard disk image (disk[0]) if specified
	{
		const char *paths[3] = {
			bios_path,
			fd_path,
			hd_path ? (hd_path[0] == '@' ? (hd_path + 1) : hd_path) : nullptr,
		};
		for (int i = 0; i < 3; ++i)
		{
			const char *path = paths[i];
			const int target_index = 2 - i; // [bios,fd,hd] -> disk[2], disk[1], disk[0]
			if (!path)
			{
				disk[target_index] = 0;
				continue;
			}
			const int fd = platform::open_disk_image(path);
			if (fd < 0)
			{
				std::perror(path);
				disk[target_index] = 0;
				continue;
			}
			disk[target_index] = fd;
		}
	}

	// BIOS and floppy are required to boot.
	if (disk[2] <= 0)
	{
		std::fprintf(stderr, "Failed to open BIOS image: %s\n", bios_path);
		return 2;
	}
	{
		const std::int64_t bios_size = platform::seek_end_bytes(disk[2]);
		platform::seek_set_bytes(disk[2], 0);
		// This emulator expects a BIOS image that includes the helper tables and full code region.
		// We read 0xFF00 bytes into F000:0100.
		if (bios_size > 0 && bios_size < 0xFF00)
		{
			std::fprintf(stderr,
				"Fatal: BIOS image '%s' is too small (%lld bytes). Expected at least %u bytes.\n"
				"Hint: rebuild the BIOS from bios_source/bios.asm (e.g., with nasm) or provide a correct BIOS binary.\n",
				bios_path,
				(long long)bios_size,
				(unsigned)0xFF00);
			return 2;
		}
	}
	if (disk[1] <= 0)
	{
		std::fprintf(stderr, "Failed to open floppy image: %s\n", fd_path);
		return 2;
	}
	if (hd_path && disk[0] <= 0)
		std::fprintf(stderr, "Warning: hard disk image not opened: %s\n", hd_path);

	// Set CX:AX equal to the hard disk image size, if present
	const std::int64_t disk_size_bytes = *disk ? platform::seek_end_bytes(*disk) : -1;
	ref<unsigned>(regs16[idx(Reg16::AX)]) = (disk_size_bytes > 0) ? (unsigned)(disk_size_bytes >> 9) : 0;

	// Load BIOS image into F000:0100, and set IP to 0100
	{
		reg_ip = 0x100;
		const std::int64_t bios_read = platform::read_fd(disk[2], regs8 + reg_ip, 0xFF00);
		if (bios_read < 0)
		{
			std::perror("BIOS read");
			return 2;
		}
		if (bios_read != 0xFF00)
		{
			std::fprintf(stderr,
				"Fatal: BIOS read short from '%s': got %lld bytes, expected %u.\n"
				"Hint: provide a correct BIOS binary (must include decoder tables).\n",
				bios_path,
				(long long)bios_read,
				(unsigned)0xFF00);
			return 2;
		}
	}

	// Load instruction decoding helper table
	for (int i = 0; i < 20; i++)
		for (int j = 0; j < 256; j++)
			bios_table_lookup[i][j] = regs8[regs16[0x81 + i] + j];

	// Exploration mode: run many trials in-process for speed.
	VmSnapshot base_snapshot;
	std::mt19937 explore_rng;
	std::vector<std::vector<u8>> explore_corpus;
	std::vector<u8> explore_alphabet;
	std::vector<u8> explore_current;
	std::vector<u8> explore_user_seed_ascii;
	u32 explore_saved_max_inst = 0;
	u32 explore_warmup_ext_left = 12;
	u32 explore_iter = 0;
	u32 explore_best_gain = 0;
	std::vector<u8> explore_best_seq;

	if (g_explore_mode)
	{
		g_track_coverage = true;
		g_global_coverage.assign(PHYS_MEM_SIZE, 0);
		g_run_coverage.assign(PHYS_MEM_SIZE, 0);
		g_run_touched.clear();
		g_run_touched.reserve(4096);

		explore_rng.seed(g_explore_seed);
		explore_alphabet = default_explore_alphabet();

		// Preserve any user-provided key seed, then run a warmup boot to reach the DOS prompt.
		if (g_scripted_input && !g_scripted_keys.empty())
		{
			explore_user_seed_ascii.reserve(g_scripted_keys.size());
			for (u16 kw : g_scripted_keys)
				explore_user_seed_ascii.push_back((u8)(kw & 0xFF));
		}

		explore_saved_max_inst = g_max_instructions;
		if (!g_max_instructions || g_max_instructions < g_explore_warmup_inst)
			g_max_instructions = g_explore_warmup_inst;

		g_explore_warmup_active = true;
		g_explore_prompt_seen = false;
		g_explore_tty_tail.clear();
		g_last_run_gain = 0;

		// Empty scripted input to keep warmup deterministic and non-blocking.
		g_scripted_input = true;
		g_scripted_keys.clear();
		g_scripted_key_pos = 0;
	}

	run_loop:

	// Instruction execution loop. Terminates if CS:IP = 0:0
	for (; opcode_stream = mem + 16 * regs16[idx(Reg16::CS)] + reg_ip, opcode_stream != mem;)
	{
		if (g_host_stop_signal)
			break;
		const u32 cur_inst = inst_counter + 1u;
		const u16 inst_cs = regs16[idx(Reg16::CS)];
		const u16 inst_ip = reg_ip;
		const u32 inst_phys = phys20(inst_cs, inst_ip);

		// Set up variables to prepare for decoding an opcode
		set_opcode(*opcode_stream);

		// Extract i_w and i_d fields from instruction
		i_w = (i_reg4bit = raw_opcode_id & 7) & 1;
		i_d = i_reg4bit / 2 & 1;

		// Extract instruction data fields
		i_data0 = ref<short>(opcode_stream[1]);
		i_data1 = ref<short>(opcode_stream[2]);
		i_data2 = ref<short>(opcode_stream[3]);

#if TRACE_LEN
		{
			const u32 cur_inst = inst_counter + 1u;
			if (cur_inst >= (u32)TRACE_FROM && cur_inst < (u32)TRACE_FROM + (u32)TRACE_LEN)
			{
				// Format matches the historic trace files in this repo (trace_c.txt / trace_cpp.txt).
				std::fprintf(stdout,
					"T %u CS:IP=%04X:%04X op=%02X xlat=%u extra=%u w=%u d=%u modsz=%u AL=%02X AX=%04X IF=%u CF=%u ZF=%u SF=%u OF=%u\n",
					(unsigned)cur_inst,
					(unsigned)regs16[idx(Reg16::CS)],
					(unsigned)reg_ip,
					(unsigned)(unsigned char)*opcode_stream,
					(unsigned)xlat_opcode_id,
					(unsigned)extra,
					(unsigned)i_w,
					(unsigned)i_d,
					(unsigned)i_mod_size,
					(unsigned)regs8[idx(Reg8::AL)],
					(unsigned)regs16[idx(Reg16::AX)],
					(unsigned)regs8[idx(Flag::IF)],
					(unsigned)regs8[idx(Flag::CF)],
					(unsigned)regs8[idx(Flag::ZF)],
					(unsigned)regs8[idx(Flag::SF)],
					(unsigned)regs8[idx(Flag::OF)]);
			}
			// Stop once the trace window is complete to keep runs short and diff-friendly.
			if (cur_inst >= (u32)TRACE_FROM + (u32)TRACE_LEN)
				break;
		}
#endif

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

		const u32 inst_len = compute_inst_len();
		record_coverage(inst_phys);
		record_instruction(inst_phys, opcode_stream, inst_len);

		// Instruction execution unit
		switch (static_cast<OpcodeGroup>(xlat_opcode_id))
		{
			case OpcodeGroup::COND_JUMP:
				// i_w is the invert flag, e.g. i_w == 1 means JNAE, whereas i_w == 0 means JAE 
				scratch_uchar = raw_opcode_id / 2 & 7;
				reg_ip += (char)i_data0 * (i_w ^ (regs8[bios_table_lookup[idx(BiosTable::COND_JUMP_DECODE_A)][scratch_uchar]] || regs8[bios_table_lookup[idx(BiosTable::COND_JUMP_DECODE_B)][scratch_uchar]] || regs8[bios_table_lookup[idx(BiosTable::COND_JUMP_DECODE_C)][scratch_uchar]] ^ regs8[bios_table_lookup[idx(BiosTable::COND_JUMP_DECODE_D)][scratch_uchar]]));
				break;
			case OpcodeGroup::MOV_REG_IMM:
				i_w = !!(raw_opcode_id & 8);
				R_M_OP(mem[get_reg_addr(i_reg4bit)], =, i_data0);
				break;
			case OpcodeGroup::PUSH_REG16:
				r_m_push(regs16[i_reg4bit]);
				break;
			case OpcodeGroup::POP_REG16:
				r_m_pop(regs16[i_reg4bit]);
				break;
			case OpcodeGroup::INC_DEC_REG16:
				i_w = 1;
				i_d = 0;
				i_reg = i_reg4bit;
				decode_rm_reg();
				i_reg = extra;
					[[fallthrough]];
			case OpcodeGroup::GRP_INC_DEC_JMP_CALL_PUSH:
				if (i_reg < 2) // INC | DEC
					MEM_OP(op_from_addr, += 1 - 2 * i_reg +, REGS_BASE + 2 * idx(Reg16::ZERO)),
					op_source = 1,
					set_AF_OF_arith(),
					set_OF(op_dest + 1 - i_reg == (1u << (top_bit() - 1u))),
					(xlat_opcode_id == idx(OpcodeGroup::GRP_INC_DEC_JMP_CALL_PUSH)) && (set_opcode(0x10), 0); // Decode like ADC
				else if (i_reg != idx(Group5Op::PUSH)) // JMP | CALL
					i_reg - idx(Group5Op::CALL_FAR) || r_m_push(regs16[idx(Reg16::CS)]), // CALL (far)
					i_reg & 2 && r_m_push(reg_ip + 2 + i_mod*(i_mod != 3) + 2*(!i_mod && i_rm == 6)), // CALL (near or far)
					i_reg & 1 && (regs16[idx(Reg16::CS)] = ref<short>(mem[op_from_addr + 2])), // JMP|CALL (far)
					R_M_OP(reg_ip, =, mem[op_from_addr]),
					set_opcode(0x9A); // Decode like CALL
				else // PUSH
						r_m_push(ref<unsigned short>(mem[rm_addr]));
					break;
				case OpcodeGroup::GRP_TEST_NOT_NEG_MUL_DIV:
				op_to_addr = op_from_addr;

				switch (static_cast<Group6Op>(i_reg))
				{
						case Group6Op::TEST:
						set_opcode(0x20); // Decode like AND
						reg_ip += i_w + 1;
							R_M_OP(mem[op_to_addr], &, i_data2);
							break;
						case Group6Op::NOT:
							OP(=~);
							break;
						case Group6Op::NEG:
						OP(=-);
						op_dest = 0;
						set_opcode(0x28); // Decode like SUB
							set_CF(op_result > op_dest);
							break;
						case Group6Op::MUL:
							i_w ? mul_op<unsigned short>(regs16) : mul_op<unsigned char>(regs8);
							break;
						case Group6Op::IMUL:
							i_w ? mul_op<short>(regs16) : mul_op<char>(regs8);
							break;
						case Group6Op::DIV:
							i_w ? div_op<unsigned short, unsigned>(regs16) : div_op<unsigned char, unsigned short>(regs8);
							break;
						case Group6Op::IDIV:
							i_w ? div_op<short, int>(regs16) : div_op<char, short>(regs8);
							break;
				}
					break;
				case OpcodeGroup::ALU_AL_AX_IMM:
					rm_addr = REGS_BASE;
					i_data2 = i_data0;
					i_mod = 3;
					i_reg = extra;
					reg_ip--;
					[[fallthrough]];
				case OpcodeGroup::ALU_REG_IMM:
				op_to_addr = rm_addr;
				regs16[idx(Reg16::SCRATCH)] = (i_d |= !i_w) ? (char)i_data2 : i_data2;
				op_from_addr = REGS_BASE + 2 * idx(Reg16::SCRATCH);
				reg_ip += !i_d + 1;
				set_opcode(0x08 * (extra = i_reg));
					[[fallthrough]];
				case OpcodeGroup::ALU_REG_RM:
				switch (static_cast<AluOp>(extra))
				{
						case AluOp::ADD:
							OP(+=),
							set_CF(op_result < op_dest);
							break;
						case AluOp::OR:
							OP(|=);
							break;
						case AluOp::ADC:
							adc_op();
							break;
						case AluOp::SBB:
							sbb_op();
							break;
						case AluOp::AND:
							OP(&=);
							break;
						case AluOp::SUB:
							OP(-=),
							set_CF(op_result > op_dest);
							break;
						case AluOp::XOR:
							OP(^=);
							break;
						case AluOp::CMP:
							OP(-),
							set_CF(op_result > op_dest);
							break;
						case AluOp::MOV:
							OP(=);
							break;
				}
					break;
				case OpcodeGroup::MOV_SREG_POP_LEA:
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
				case OpcodeGroup::MOV_AL_AX_LOC:
				i_mod = i_reg = 0;
				i_rm = 6;
				i_data1 = i_data0;
				decode_rm_reg();
					MEM_OP(op_from_addr, =, op_to_addr);
					break;
				case OpcodeGroup::GRP_BITWISE:
				{
					const OpView opv = op_view_for(OpcodeGroup::GRP_BITWISE, i_w, i_d);
					const u32 bits = top_bit(opv.size);

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
						scratch_uint %= i_reg / 2 + bits,
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

					switch (static_cast<BitwiseOp>(i_reg))
					{
						case BitwiseOp::ROL:
							R_M_OP(mem[rm_addr], += , scratch2_uint >> (bits - scratch_uint));
							set_OF(sign_of(op_result) ^ set_CF(op_result & 1));
							break;
						case BitwiseOp::ROR:
							scratch2_uint &= (1 << scratch_uint) - 1;
							R_M_OP(mem[rm_addr], += , scratch2_uint << (bits - scratch_uint));
							set_OF(sign_of(op_result * 2) ^ set_CF(sign_of(op_result)));
							break;
						case BitwiseOp::RCL:
							R_M_OP(mem[rm_addr], += (regs8[idx(Flag::CF)] << (scratch_uint - 1)) + , scratch2_uint >> (1u + bits - scratch_uint));
							set_OF(sign_of(op_result) ^ set_CF(scratch2_uint & (1u << (bits - scratch_uint))));
							break;
						case BitwiseOp::RCR:
							R_M_OP(mem[rm_addr], += (regs8[idx(Flag::CF)] << (bits - scratch_uint)) + , scratch2_uint << (1u + bits - scratch_uint));
							set_CF(scratch2_uint & 1 << (scratch_uint - 1));
							set_OF(sign_of(op_result) ^ sign_of(op_result * 2));
							break;
						case BitwiseOp::SHL:
							set_OF(sign_of(op_result) ^ set_CF(sign_of(op_dest << (scratch_uint - 1))));
							break;
						case BitwiseOp::SHR:
							set_OF(sign_of(op_dest));
							break;
						case BitwiseOp::SAR:
							scratch_uint < bits || set_CF(scratch2_uint);
							set_OF(0);
							R_M_OP(mem[rm_addr], +=, scratch2_uint *= ~(((1u << bits) - 1u) >> scratch_uint));
							break;
						default:
							break;
					}
					break;
				}
				case OpcodeGroup::GRP_LOOP:
				scratch_uint = !!--regs16[idx(Reg16::CX)];

				switch (static_cast<LoopOp>(i_reg4bit))
				{
					case LoopOp::LOOPNZ:
						scratch_uint &= !regs8[idx(Flag::ZF)];
						break;
					case LoopOp::LOOPZ:
						scratch_uint &= regs8[idx(Flag::ZF)];
						break;
					case LoopOp::JCXZ:
						scratch_uint = !++regs16[idx(Reg16::CX)];
						break;
					default:
						break;
				}
				reg_ip += scratch_uint*(char)i_data0;
				break;
			case OpcodeGroup::JMP_CALL_SHORT_NEAR:
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
			case OpcodeGroup::TEST_REG_RM:
				MEM_OP(op_from_addr, &, op_to_addr);
				break;
			case OpcodeGroup::XCHG_AX_REG16:
				i_w = 1;
				op_to_addr = REGS_BASE;
				op_from_addr = get_reg_addr(i_reg4bit);
				[[fallthrough]];
			case OpcodeGroup::NOP_XCHG_REG_RM:
				if (op_to_addr != op_from_addr)
					OP(^=),
					MEM_OP(op_from_addr, ^=, op_to_addr),
					OP(^=);
				break;
			case OpcodeGroup::GRP_MOVS_STOS_LODS:
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
			case OpcodeGroup::GRP_CMPS_SCAS:
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
			case OpcodeGroup::RET_RETF_IRET:
				i_d = i_w;
				r_m_pop(reg_ip);
				if (extra) // IRET|RETF|RETF imm16
					r_m_pop(regs16[idx(Reg16::CS)]);
				if (extra & 2) // IRET
					set_flags(r_m_pop(scratch_uint));
				else if (!i_d) // RET|RETF imm16
					regs16[idx(Reg16::SP)] += i_data0;
				break;
			case OpcodeGroup::MOV_RM_IMM:
				R_M_OP(mem[op_from_addr], =, i_data2);
				break;
			case OpcodeGroup::IN_AL_AX:
				io_ports[0x20] = 0; // PIC EOI
				io_ports[0x42] = --io_ports[0x40]; // PIT channel 0/2 read placeholder
				io_ports[0x3DA] ^= 9; // CGA refresh
				scratch_uint = extra ? regs16[idx(Reg16::DX)] : (unsigned char)i_data0;
				scratch_uint == 0x60 && (io_ports[0x64] = 0); // Scancode read flag
				scratch_uint == 0x3D5 && (io_ports[0x3D4] >> 1 == 7) && (io_ports[0x3D5] = ((mem[0x49E]*80 + mem[0x49D] + ref<short>(mem[0x4AD])) & (io_ports[0x3D4] & 1 ? 0xFF : 0xFF00)) >> (io_ports[0x3D4] & 1 ? 0 : 8)); // CRT cursor position
				R_M_OP(regs8[idx(Reg8::AL)], =, io_ports[scratch_uint]);
				break;
			case OpcodeGroup::OUT_AL_AX:
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
			case OpcodeGroup::REPXX:
				rep_override_en = 2;
				rep_mode = i_w;
				seg_override_en && seg_override_en++;
				break;
			case OpcodeGroup::PUSH_REG:
				r_m_push(regs16[extra]);
				break;
			case OpcodeGroup::POP_REG:
				r_m_pop(regs16[extra]);
				break;
			case OpcodeGroup::SEGMENT_OVERRIDE:
				seg_override_en = 2;
				seg_override = extra;
				rep_override_en && rep_override_en++;
				break;
			case OpcodeGroup::DAA_DAS:
				i_w = 0;
				extra ? das() : daa(); // extra = 0 for DAA, 1 for DAS
				break;
			case OpcodeGroup::AAA_AAS:
				op_result = AAA_AAS(extra - 1);
				break;
			case OpcodeGroup::CBW:
				regs8[idx(Reg8::AH)] = -sign_of(regs8[idx(Reg8::AL)]);
				break;
			case OpcodeGroup::CWD:
				regs16[idx(Reg16::DX)] = -sign_of(regs16[idx(Reg16::AX)]);
				break;
			case OpcodeGroup::CALL_FAR_IMM:
				r_m_push(regs16[idx(Reg16::CS)]);
				r_m_push(reg_ip + 5);
				regs16[idx(Reg16::CS)] = i_data2;
				reg_ip = i_data0;
				break;
			case OpcodeGroup::PUSHF:
				make_flags();
				r_m_push(scratch_uint);
				break;
			case OpcodeGroup::POPF:
				set_flags(r_m_pop(scratch_uint));
				break;
			case OpcodeGroup::SAHF:
				make_flags();
				set_flags((scratch_uint & 0xFF00) + regs8[idx(Reg8::AH)]);
				break;
			case OpcodeGroup::LAHF:
				make_flags(),
				regs8[idx(Reg8::AH)] = scratch_uint;
				break;
			case OpcodeGroup::LES_LDS_REG_RM:
				i_w = i_d = 1;
				decode_rm_reg();
				OP(=);
				MEM_OP(REGS_BASE + extra, =, rm_addr + 2);
				break;
			case OpcodeGroup::INT_3:
				++reg_ip;
				pc_interrupt(3);
				break;
			case OpcodeGroup::INT_IMM8:
				if ((u8)i_data0 == 0x16)
				{
					const u8 ah = regs8[idx(Reg8::AH)];
					if (ah == 0x00 || ah == 0x01)
					{
						if (g_dump_decisions_path || g_decision_points.size() < 100000u)
							g_decision_points.push_back(DecisionPoint{cur_inst, inst_cs, inst_ip, inst_phys, ah});
					}

					// Scripted input mode: emulate INT 16h directly for speed and determinism.
					// Returns ASCII in AL, sets ZF for AH=01h, and avoids BIOS INT 7h buffer logic.
					if (g_scripted_input && (ah == 0x00 || ah == 0x01))
					{
						if (ah == 0x01)
						{
							if (!scripted_input_has_key())
							{
								regs8[idx(Flag::ZF)] = 1;
								reg_ip += 2;
								break;
							}
							const u16 kw = scripted_input_peek();
							regs8[idx(Flag::ZF)] = 0;
							regs8[idx(Reg8::AL)] = (u8)(kw & 0xFF);
							regs8[idx(Reg8::AH)] = 0;
							reg_ip += 2;
							break;
						}

						// AH=00h: read key; if no key left, end run (avoid blocking forever).
						if (!scripted_input_has_key())
						{
							if (g_explore_mode)
							{
								if (g_explore_warmup_active)
								{
									// Warmup should be non-blocking: many DOS boot menus wait for a key.
									// Synthesize ENTER to accept the default path and keep booting.
									regs8[idx(Reg8::AL)] = 0x0D;
									regs8[idx(Reg8::AH)] = 0;
									reg_ip += 2;
									break;
								}
								// Exploration run: stop the host run without blocking forever.
								g_explore_stop_requested = true;
								break;
							}
							// Non-explore scripted mode: end run (avoid blocking forever).
							regs16[idx(Reg16::CS)] = 0;
							reg_ip = 0;
							break;
						}
						const u16 kw = scripted_input_pop();
						regs8[idx(Reg8::AL)] = (u8)(kw & 0xFF);
						regs8[idx(Reg8::AH)] = 0;
						reg_ip += 2;
						break;
					}
				}

				reg_ip += 2;
				pc_interrupt(i_data0);
				break;
			case OpcodeGroup::INTO:
				++reg_ip;
				regs8[idx(Flag::OF)] && pc_interrupt(4);
				break;
			case OpcodeGroup::AAM:
				if (i_data0 &= 0xFF)
					regs8[idx(Reg8::AH)] = regs8[idx(Reg8::AL)] / i_data0,
					op_result = regs8[idx(Reg8::AL)] %= i_data0;
				else // Divide by zero
					pc_interrupt(0);
				break;
			case OpcodeGroup::AAD:
				i_w = 0;
				regs16[idx(Reg16::AX)] = op_result = 0xFF & regs8[idx(Reg8::AL)] + i_data0 * regs8[idx(Reg8::AH)];
				break;
			case OpcodeGroup::SALC:
				regs8[idx(Reg8::AL)] = -regs8[idx(Flag::CF)];
				break;
			case OpcodeGroup::XLAT:
				regs8[idx(Reg8::AL)] = mem[segreg(seg_override_en ? (u32)seg_override : (u32)idx(Reg16::DS), (u16)(regs16[idx(Reg16::BX)] + regs8[idx(Reg8::AL)]))];
				break;
			case OpcodeGroup::CMC:
				regs8[idx(Flag::CF)] ^= 1;
				break;
			case OpcodeGroup::CLC_STC_CLI_STI_CLD_STD:
				regs8[extra / 2] = extra & 1;
				break;
			case OpcodeGroup::TEST_AL_AX_IMM:
				R_M_OP(regs8[idx(Reg8::AL)], &, i_data0);
				break;
			case OpcodeGroup::EMULATOR_SPECIFIC:
				switch (static_cast<EmulatorOp>((char)i_data0))
				{
					case EmulatorOp::PUTCHAR_AL:
					{
						const u8 ch = regs8[idx(Reg8::AL)];
						explore_observe_tty_char(ch);
						if (!g_quiet_output)
						{
							// Guardrail: loud boot logs can be enormous; allow capping to keep the terminal responsive.
							if (g_tty_limit_bytes && !g_tty_limit_tripped && g_tty_bytes_written >= g_tty_limit_bytes)
							{
								g_tty_limit_tripped = true;
								g_quiet_output = true;
								std::fprintf(stderr, "TTY output capped at %u bytes; switching to --quiet. Use --tty-limit 0 to disable.\n", (unsigned)g_tty_limit_bytes);
							}
							else
							{
								platform::write_fd(1, &ch, 1);
								++g_tty_bytes_written;
							}
						}
						break;
					}
						break;
					case EmulatorOp::GET_RTC:
#ifdef DETERMINISTIC_RTC
						// Use deterministic values that increment, for testing
						{
							static struct tm fixed_tm = {0, 0, 12, 15, 0, 125, 3, 14, 0};
							static short fixed_ms = 0;
							fixed_ms = (fixed_ms + 100) % 1000;  // Increment by 100ms each call
							if (fixed_ms == 0) fixed_tm.tm_sec++;  // Increment seconds
							memcpy(mem + segreg(idx(Reg16::ES), (u16)regs16[idx(Reg16::BX)]), &fixed_tm, sizeof(struct tm));
							ref<short>(mem[segreg(idx(Reg16::ES), (u16)(regs16[idx(Reg16::BX)] + 36))]) = fixed_ms;
						}
#else
						time(&clock_buf);
						ftime(&ms_clock);
						memcpy(mem + segreg(idx(Reg16::ES), (u16)regs16[idx(Reg16::BX)]), localtime(&clock_buf), sizeof(struct tm));
						ref<short>(mem[segreg(idx(Reg16::ES), (u16)(regs16[idx(Reg16::BX)] + 36))]) = ms_clock.millitm;
#endif
						break;
					case EmulatorOp::DISK_READ:
					case EmulatorOp::DISK_WRITE:
					{
						const bool is_write = static_cast<EmulatorOp>((char)i_data0) == EmulatorOp::DISK_WRITE;
						if (is_write)
							++g_disk_writes;
						else
							++g_disk_reads;
						g_last_diskio_inst = inst_counter;
						const u8 dl = regs8[idx(Reg8::DL)];
						const int fd = (dl & 0x80u) ? disk[0] : disk[1];
						const unsigned int byte_count = regs16[idx(Reg16::AX)];
						u8 *buf = mem + segreg(idx(Reg16::ES), (u16)regs16[idx(Reg16::BX)]);

						if (byte_count > RAM_SIZE)
						{
							regs16[idx(Reg16::AX)] = 0;
							break;
						}
						if ((buf < mem) || (buf + byte_count > mem + RAM_SIZE))
						{
							regs16[idx(Reg16::AX)] = 0;
							break;
						}

						// Match original C semantics:
						// - offset uses a 32-bit sector index sourced from SI:BP
						// - seek failure returns AL=0
						// - read/write failure returns AL=0xFF (via u8 cast of -1)
						const u32 lba = ref<u32>(regs8[2u * idx(Reg16::BP)]);
						const std::uint64_t byte_offset = (std::uint64_t)lba << 9;
						if (g_debug_diskio && g_debug_diskio_left)
						{
							--g_debug_diskio_left;
							std::fprintf(stderr, "DISK_%s dl=%u fd=%d lba=%u off=%llu bytes=%u\n",
								is_write ? "WRITE" : "READ",
								(unsigned)regs8[idx(Reg8::DL)],
								fd,
								(unsigned)lba,
								(unsigned long long)byte_offset,
								(unsigned)byte_count);
						}
						if (fd <= 0)
						{
							if (g_debug_diskio && g_debug_diskio_left)
							{
								--g_debug_diskio_left;
								std::fprintf(stderr, "DISK_%s attempted on missing drive: dl=%u\n",
									is_write ? "WRITE" : "READ",
									(unsigned)dl);
							}
							regs16[idx(Reg16::AX)] = 0;
							break;
						}
						const std::int64_t pos = platform::seek_set_bytes(fd, byte_offset);
						if (pos == (std::int64_t)-1)
						{
							regs16[idx(Reg16::AX)] = 0;
						}
						else
						{
							const std::int64_t rv = is_write
								? platform::write_fd(fd, buf, (std::size_t)byte_count)
								: platform::read_fd(fd, buf, (std::size_t)byte_count);
							regs16[idx(Reg16::AX)] = (rv > 0) ? (u16)rv : 0;
						}
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

		// Poll timer/keyboard periodically; explore warmup can run with a higher tick rate to
		// reach a usable prompt sooner (many boot paths wait on timer ticks).
		const u32 timer_delay = (g_explore_mode && g_explore_warmup_active)
			? kExploreWarmupKeyboardTimerUpdateDelay
			: kKeyboardTimerUpdateDelay;
		if (!(++inst_counter % timer_delay))
			int8_asap = 1;

		maybe_print_progress(inst_counter);

		// Exploration warmup: also look for the DOS prompt in text VRAM.
		// Keep the scan infrequent to avoid overhead.
		if (g_explore_mode && g_explore_warmup_active && !g_explore_prompt_seen)
		{
			if (!(inst_counter % 20000u))
				explore_observe_textmode_prompt();
		}

		if (g_max_instructions && inst_counter >= g_max_instructions)
		{
			std::fprintf(stderr,
				"STOP: reached --max-inst=%u at inst=%u CS:IP=%04X:%04X IF=%u TF=%u\n",
				(unsigned)g_max_instructions,
				(unsigned)inst_counter,
				(unsigned)regs16[idx(Reg16::CS)],
				(unsigned)reg_ip,
				(unsigned)regs8[idx(Flag::IF)],
				(unsigned)regs8[idx(Flag::TF)]);
			break;
		}

		if (g_explore_stop_requested)
		{
			g_explore_stop_requested = false;
			break;
		}


	if (g_host_stop_signal)
	{
		std::fprintf(stderr, "Stopping due to signal %d; dumping outputs...\n", (int)g_host_stop_signal);
		if (g_explore_mode)
			finalize_run_coverage();
		g_explore_mode = false;
	}
#if SNAP_AT
		if (inst_counter == (u32)SNAP_AT)
		{
			std::fprintf(stderr,
				"SNAP_AT=%u CS:IP=%04X:%04X raw=%02X xlat=%u extra=%u w=%u d=%u IF=%u TF=%u seg_ovr=%u rep_ovr=%u int8=%u\n",
				(unsigned)inst_counter,
				(unsigned)regs16[idx(Reg16::CS)],
				(unsigned)reg_ip,
				(unsigned)raw_opcode_id,
				(unsigned)xlat_opcode_id,
				(unsigned)extra,
				(unsigned)i_w,
				(unsigned)i_d,
				(unsigned)regs8[idx(Flag::IF)],
				(unsigned)regs8[idx(Flag::TF)],
				(unsigned)seg_override_en,
				(unsigned)rep_override_en,
				(unsigned)int8_asap);
			break;
		}
#endif

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

	if (g_explore_mode)
	{
		if (g_explore_warmup_active)
		{
			// If we hit the warmup instruction budget without seeing a prompt, auto-extend a few times
			// (unless the user explicitly set --explore-warmup-inst).
			if (!g_explore_prompt_seen && !g_explore_warmup_inst_user_set && g_max_instructions && inst_counter >= g_max_instructions && explore_warmup_ext_left)
			{
				--explore_warmup_ext_left;
				const u32 prev = g_max_instructions;
				const u32 next = (prev < 500000000u) ? (prev * 2u) : 500000000u;
				if (next > prev)
				{
					g_max_instructions = next;
					std::fprintf(stderr,
						"Explore warmup: extending budget to %u instructions (boot not ready yet) CS:IP=%04X:%04X IF=%u TF=%u disk_r=%llu disk_w=%llu last_disk_inst=%u\n",
						(unsigned)g_max_instructions,
						(unsigned)regs16[idx(Reg16::CS)],
						(unsigned)reg_ip,
						(unsigned)regs8[idx(Flag::IF)],
						(unsigned)regs8[idx(Flag::TF)],
						(unsigned long long)g_disk_reads,
						(unsigned long long)g_disk_writes,
						(unsigned)g_last_diskio_inst);
					goto run_loop;
				}
			}

			// Warmup just ended: snapshot state (ideally at DOS prompt), then reset instrumentation
			// so exploration measures only from the prompt onward.
			g_explore_warmup_active = false;
			if (!g_explore_prompt_seen)
			{
				std::fprintf(stderr,
					"Warning: explore warmup ended without detecting a DOS prompt.\n"
					"  Last CS:IP=%04X:%04X IF=%u TF=%u disk_r=%llu disk_w=%llu last_disk_inst=%u\n"
					"  Hint: increase --explore-warmup-inst, run with --progress, or validate BIOS/floppy images.\n",
					(unsigned)regs16[idx(Reg16::CS)],
					(unsigned)reg_ip,
					(unsigned)regs8[idx(Flag::IF)],
					(unsigned)regs8[idx(Flag::TF)],
					(unsigned long long)g_disk_reads,
					(unsigned long long)g_disk_writes,
					(unsigned)g_last_diskio_inst);
				if (!g_explore_tty_tail.empty())
					std::fprintf(stderr, "  TTY tail (may be partial): %s\n", g_explore_tty_tail.c_str());
			}
			g_explore_tty_tail.clear();
			base_snapshot = make_snapshot();

			std::fill(g_global_coverage.begin(), g_global_coverage.end(), 0);
			std::fill(g_run_coverage.begin(), g_run_coverage.end(), 0);
			g_run_touched.clear();
			g_last_run_gain = 0;
			g_decision_points.clear();
			if (!g_instr_map.empty())
				g_instr_map.assign(PHYS_MEM_SIZE, InstrInfo{});

			explore_corpus.clear();
			// Seed: launch a program from the boot floppy (typically already at A:>).
			{
				std::string prog = "GORILLA";
				if (g_run_exe_host_path)
				{
					std::string base = std::filesystem::path(g_run_exe_host_path).filename().string();
					char name83[11];
					if (make_dos_83_upper(base, name83))
					{
						prog.assign(name83, name83 + 8);
						while (!prog.empty() && prog.back() == ' ')
							prog.pop_back();
					}
				}
				std::vector<u8> seed;
				for (char c : prog)
					seed.push_back((u8)c);
				seed.push_back(0x0D);
				explore_corpus.push_back(std::move(seed));
			}
			if (!explore_user_seed_ascii.empty())
				explore_corpus.push_back(explore_user_seed_ascii);

			explore_iter = 0;
			explore_best_gain = 0;
			explore_best_seq.clear();

			g_max_instructions = explore_saved_max_inst;
			restore_snapshot(base_snapshot);
			explore_current = explore_corpus.front();
			g_scripted_input = true;
			g_scripted_keys = make_key_words_from_ascii(explore_current);
			g_scripted_key_pos = 0;
			goto run_loop;
		}

		const u32 gain = g_last_run_gain;
		finalize_run_coverage();

		if (gain > explore_best_gain)
		{
			explore_best_gain = gain;
			explore_best_seq = explore_current;
			if (g_explore_best_keys_path && !explore_best_seq.empty())
			{
				if (!save_keys_file(g_explore_best_keys_path, explore_best_seq))
					std::fprintf(stderr, "Warning: failed to save best keys to %s\n", g_explore_best_keys_path);
			}
		}

		if (gain > 0)
		{
			if (explore_corpus.size() < (std::size_t)g_explore_corpus_max)
				explore_corpus.push_back(explore_current);
			else
				explore_corpus[(std::size_t)(explore_rng() % explore_corpus.size())] = explore_current;
		}

		if ((explore_iter % 100u) == 0u)
			std::fprintf(stderr, "EXPLORE iter=%u corpus=%u last_gain=%u best_gain=%u\n",
				(unsigned)explore_iter,
				(unsigned)explore_corpus.size(),
				(unsigned)gain,
				(unsigned)explore_best_gain);

		// Periodic persistence to survive long runs / interrupts.
		if (g_explore_save_every && (explore_iter % g_explore_save_every) == 0u)
		{
			if (g_explore_best_keys_path && !explore_best_seq.empty())
				(void)save_keys_file(g_explore_best_keys_path, explore_best_seq);
		}
		if (g_explore_dump_every && (explore_iter % g_explore_dump_every) == 0u)
		{
			dump_instr_map_to_file(g_dump_instr_path);
			dump_decisions_to_file(g_dump_decisions_path);
		}

		++explore_iter;
		if (explore_iter < g_explore_iters)
		{
			restore_snapshot(base_snapshot);
			g_last_run_gain = 0;

			std::uniform_int_distribution<std::size_t> pick_parent(0, explore_corpus.size() - 1);
			const std::vector<u8> &parent = explore_corpus[pick_parent(explore_rng)];
			explore_current = mutate_keys(explore_rng, parent, explore_alphabet, g_explore_max_keys);

			g_scripted_input = true;
			g_scripted_keys = make_key_words_from_ascii(explore_current);
			g_scripted_key_pos = 0;
			goto run_loop;
		}

		std::fprintf(stderr, "EXPLORE done: iters=%u corpus=%u best_gain=%u\n",
			(unsigned)explore_iter,
			(unsigned)explore_corpus.size(),
			(unsigned)explore_best_gain);
		if (g_explore_best_keys_path && !explore_best_seq.empty())
			(void)save_keys_file(g_explore_best_keys_path, explore_best_seq);
	}

	dump_instr_map_to_file(g_dump_instr_path);
	dump_decisions_to_file(g_dump_decisions_path);

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
