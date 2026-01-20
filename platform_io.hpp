#pragma once

#include <cstddef>
#include <cstdint>

#if defined(_WIN32)
	#include <io.h>
	#include <fcntl.h>
	#include <sys/stat.h>
	#include <sys/types.h>
#else
	#include <unistd.h>
	#include <fcntl.h>
	#include <sys/types.h>
#endif

#ifndef O_BINARY
	#define O_BINARY 0
#endif
#ifndef O_LARGEFILE
	#define O_LARGEFILE 0
#endif
#ifndef O_CLOEXEC
	#define O_CLOEXEC 0
#endif

namespace platform
{
	inline constexpr int kOpenDiskFlags = (O_BINARY | O_LARGEFILE | O_CLOEXEC | O_RDWR);
	inline constexpr int kOpenDiskFlagsFallback = (O_BINARY | O_LARGEFILE | O_CLOEXEC | O_RDONLY);

	inline std::int64_t read_fd(int fd, void *buffer, std::size_t byte_count)
	{
#if defined(_WIN32)
		const unsigned int count = (unsigned int)byte_count;
		return (std::int64_t)_read(fd, buffer, count);
#else
		return (std::int64_t)::read(fd, buffer, byte_count);
#endif
	}

	inline std::int64_t write_fd(int fd, const void *buffer, std::size_t byte_count)
	{
#if defined(_WIN32)
		const unsigned int count = (unsigned int)byte_count;
		return (std::int64_t)_write(fd, buffer, count);
#else
		return (std::int64_t)::write(fd, buffer, byte_count);
#endif
	}

	inline std::int64_t seek_end_bytes(int fd)
	{
#if defined(_WIN32)
		const auto pos = _lseeki64(fd, 0, SEEK_END);
		return (std::int64_t)pos;
#else
		const auto pos = ::lseek(fd, 0, SEEK_END);
		return (std::int64_t)pos;
#endif
	}

	inline std::int64_t seek_set_bytes(int fd, std::uint64_t offset)
	{
#if defined(_WIN32)
		const auto pos = _lseeki64(fd, (std::int64_t)offset, SEEK_SET);
		return (std::int64_t)pos;
#else
		const auto pos = ::lseek(fd, (off_t)offset, SEEK_SET);
		return (std::int64_t)pos;
#endif
	}

	inline int open_disk_image(const char *path)
	{
#if defined(_WIN32)
		int fd = _open(path, kOpenDiskFlags);
		if (fd < 0)
			fd = _open(path, kOpenDiskFlagsFallback);
		return fd;
#else
		int fd = ::open(path, kOpenDiskFlags);
		if (fd < 0)
			fd = ::open(path, kOpenDiskFlagsFallback);
		return fd;
#endif
	}

	inline int close_fd(int fd)
	{
#if defined(_WIN32)
		return _close(fd);
#else
		return ::close(fd);
#endif
	}

	// Read/write at a byte offset. Returns number of bytes transferred, or -1 on error.
	inline std::int64_t rw_at_offset(int fd, void *buffer, std::size_t byte_count, std::uint64_t offset, bool is_write)
	{
#if !defined(_WIN32)
		const auto count = (std::size_t)byte_count;
		const auto off = (off_t)offset;
		const ssize_t rv = is_write
			? ::pwrite(fd, buffer, count, off)
			: ::pread(fd, buffer, count, off);
		return (std::int64_t)rv;
#else
		if (_lseeki64(fd, (std::int64_t)offset, SEEK_SET) == (std::int64_t)-1)
			return -1;
		const unsigned int count = (unsigned int)byte_count;
		const int rv = is_write ? _write(fd, buffer, count) : _read(fd, buffer, count);
		return (std::int64_t)rv;
#endif
	}
}
