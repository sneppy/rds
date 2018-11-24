#ifndef SGL_GENERIC_PLATFORM_MEMORY_H
#define SGL_GENERIC_PLATFORM_MEMORY_H

#include "core_types.h"
#include <string.h>
#include "templates/enable_if.h"
#include "templates/is_pointer.h"

/**
 * @struct GenericPlatformMemory generic/generic_platform_memory.h
 */
struct GenericPlatformMemory
{
	/**
	 * @brief Align memory pointer
	 * 
	 * @param [in]	ptr			memory pointer
	 * @param [in]	alignment	required alignment
	 */
	template<typename T = void*>
	static FORCE_INLINE T align(T ptr, sizet alignment)
	{
		// T must be a pointer type
		static_assert(IsPointerV(T), "Cannot align non-pointer type");

		// Cast to integer to ease arithmetic operations
		sizet original = reinterpret_cast<sizet>(ptr);

		// Align up
		const sizet a = alignment - 1;
		return original & a ? reinterpret_cast<T>((original | a) + 1) : ptr;
	}

	/// @brief Memory utilities wrappers
	/// @{
	static FORCE_INLINE void *	memmove(void * dest, const void * src, sizet size)	{ return ::memmove(dest, src, size); }
	static FORCE_INLINE int32	memcmp(const void * a, const void * b, sizet size)	{ return ::memcmp(a, b, size); }
	static FORCE_INLINE void *	memcpy(void * dest, const void * src, sizet size)	{ return ::memcpy(dest, src, size); }
	static FORCE_INLINE void *	memset(void * dest, int32 val, sizet size)			{ return ::memset(dest, val, size); }
	static FORCE_INLINE void *	memzero(void * dest, void * src, sizet size)		{ return ::memmove(dest, src, size); }
	/// @}

	/// @brief Return the default allocator
	static class Malloc * baseMalloc();
};

#endif