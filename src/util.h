/**
 * @file util.h
 * @brief Header for utility functions.
 *
 * Provides:
 * - Platform-independent aligned memory allocation
 * - DSP benchmark and ASCII FFT visualization
 * - Helper functions for statistics and display
 *
 * @author Joshua Lackey (original)
 * @author Benjamin Vernoux <bvernoux@hydrasdr.com> (improvements)
 * @copyright 2010 Joshua Lackey, 2025 Benjamin Vernoux
 * @license BSD-2-Clause
 */

#ifndef __UTIL_H__
#define __UTIL_H__

#include <complex>
#include <vector>
#include <cstdlib>

/*
 * ---------------------------------------------------------------------------
 * ALIGNED MEMORY ALLOCATION UTILITIES
 * ---------------------------------------------------------------------------
 * Platform-specific aligned allocation support for SIMD-friendly data structures.
 * Required for types with alignas(64) when using C++11/14 (pre-C++17 aligned new).
 */

/** @brief Default alignment for SIMD buffers (cache line size). */
#define DEFAULT_ALIGNMENT 64

#if defined(__MINGW32__) && !defined(__MINGW64_VERSION_MAJOR)
	/* MinGW 32-bit */
	#include <malloc.h>
	#define aligned_alloc_impl(size, alignment) __mingw_aligned_malloc(size, alignment)
	#define aligned_free_impl(mem) __mingw_aligned_free(mem)

#elif defined(__APPLE__)
	/* macOS - use posix_memalign */
	#include <stdlib.h>
	static inline void* aligned_alloc_impl(size_t size, size_t alignment) {
		void* result = NULL;
		if (posix_memalign(&result, alignment, size) == 0)
			return result;
		return NULL;
	}
	#define aligned_free_impl(mem) free(mem)

#elif defined(__FreeBSD__)
	/* FreeBSD - use posix_memalign */
	#include <cstdlib>
	static inline void* aligned_alloc_impl(size_t size, size_t alignment) {
		void* result = NULL;
		if (posix_memalign(&result, alignment, size) == 0)
			return result;
		return NULL;
	}
	#define aligned_free_impl(mem) free(mem)

#elif defined(_MSC_VER) || (defined(_WIN32) && defined(__MINGW64_VERSION_MAJOR))
	/* MSVC and MinGW-w64 */
	#include <malloc.h>
	#define aligned_alloc_impl(size, alignment) _aligned_malloc(size, alignment)
	#define aligned_free_impl(mem) _aligned_free(mem)

#elif defined(__GNUC__)
	/* GCC/Linux - use posix_memalign */
	#include <malloc.h>
	#include <cstdlib>
	static inline void* aligned_alloc_impl(size_t size, size_t alignment) {
		void* result = NULL;
		if (posix_memalign(&result, alignment, size) == 0)
			return result;
		return NULL;
	}
	#define aligned_free_impl(mem) free(mem)

#else
	/* Fallback: standard allocation (may not meet alignment requirements) */
	#warning "No platform-specific aligned allocator found, using standard malloc"
	#define aligned_alloc_impl(size, alignment) malloc(size)
	#define aligned_free_impl(mem) free(mem)
#endif

/* MSVC alignment attribute */
#if defined(_MSC_VER)
	#define ALIGNED __declspec(align(DEFAULT_ALIGNMENT))
#else
	#define ALIGNED
#endif

/**
 * @brief Allocates aligned memory.
 * @param size      Size in bytes to allocate.
 * @param alignment Required alignment (must be power of 2).
 * @return Pointer to allocated memory or NULL on failure.
 */
static inline void* aligned_malloc(size_t size, size_t alignment = DEFAULT_ALIGNMENT)
{
	return aligned_alloc_impl(size, alignment);
}

/**
 * @brief Frees memory allocated by aligned_malloc().
 * @param ptr Pointer to memory block (NULL-safe).
 */
static inline void aligned_free(void* ptr)
{
	if (ptr) {
		aligned_free_impl(ptr);
	}
}

/*
 * ---------------------------------------------------------------------------
 * DSP BENCHMARK AND VISUALIZATION
 * ---------------------------------------------------------------------------
 */

/**
 * @brief Runs DSP pipeline benchmark with synthetic data.
 *
 * Generates 5 seconds of test signal at 2.5 MSPS, processes through
 * the resampling pipeline, and measures throughput.
 */
void run_dsp_benchmark();

/**
 * @brief Draws ASCII spectrum visualization.
 *
 * Uses FFTW to compute spectrum, then renders as colored ASCII bars.
 * Useful for visual debugging of signal quality.
 *
 * @param data        Complex sample buffer.
 * @param len         Number of samples (used as FFT size).
 * @param width       Display width in characters (default 70).
 * @param sample_rate Sample rate in Hz for frequency axis (default 0 = bins only).
 */
void draw_ascii_fft(const std::complex<float> *data, int len,
		    int width = 70, float sample_rate = 0.0f);

/*
 * ---------------------------------------------------------------------------
 * HELPER FUNCTIONS
 * ---------------------------------------------------------------------------
 */

/**
 * @brief Displays frequency in human-readable format.
 * @param f Frequency in Hz.
 */
void display_freq(float f);

/**
 * @brief Sorts float array in ascending order.
 * @param data Array to sort.
 * @param len  Array length.
 * @return 0 on success.
 */
int sort(float *data, int len);

/**
 * @brief Computes mean and standard deviation.
 * @param data   Input array.
 * @param len    Array length.
 * @param stddev Output: standard deviation (may be NULL).
 * @return Mean value.
 */
double avg(float *data, int len, double *stddev);

#endif /* __UTIL_H__ */
