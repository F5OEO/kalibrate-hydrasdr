/**
 * @file dsp_resampler.h
 * @brief Header for DSP Resampler (Stage 1 Decimator + Stage 2 Polyphase).
 *
 * Two-stage rational resampling pipeline:
 *   2,500,000 Hz → [÷5] → 500,000 Hz → [×13/24] → 270,833.333 Hz
 *
 * @author Benjamin Vernoux <bvernoux@hydrasdr.com>
 * @copyright 2025 Benjamin Vernoux
 * @license BSD-2-Clause
 */

/*
 * Copyright 2025 Benjamin Vernoux <bvernoux@hydrasdr.com>
 * SPDX-License-Identifier: BSD-2-Clause
 */

#ifndef __DSP_RESAMPLER_H__
#define __DSP_RESAMPLER_H__

#include <complex>
#include <vector>
#include <cstddef>
#include <new>
#include "util.h"

/** @brief Stage 1 decimation factor. */
#define S1_DECIMATION 5

/** @brief Stage 1 FIR filter tap count. */
#define S1_TAPS 61

/** @brief Stage 2 interpolation factor. */
#define S2_INTERP 13

/** @brief Stage 2 decimation factor. */
#define S2_DECIM 24

/** @brief Stage 2 total prototype filter taps. */
#define S2_TAPS_TOTAL 729

/** @brief Stage 2 polyphase branch count (= interpolation factor). */
#define S2_PHASES 13

/** @brief Stage 2 taps per polyphase branch. */
#define S2_TAPS_PER_PHASE 57

/**
 * @class dsp_resampler
 * @brief Two-stage rational resampler optimized for SIMD processing.
 *
 * Converts 2.5 MSPS input to 270.833 kSPS output using:
 * - Stage 1: Integer decimation by 5 with 61-tap anti-alias filter
 * - Stage 2: Polyphase rational resampling (13/24) with 729-tap prototype
 *
 * Internal buffers use alignas(64) for SIMD-friendly memory alignment.
 * Custom operator new/delete ensures proper alignment when heap-allocated.
 */
class dsp_resampler {
public:
	dsp_resampler();
	~dsp_resampler();

	/**
	 * @brief Resets the internal filter state.
	 *
	 * Call this when retuning to prevent filter transients from
	 * the old frequency from contaminating the new signal.
	 */
	void reset();

	/**
	 * @brief Processes a block of input samples.
	 *
	 * @param in         Pointer to input samples (2.5 MSPS complex float).
	 * @param in_count   Number of input samples.
	 * @param out_buffer Pointer to destination buffer.
	 * @param out_cap    Capacity of destination buffer in samples.
	 * @return Number of samples written to out_buffer.
	 *
	 * @note Output rate is approximately in_count / 9.23 samples.
	 *       Caller must ensure out_cap >= in_count / 9 to avoid data loss.
	 *
	 * @warning If out_buffer fills before all input is processed,
	 *          remaining input samples are LOST. Size buffers appropriately.
	 */
	size_t process(const std::complex<float>* in, size_t in_count,
		       std::complex<float>* out_buffer, size_t out_cap);

	/**
	 * @brief Custom aligned operator new for SIMD-friendly allocation.
	 *
	 * This class uses alignas(64) for internal buffers to enable SIMD
	 * optimizations. C++11/14 default operator new does not guarantee
	 * 64-byte alignment, so we provide custom operators.
	 *
	 * @param size Size in bytes to allocate.
	 * @return Pointer to aligned memory block.
	 * @throws std::bad_alloc if allocation fails.
	 */
	static void* operator new(size_t size) {
		void* ptr = aligned_malloc(size, DEFAULT_ALIGNMENT);
		if (!ptr) {
			throw std::bad_alloc();
		}
		return ptr;
	}

	/**
	 * @brief Custom aligned operator delete.
	 * @param ptr Pointer to memory block allocated by operator new.
	 */
	static void operator delete(void* ptr) noexcept {
		aligned_free(ptr);
	}

	/**
	 * @brief Custom aligned operator new[] for array allocation.
	 * @param size Size in bytes to allocate.
	 * @return Pointer to aligned memory block.
	 * @throws std::bad_alloc if allocation fails.
	 */
	static void* operator new[](size_t size) {
		void* ptr = aligned_malloc(size, DEFAULT_ALIGNMENT);
		if (!ptr) {
			throw std::bad_alloc();
		}
		return ptr;
	}

	/**
	 * @brief Custom aligned operator delete[] for array deallocation.
	 * @param ptr Pointer to memory block allocated by operator new[].
	 */
	static void operator delete[](void* ptr) noexcept {
		aligned_free(ptr);
	}

private:
	/*
	 * Stage 1 State (Decimator)
	 */

	/** @brief Decimation counter (0 to S1_DECIMATION-1). */
	int s1_index;

	/**
	 * @brief Double-sized history buffer for linear convolution.
	 *
	 * Samples are written at both [head] and [head + S1_TAPS] to enable
	 * contiguous memory access during convolution without modulo operations.
	 */
	alignas(64) std::complex<float> s1_history[2 * S1_TAPS];

	/** @brief Pre-reversed coefficients for forward-scan vectorization. */
	alignas(64) float s1_coeffs_rev[S1_TAPS];

	/** @brief Write position in history buffer. */
	int s1_head;

	/*
	 * Stage 2 State (Polyphase Resampler)
	 */

	/**
	 * @brief Polyphase filter banks (coefficients pre-reversed).
	 *
	 * Organized as [phase][tap] for cache-friendly access during
	 * the polyphase convolution inner loop.
	 */
	alignas(64) float s2_coeffs_poly[S2_PHASES][S2_TAPS_PER_PHASE];

	/** @brief Double-sized history buffer (same technique as Stage 1). */
	alignas(64) std::complex<float> s2_history[2 * S2_TAPS_PER_PHASE];

	/** @brief Write position in Stage 2 history buffer. */
	int s2_head;

	/** @brief Current polyphase phase accumulator. */
	int s2_phase_state;

	/*
	 * Internal Processing Functions
	 */

	/**
	 * @brief Processes one input sample through Stage 1.
	 *
	 * When S1_DECIMATION samples have been accumulated, produces one
	 * output sample and passes it to Stage 2.
	 */
	inline void push_stage1(std::complex<float> sample,
				std::complex<float>* out_buffer,
				size_t out_cap, size_t& out_produced);

	/**
	 * @brief Processes one sample through Stage 2 polyphase resampler.
	 *
	 * May produce 0 or 1 output samples depending on phase state.
	 */
	inline void push_stage2(std::complex<float> sample,
				std::complex<float>* out_buffer,
				size_t out_cap, size_t& out_produced);
};

#endif /* __DSP_RESAMPLER_H__ */
