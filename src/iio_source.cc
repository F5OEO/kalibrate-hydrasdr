/**
 * @file iio_source.cc
 * @brief Implementation of AD96x iio source with DSP resampling pipeline.
 *
 * This file implements the iio_source class, providing:
 * - Hardware initialization and configuration via IIO
 * - Asynchronous USB or Ethernet sample reception with callback handling
 * - Integrated two-stage DSP resampling (2.5 MSPS → 270.833 kSPS)
 * - Thread-safe producer/consumer buffering
 *
 * @section DSP Pipeline
 *
 * The resampling pipeline converts the native hardware rate to GSM symbol rate:
 *
 * @code
 *   2,500,000 Hz ─▶ [Stage 1: ÷5] ─▶ 500,000 Hz ─▶ [Stage 2: ×13/24] ─▶ 270,833.333 Hz
 *                   (61-tap LPF)                   (729-tap Polyphase)
 * @endcode
 *
 * @see dsp_resampler for filter coefficient details.
 *
 * @author Benjamin Vernoux <bvernoux@hydrasdr.com>
 * @copyright 2025 Benjamin Vernoux
 * @author adapt 2025 Evariste F5OEO
 * @license BSD-2-Clause
 */

#ifdef _WIN32
#include "win_compat.h"
#endif

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include <algorithm>
#include <signal.h>
#include <iostream>

#include "iio_source.h"

extern volatile sig_atomic_t g_kal_exit_req;

iio_source::iio_source(float gain, const char* uri)
{
	m_gain = gain;
	if (uri) m_uri = std::string(uri);

	// Target GSM symbol rate
	m_sample_rate = 270833.333333;
	m_center_freq = 0.0;
	m_freq_corr = 0;
	m_overflow_count = 0;

	m_ctx = NULL;
	m_dev = NULL;
	m_phy = NULL;
	m_rx0_i = NULL;
	m_rx0_q = NULL;
	m_rxbuf = NULL;
	cb = NULL;
	streaming = false;

	m_resampler = new dsp_resampler();
}

iio_source::~iio_source()
{
	close();
	delete m_resampler;
}

int iio_source::open(void)
{
	if (!m_uri.empty()) {
		m_ctx = iio_create_context_from_uri(m_uri.c_str());
	} else {
		m_ctx = iio_create_default_context();
	}

	if (!m_ctx) {
		fprintf(stderr, "Error: Failed to create IIO context. Is PlutoSDR connected?\n");
		return -1;
	}

	m_dev = iio_context_find_device(m_ctx, "cf-ad9361-lpc");
	m_phy = iio_context_find_device(m_ctx, "ad9361-phy");

	if (!m_dev || !m_phy) {
		fprintf(stderr, "Error: Failed to find AD9361 devices.\n");
		return -1;
	}

	m_rx0_i = iio_device_find_channel(m_dev, "voltage0", false);
	m_rx0_q = iio_device_find_channel(m_dev, "voltage1", false);

	if (!m_rx0_i || !m_rx0_q) {
		fprintf(stderr, "Error: Failed to find RX channels.\n");
		return -1;
	}

	iio_channel_enable(m_rx0_i);
	iio_channel_enable(m_rx0_q);

	// Set Sample Rate to 2.5 MSPS for DSP pipeline compatibility
	long long rate = IIO_2_5MSPS_NATIVE_RATE;
	if (iio_channel_attr_write_longlong(iio_device_find_channel(m_phy, "voltage0", false), "sampling_frequency", rate) < 0) {
		fprintf(stderr, "Warning: Failed to set sampling rate to 2.5 MSPS.\n");
	}

	// Set Gain Mode to Manual
	iio_channel_attr_write(iio_device_find_channel(m_phy, "voltage0", false), "gain_control_mode", "manual");

	set_gain(m_gain);

	try {
		cb = new circular_buffer(256 * 1024, sizeof(complex));
	} catch (const std::exception& e) {
		fprintf(stderr, "Failed to allocate circular buffer: %s\n", e.what());
		return -1;
	}

	return 0;
}

int iio_source::close()
{
	stop();

	if (m_rxbuf) { iio_buffer_destroy(m_rxbuf); m_rxbuf = NULL; }
	if (m_ctx) { iio_context_destroy(m_ctx); m_ctx = NULL; }
	if (cb) { delete cb; cb = NULL; }

	return 0;
}

int iio_source::tune(double freq)
{
	if (!m_phy) return -1;

	long long freq_ll = (long long)freq;
	if (iio_channel_attr_write_longlong(iio_device_find_channel(m_phy, "altvoltage0", true), "frequency", freq_ll) < 0) {
		fprintf(stderr, "Failed to tune to %.0f Hz\n", freq);
		return -1;
	}

	m_center_freq = freq;
	m_resampler->reset();
	return 0;
}

int iio_source::set_gain(float gain)
{
	if (!m_phy) return -1;
	m_gain = gain;
	
	// Map 0-21 range roughly to hardware gain (0-70dB) or use directly if user provides dB
	// Assuming user provides dB for Pluto (0-70)
	long long gain_ll = (long long)gain;
	iio_channel_attr_write_longlong(iio_device_find_channel(m_phy, "voltage0", false), "hardwaregain", gain_ll);
	
	return 0;
}

int iio_source::start()
{
	if (!m_dev) return -1;

	m_resampler->reset();
	m_overflow_count = 0;

	// Create buffer: 128k samples
	m_rxbuf = iio_device_create_buffer(m_dev, 128 * 1024, false);
	if (!m_rxbuf) {
		fprintf(stderr, "Failed to create IIO buffer.\n");
		return -1;
	}

	streaming.store(true);
	m_worker = std::thread(&iio_source::worker_thread, this);

	return 0;
}

int iio_source::stop()
{
	if (streaming.load()) {
		streaming.store(false);
		if (m_worker.joinable()) {
			m_worker.join();
		}
		if (m_rxbuf) {
			iio_buffer_destroy(m_rxbuf);
			m_rxbuf = NULL;
		}
		data_ready.notify_all();
	}
	return 0;
}

void iio_source::start_benchmark()
{
	if (!cb) cb = new circular_buffer(256 * 1024, sizeof(complex));
	m_resampler->reset();
	m_overflow_count = 0;
	streaming.store(true);
}

void iio_source::worker_thread()
{
	const float scale = 1.0f / 2048.0f; // 12-bit ADC

	while (streaming.load()) {
		ssize_t nbytes = iio_buffer_refill(m_rxbuf);
		if (nbytes < 0) break;

		void *start = iio_buffer_first(m_rxbuf, m_rx0_i);
		void *end = iio_buffer_end(m_rxbuf);
		ptrdiff_t step = iio_buffer_step(m_rxbuf);

		size_t count = 0;
		// Convert interleaved int16 to complex float batch
		for (void *p = start; p < end && count < BATCH_SIZE; p = (void*)((char*)p + step)) {
			int16_t i = ((int16_t*)p)[0];
			int16_t q = ((int16_t*)p)[1];
			m_batch_buffer[count++] = std::complex<float>(i * scale, q * scale);
		}

		// Run DSP Pipeline
		// Note: We are reusing m_batch_buffer for input here, which is safe if we process in chunks
		// But dsp_resampler expects separate input/output buffers.
		// Let's allocate a temp output buffer on stack or class
		std::complex<float> out_buf[BATCH_SIZE];
		size_t produced = m_resampler->process(m_batch_buffer, count, out_buf, BATCH_SIZE);

		if (produced > 0) {
			std::unique_lock<std::mutex> lock(data_mutex, std::defer_lock);
			if (lock.try_lock()) {
				if (cb) {
					unsigned int written = cb->write(out_buf, produced);
					if (written < produced) m_overflow_count += (produced - written);
				}
				lock.unlock();
				data_ready.notify_one();
			} else {
				m_overflow_count += produced;
			}
		}
	}
}

int iio_source::fill(unsigned int num_samples, unsigned int *overruns)
{
	if (!cb) return -1;
	if (!streaming.load()) start();

	std::unique_lock<std::mutex> lock(data_mutex);
	while (true) {
		if (g_kal_exit_req) return -1;
		if ((cb->data_available() >= num_samples) || !streaming.load()) break;
		data_ready.wait_for(lock, std::chrono::milliseconds(100));
	}
	if (!streaming.load()) return -1;
	if (overruns) *overruns = m_overflow_count.exchange(0);
	return 0;
}

int iio_source::flush() { if (cb) cb->flush(); m_overflow_count = 0; return 0; }