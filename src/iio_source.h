/**
 * @file iio_source.h
 * @brief SDR source interface for IIO devices (PlutoSDR/AD936x).
 *
 * This module provides a high-level C++ interface for receiving samples from
 * AD936x  hardware. It handles device initialization, tuning, gain
 * control, and integrates a two-stage DSP resampling pipeline to convert
 * the native 2.5 MSPS sample rate to GSM-compatible 270.833 kSPS.
 *
 * @section Architecture
 *
 * @code
 *  ┌─────────────┐    ┌──────────────┐     ┌────────────────┐     ┌──────────┐
 *  │  AD9361x    │───▶│  IIO         │────▶│  DSP Pipeline  │───▶│ Circular │
 *  │  Hardware   │    │  (Callback)  │     │  (Resampler)   │     │  Buffer  │
 *  │  2.5 MSPS   │    │              │     │  270.833 kSPS  │     │          │
 *  └─────────────┘    └──────────────┘     └────────────────┘     └────┬─────┘
 *                                                                      │
 *                                                                      ▼
 *                                                               ┌──────────────┐
 *                                                               │  Main Thread │
 *                                                               │  (Consumer)  │
 *                                                               └──────────────┘
 * @endcode
 *
 * @section Threading Model
 *
 * - **USB Thread**: Invoked by HydraSDR driver via callback, runs DSP pipeline
 * - **Main Thread**: Consumes processed samples via fill() method
 * - **Synchronization**: std::mutex + std::condition_variable for thread-safe handoff
 *
 * @author Benjamin Vernoux <bvernoux@hydrasdr.com>
 * @copyright 2025 Benjamin Vernoux
 * @author adapt 2025 Evariste F5OEO
 * @license BSD-2-Clause
 */



#ifndef __IIO_SOURCE_H__
#define __IIO_SOURCE_H__

#include <vector>
#include <complex>
#include <mutex>
#include <atomic>
#include <condition_variable>
#include <thread>
#include <string>
#include <iio.h>
#include "circular_buffer.h"
#include "dsp_resampler.h"

typedef std::complex<float> complex;

#define IIO_2_5MSPS_NATIVE_RATE 2500000

class iio_source {
public:
	iio_source(float gain, const char* uri = nullptr);
	~iio_source();

	int open();
	int tune(double freq);
	int set_gain(float gain);
	int start();
	int stop();
	int close();
	void start_benchmark();

	inline double sample_rate() { return m_sample_rate; }
	inline circular_buffer* get_buffer() { return cb; }

	int fill(unsigned int num_samples, unsigned int *overruns);
	int flush();

	double m_center_freq;
	int m_freq_corr;

private:
	struct iio_context *m_ctx;
	struct iio_device *m_dev; // RX device (cf-ad9361-lpc)
	struct iio_device *m_phy; // PHY device (ad9361-phy)
	struct iio_channel *m_rx0_i;
	struct iio_channel *m_rx0_q;
	struct iio_buffer *m_rxbuf;

	circular_buffer* cb;
	std::condition_variable data_ready;
	std::mutex data_mutex;
	std::atomic<bool> streaming;
	std::thread m_worker;

	float m_gain;
	double m_sample_rate;
	std::atomic<unsigned int> m_overflow_count;
	dsp_resampler* m_resampler;
	std::string m_uri;

	static const int BATCH_SIZE = 32768;
	std::complex<float> m_batch_buffer[BATCH_SIZE];

	void worker_thread();
};

#endif /* __IIO_SOURCE_H__ */