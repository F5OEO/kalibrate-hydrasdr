/**
 * @file dsp_resampler.cc
 * @brief Implementation of Polyphase Rational Resampler.
 *
 * @author Benjamin Vernoux <bvernoux@hydrasdr.com>
 * @copyright 2025 Benjamin Vernoux
 * @license BSD-2-Clause
 */

/*
 * Copyright 2025 Benjamin Vernoux <bvernoux@hydrasdr.com>
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include "dsp_resampler.h"
#include <algorithm>
#include <cmath>
#include <cstring>

/*
 * STAGE 1 FIR COEFFICIENTS
 * ========================
 * Anti-aliasing lowpass filter for decimation stage.
 *
 * Purpose:     Bandwidth-limit signal before ÷5 decimation
 * Input Rate:  2,500,000 Hz (HYDRASDR_2_5MSPS_NATIVE_RATE)
 * Output Rate:   500,000 Hz
 * Decimation:  5
 *
 * Filter Specifications (Measured):
 *   Type:           Lowpass FIR, Linear Phase (symmetric)
 *   Taps:           61
 *   -3dB Cutoff:    111.5 kHz
 *   Passband:       0 - 100 kHz (GSM channel bandwidth)
 *   Stopband:       > 150 kHz
 *   DC Gain:        1.0 (unity, 0 dB)
 *   Stopband Atten: > 60 dB
 *
 * Design validated with scipy.signal.freqz() at Fs = 2.5 MHz.
 * See analyze_coeffs.py for coefficient verification.
 */
static const float S1_COEFFS[S1_TAPS] = {
    -0.00031204f, -0.00004545f, 0.00027904f, 0.00068462f, 0.00117369f, 0.00171261f, 0.00222291f, 0.00258239f, 0.00263792f, 0.00222986f,
    0.00122527f, -0.00044472f, -0.00274968f, -0.00553362f, -0.00850401f, -0.01124041f, -0.01322480f, -0.01389213f, -0.01269630f, -0.00918414f,
    -0.00306760f, 0.00571594f, 0.01696486f, 0.03020315f, 0.04470262f, 0.05953716f, 0.07366408f, 0.08602410f, 0.09564828f, 0.10175928f,
    0.10385425f, 0.10175928f, 0.09564828f, 0.08602410f, 0.07366408f, 0.05953716f, 0.04470262f, 0.03020315f, 0.01696486f, 0.00571594f,
    -0.00306760f, -0.00918414f, -0.01269630f, -0.01389213f, -0.01322480f, -0.01124041f, -0.00850401f, -0.00553362f, -0.00274968f, -0.00044472f,
    0.00122527f, 0.00222986f, 0.00263792f, 0.00258239f, 0.00222291f, 0.00171261f, 0.00117369f, 0.00068462f, 0.00027904f, -0.00004545f,
    -0.00031204f
};

/*
 * STAGE 2 FIR COEFFICIENTS (RAW)
 * ==============================
 * Polyphase rational resampler prototype filter for final rate conversion.
 *
 * Purpose:     Resample from intermediate rate to GSM symbol rate
 * Input Rate:    500,000 Hz (output of Stage 1)
 * Output Rate:   270,833.333... Hz (GSM 13 MHz / 48)
 * Ratio:         13/24 (interpolate by 13, decimate by 24)
 *
 * Filter Specifications (Measured):
 *   Type:           Lowpass FIR, Linear Phase (symmetric)
 *   Total Taps:     729 (S2_TAPS_TOTAL)
 *   Phases:         13  (S2_PHASES = interpolation factor)
 *   Taps/Phase:     57  (S2_TAPS_PER_PHASE, ceil(729/13))
 *   -3dB Cutoff:    163.2 kHz (at 6.5 MHz virtual rate)
 *   DC Gain:        13.0 (+22.3 dB, equals interpolation factor)
 *   Stopband Atten: > 80 dB
 *
 * Prototype filter analysis uses virtual sample rate = 500 kHz × 13 = 6.5 MHz.
 * The +22.3 dB gain compensates for energy spreading during interpolation.
 * Coefficients stored in sequential order, reorganized into polyphase
 * filter banks at runtime for efficient SIMD convolution.
 *
 * Design validated with scipy.signal.freqz() at Fs = 6.5 MHz.
 * See analyze_coeffs.py for coefficient verification.
 *
 * Overall Resampling Pipeline:
 *   2,500,000 Hz → [S1: ÷5] → 500,000 Hz → [S2: ×13/24] → 270,833.333 Hz
 *   Combined decimation ratio: 120/13 ≈ 9.23077
 */
static const float S2_COEFFS_RAW[S2_TAPS_TOTAL] = {
    0.00006223f, 0.00008348f, 0.00010558f, 0.00012822f, 0.00015103f, 0.00017364f, 0.00019563f, 0.00021657f, 0.00023602f, 0.00025352f,
    0.00026862f, 0.00028088f, 0.00028987f, 0.00029518f, 0.00029645f, 0.00029335f, 0.00028560f, 0.00027297f, 0.00025530f, 0.00023250f,
    0.00020457f, 0.00017156f, 0.00013363f, 0.00009102f, 0.00004406f, -0.00000685f, -0.00006118f, -0.00011837f, -0.00017773f, -0.00023854f,
    -0.00029997f, -0.00036117f, -0.00042123f, -0.00047919f, -0.00053408f, -0.00058492f, -0.00063073f, -0.00067054f, -0.00070345f, -0.00072856f,
    -0.00074507f, -0.00075225f, -0.00074948f, -0.00073624f, -0.00071213f, -0.00067689f, -0.00063041f, -0.00057275f, -0.00050410f, -0.00042486f,
    -0.00033556f, -0.00023693f, -0.00012986f, -0.00001541f, 0.00010521f, 0.00023065f, 0.00035940f, 0.00048986f, 0.00062030f, 0.00074893f,
    0.00087389f, 0.00099328f, 0.00110520f, 0.00120776f, 0.00129911f, 0.00137746f, 0.00144115f, 0.00148862f, 0.00151848f, 0.00152951f,
    0.00152070f, 0.00149128f, 0.00144071f, 0.00136874f, 0.00127538f, 0.00116095f, 0.00102608f, 0.00087168f, 0.00069899f, 0.00050954f,
    0.00030517f, 0.00008797f, -0.00013967f, -0.00037515f, -0.00061564f, -0.00085814f, -0.00109948f, -0.00133640f, -0.00156555f, -0.00178356f,
    -0.00198708f, -0.00217281f, -0.00233757f, -0.00247834f, -0.00259230f, -0.00267687f, -0.00272980f, -0.00274914f, -0.00273333f, -0.00268124f,
    -0.00259216f, -0.00246584f, -0.00230256f, -0.00210307f, -0.00186864f, -0.00160106f, -0.00130261f, -0.00097608f, -0.00062475f, -0.00025231f,
    0.00013710f, 0.00053898f, 0.00094851f, 0.00136058f, 0.00176987f, 0.00217094f, 0.00255821f, 0.00292613f, 0.00326921f, 0.00358207f,
    0.00385958f, 0.00409687f, 0.00428946f, 0.00443327f, 0.00452477f, 0.00456097f, 0.00453952f, 0.00445874f, 0.00431768f, 0.00411615f,
    0.00385475f, 0.00353489f, 0.00315879f, 0.00272948f, 0.00225079f, 0.00172731f, 0.00116439f, 0.00056804f, -0.00005507f, -0.00069772f,
    -0.00135221f, -0.00201043f, -0.00266396f, -0.00330420f, -0.00392241f, -0.00450990f, -0.00505810f, -0.00555867f, -0.00600365f, -0.00638554f,
    -0.00669745f, -0.00693318f, -0.00708733f, -0.00715538f, -0.00713383f, -0.00702020f, -0.00681315f, -0.00651250f, -0.00611931f, -0.00563584f,
    -0.00506559f, -0.00441331f, -0.00368495f, -0.00288760f, -0.00202948f, -0.00111983f, -0.00016881f, 0.00081255f, 0.00181253f, 0.00281883f,
    0.00381869f, 0.00479909f, 0.00574686f, 0.00664889f, 0.00749226f, 0.00826447f, 0.00895353f, 0.00954824f, 0.01003823f, 0.01041425f,
    0.01066821f, 0.01079337f, 0.01078447f, 0.01063781f, 0.01035138f, 0.00992489f, 0.00935985f, 0.00865958f, 0.00782926f, 0.00687585f,
    0.00580809f, 0.00463646f, 0.00337303f, 0.00203138f, 0.00062649f, -0.00082545f, -0.00230721f, -0.00380069f, -0.00528715f, -0.00674743f,
    -0.00816217f, -0.00951208f, -0.01077816f, -0.01194198f, -0.01298589f, -0.01389331f, -0.01464896f, -0.01523907f, -0.01565160f, -0.01587648f,
    -0.01590574f, -0.01573368f, -0.01535704f, -0.01477506f, -0.01398957f, -0.01300505f, -0.01182863f, -0.01047007f, -0.00894172f, -0.00725840f,
    -0.00543732f, -0.00349788f, -0.00146152f, 0.00064853f, 0.00280748f, 0.00498923f, 0.00716673f, 0.00931224f, 0.01139769f, 0.01339500f,
    0.01527647f, 0.01701514f, 0.01858509f, 0.01996189f, 0.02112285f, 0.02204744f, 0.02271754f, 0.02311775f, 0.02323569f, 0.02306220f,
    0.02259153f, 0.02182153f, 0.02075379f, 0.01939366f, 0.01775034f, 0.01583683f, 0.01366988f, 0.01126989f, 0.00866075f, 0.00586962f,
    0.00292669f, -0.00013512f, -0.00328049f, -0.00647209f, -0.00967096f, -0.01283700f, -0.01592941f, -0.01890715f, -0.02172949f, -0.02435652f,
    -0.02674963f, -0.02887205f, -0.03068936f, -0.03216995f, -0.03328555f, -0.03401158f, -0.03432764f, -0.03421783f, -0.03367109f, -0.03268145f,
    -0.03124827f, -0.02937639f, -0.02707622f, -0.02436378f, -0.02126065f, -0.01779388f, -0.01399582f, -0.00990387f, -0.00556016f, -0.00101120f,
    0.00369256f, 0.00849721f, 0.01334598f, 0.01817979f, 0.02293788f, 0.02755853f, 0.03197977f, 0.03614006f, 0.03997908f, 0.04343847f,
    0.04646260f, 0.04899927f, 0.05100048f, 0.05242308f, 0.05322946f, 0.05338817f, 0.05287443f, 0.05167070f, 0.04976706f, 0.04716159f,
    0.04386067f, 0.03987915f, 0.03524046f, 0.02997665f, 0.02412829f, 0.01774434f, 0.01088181f, 0.00360548f, -0.00401259f, -0.01189360f,
    -0.01995265f, -0.02809939f, -0.03623888f, -0.04427239f, -0.05209830f, -0.05961309f, -0.06671236f, -0.07329178f, -0.07924826f, -0.08448092f,
    -0.08889222f, -0.09238899f, -0.09488349f, -0.09629439f, -0.09654773f, -0.09557785f, -0.09332819f, -0.08975211f, -0.08481352f, -0.07848750f,
    -0.07076078f, -0.06163212f, -0.05111262f, -0.03922583f, -0.02600783f, -0.01150713f, 0.00421551f, 0.02108744f, 0.03902451f, 0.05793165f,
    0.07770356f, 0.09822543f, 0.11937385f, 0.14101777f, 0.16301956f, 0.18523608f, 0.20751995f, 0.22972070f, 0.25168610f, 0.27326347f,
    0.29430098f, 0.31464897f, 0.33416129f, 0.35269658f, 0.37011953f, 0.38630207f, 0.40112455f, 0.41447680f, 0.42625912f, 0.43638319f,
    0.44477288f, 0.45136492f, 0.45610949f, 0.45897070f, 0.45992685f, 0.45897070f, 0.45610949f, 0.45136492f, 0.44477288f, 0.43638319f,
    0.42625912f, 0.41447680f, 0.40112455f, 0.38630207f, 0.37011953f, 0.35269658f, 0.33416129f, 0.31464897f, 0.29430098f, 0.27326347f,
    0.25168610f, 0.22972070f, 0.20751995f, 0.18523608f, 0.16301956f, 0.14101777f, 0.11937385f, 0.09822543f, 0.07770356f, 0.05793165f,
    0.03902451f, 0.02108744f, 0.00421551f, -0.01150713f, -0.02600783f, -0.03922583f, -0.05111262f, -0.06163212f, -0.07076078f, -0.07848750f,
    -0.08481352f, -0.08975211f, -0.09332819f, -0.09557785f, -0.09654773f, -0.09629439f, -0.09488349f, -0.09238899f, -0.08889222f, -0.08448092f,
    -0.07924826f, -0.07329178f, -0.06671236f, -0.05961309f, -0.05209830f, -0.04427239f, -0.03623888f, -0.02809939f, -0.01995265f, -0.01189360f,
    -0.00401259f, 0.00360548f, 0.01088181f, 0.01774434f, 0.02412829f, 0.02997665f, 0.03524046f, 0.03987915f, 0.04386067f, 0.04716159f,
    0.04976706f, 0.05167070f, 0.05287443f, 0.05338817f, 0.05322946f, 0.05242308f, 0.05100048f, 0.04899927f, 0.04646260f, 0.04343847f,
    0.03997908f, 0.03614006f, 0.03197977f, 0.02755853f, 0.02293788f, 0.01817979f, 0.01334598f, 0.00849721f, 0.00369256f, -0.00101120f,
    -0.00556016f, -0.00990387f, -0.01399582f, -0.01779388f, -0.02126065f, -0.02436378f, -0.02707622f, -0.02937639f, -0.03124827f, -0.03268145f,
    -0.03367109f, -0.03421783f, -0.03432764f, -0.03401158f, -0.03328555f, -0.03216995f, -0.03068936f, -0.02887205f, -0.02674963f, -0.02435652f,
    -0.02172949f, -0.01890715f, -0.01592941f, -0.01283700f, -0.00967096f, -0.00647209f, -0.00328049f, -0.00013512f, 0.00292669f, 0.00586962f,
    0.00866075f, 0.01126989f, 0.01366988f, 0.01583683f, 0.01775034f, 0.01939366f, 0.02075379f, 0.02182153f, 0.02259153f, 0.02306220f,
    0.02323569f, 0.02311775f, 0.02271754f, 0.02204744f, 0.02112285f, 0.01996189f, 0.01858509f, 0.01701514f, 0.01527647f, 0.01339500f,
    0.01139769f, 0.00931224f, 0.00716673f, 0.00498923f, 0.00280748f, 0.00064853f, -0.00146152f, -0.00349788f, -0.00543732f, -0.00725840f,
    -0.00894172f, -0.01047007f, -0.01182863f, -0.01300505f, -0.01398957f, -0.01477506f, -0.01535704f, -0.01573368f, -0.01590574f, -0.01587648f,
    -0.01565160f, -0.01523907f, -0.01464896f, -0.01389331f, -0.01298589f, -0.01194198f, -0.01077816f, -0.00951208f, -0.00816217f, -0.00674743f,
    -0.00528715f, -0.00380069f, -0.00230721f, -0.00082545f, 0.00062649f, 0.00203138f, 0.00337303f, 0.00463646f, 0.00580809f, 0.00687585f,
    0.00782926f, 0.00865958f, 0.00935985f, 0.00992489f, 0.01035138f, 0.01063781f, 0.01078447f, 0.01079337f, 0.01066821f, 0.01041425f,
    0.01003823f, 0.00954824f, 0.00895353f, 0.00826447f, 0.00749226f, 0.00664889f, 0.00574686f, 0.00479909f, 0.00381869f, 0.00281883f,
    0.00181253f, 0.00081255f, -0.00016881f, -0.00111983f, -0.00202948f, -0.00288760f, -0.00368495f, -0.00441331f, -0.00506559f, -0.00563584f,
    -0.00611931f, -0.00651250f, -0.00681315f, -0.00702020f, -0.00713383f, -0.00715538f, -0.00708733f, -0.00693318f, -0.00669745f, -0.00638554f,
    -0.00600365f, -0.00555867f, -0.00505810f, -0.00450990f, -0.00392241f, -0.00330420f, -0.00266396f, -0.00201043f, -0.00135221f, -0.00069772f,
    -0.00005507f, 0.00056804f, 0.00116439f, 0.00172731f, 0.00225079f, 0.00272948f, 0.00315879f, 0.00353489f, 0.00385475f, 0.00411615f,
    0.00431768f, 0.00445874f, 0.00453952f, 0.00456097f, 0.00452477f, 0.00443327f, 0.00428946f, 0.00409687f, 0.00385958f, 0.00358207f,
    0.00326921f, 0.00292613f, 0.00255821f, 0.00217094f, 0.00176987f, 0.00136058f, 0.00094851f, 0.00053898f, 0.00013710f, -0.00025231f,
    -0.00062475f, -0.00097608f, -0.00130261f, -0.00160106f, -0.00186864f, -0.00210307f, -0.00230256f, -0.00246584f, -0.00259216f, -0.00268124f,
    -0.00273333f, -0.00274914f, -0.00272980f, -0.00267687f, -0.00259230f, -0.00247834f, -0.00233757f, -0.00217281f, -0.00198708f, -0.00178356f,
    -0.00156555f, -0.00133640f, -0.00109948f, -0.00085814f, -0.00061564f, -0.00037515f, -0.00013967f, 0.00008797f, 0.00030517f, 0.00050954f,
    0.00069899f, 0.00087168f, 0.00102608f, 0.00116095f, 0.00127538f, 0.00136874f, 0.00144071f, 0.00149128f, 0.00152070f, 0.00152951f,
    0.00151848f, 0.00148862f, 0.00144115f, 0.00137746f, 0.00129911f, 0.00120776f, 0.00110520f, 0.00099328f, 0.00087389f, 0.00074893f,
    0.00062030f, 0.00048986f, 0.00035940f, 0.00023065f, 0.00010521f, -0.00001541f, -0.00012986f, -0.00023693f, -0.00033556f, -0.00042486f,
    -0.00050410f, -0.00057275f, -0.00063041f, -0.00067689f, -0.00071213f, -0.00073624f, -0.00074948f, -0.00075225f, -0.00074507f, -0.00072856f,
    -0.00070345f, -0.00067054f, -0.00063073f, -0.00058492f, -0.00053408f, -0.00047919f, -0.00042123f, -0.00036117f, -0.00029997f, -0.00023854f,
    -0.00017773f, -0.00011837f, -0.00006118f, -0.00000685f, 0.00004406f, 0.00009102f, 0.00013363f, 0.00017156f, 0.00020457f, 0.00023250f,
    0.00025530f, 0.00027297f, 0.00028560f, 0.00029335f, 0.00029645f, 0.00029518f, 0.00028987f, 0.00028088f, 0.00026862f, 0.00025352f,
    0.00023602f, 0.00021657f, 0.00019563f, 0.00017364f, 0.00015103f, 0.00012822f, 0.00010558f, 0.00008348f, 0.00006223f
};

/*
 * ---------------------------------------------------------------------------
 * Constructor / Destructor
 * ---------------------------------------------------------------------------
 */

dsp_resampler::dsp_resampler()
{
	reset();

	/* Pre-calculate reversed S1 coefficients for optimized SIMD convolution */
	for (int i = 0; i < S1_TAPS; i++) {
		s1_coeffs_rev[i] = S1_COEFFS[S1_TAPS - 1 - i];
	}

	/*
	 * Pre-calculate polyphase filter banks with reversed coefficients.
	 * The prototype filter is decomposed into S2_PHASES branches,
	 * each containing S2_TAPS_PER_PHASE coefficients.
	 */
	for (int phase = 0; phase < S2_PHASES; phase++) {
		for (int tap = 0; tap < S2_TAPS_PER_PHASE; tap++) {
			int raw_idx = phase + tap * S2_PHASES;
			/* Store in reverse order for contiguous dot product */
			if (raw_idx < S2_TAPS_TOTAL) {
				s2_coeffs_poly[phase][S2_TAPS_PER_PHASE - 1 - tap] = S2_COEFFS_RAW[raw_idx];
			} else {
				s2_coeffs_poly[phase][S2_TAPS_PER_PHASE - 1 - tap] = 0.0f;
			}
		}
	}
}

dsp_resampler::~dsp_resampler()
{
	/* Nothing to free - all storage is inline */
}

/*
 * ---------------------------------------------------------------------------
 * State Management
 * ---------------------------------------------------------------------------
 */

void dsp_resampler::reset()
{
	s1_index = 0;
	s1_head = 0;
	std::fill(std::begin(s1_history), std::end(s1_history), std::complex<float>(0, 0));

	s2_head = 0;
	s2_phase_state = 0;
	std::fill(std::begin(s2_history), std::end(s2_history), std::complex<float>(0, 0));
}

/*
 * ---------------------------------------------------------------------------
 * Main Processing Entry Point
 * ---------------------------------------------------------------------------
 */

size_t dsp_resampler::process(const std::complex<float>* in, size_t in_count,
			      std::complex<float>* out_buffer, size_t out_cap)
{
	size_t out_produced = 0;

	for (size_t i = 0; i < in_count; i++) {
		push_stage1(in[i], out_buffer, out_cap, out_produced);

		/*
		 * WARNING: If output buffer fills, remaining input is lost!
		 * Caller must ensure out_cap >= in_count / 9 to avoid this.
		 */
		if (out_produced >= out_cap)
			break;
	}

	return out_produced;
}

/*
 * ---------------------------------------------------------------------------
 * Stage 1: Integer Decimator (÷5)
 * ---------------------------------------------------------------------------
 */

void dsp_resampler::push_stage1(std::complex<float> sample,
				std::complex<float>* out_buffer,
				size_t out_cap, size_t& out_produced)
{
	/*
	 * Double-buffering technique: Write sample at both [head] and
	 * [head + S1_TAPS] so convolution can access S1_TAPS contiguous
	 * samples starting at [head] without modulo arithmetic.
	 */
	s1_history[s1_head] = sample;
	s1_history[s1_head + S1_TAPS] = sample;

	s1_head++;
	if (s1_head >= S1_TAPS)
		s1_head = 0;

	/* Increment decimation counter */
	s1_index++;

	/* Every S1_DECIMATION input samples, produce one output sample */
	if (s1_index >= S1_DECIMATION) {
		s1_index = 0;

		float acc_r = 0.0f;
		float acc_i = 0.0f;

		/*
		 * Vectorizable convolution: history at s1_head contains the
		 * last S1_TAPS samples in oldest-to-newest order. Coefficients
		 * are pre-reversed for straight dot product.
		 */
		const std::complex<float>* h_ptr = &s1_history[s1_head];
		const float* c_ptr = s1_coeffs_rev;

		for (int k = 0; k < S1_TAPS; k++) {
			acc_r += h_ptr[k].real() * c_ptr[k];
			acc_i += h_ptr[k].imag() * c_ptr[k];
		}

		/* Pass filtered sample to Stage 2 */
		push_stage2(std::complex<float>(acc_r, acc_i),
			    out_buffer, out_cap, out_produced);
	}
}

/*
 * ---------------------------------------------------------------------------
 * Stage 2: Polyphase Rational Resampler (×13/24)
 * ---------------------------------------------------------------------------
 */

void dsp_resampler::push_stage2(std::complex<float> sample,
				std::complex<float>* out_buffer,
				size_t out_cap, size_t& out_produced)
{
	/* Double-buffering (same technique as Stage 1) */
	s2_history[s2_head] = sample;
	s2_history[s2_head + S2_TAPS_PER_PHASE] = sample;

	s2_head++;
	if (s2_head >= S2_TAPS_PER_PHASE)
		s2_head = 0;

	/*
	 * Polyphase output generation: For each input sample, we may
	 * produce 0 or 1 output samples depending on phase state.
	 * The ratio 13/24 means we interpolate by 13 then decimate by 24.
	 */
	while (s2_phase_state < S2_INTERP) {
		/* Check output buffer capacity */
		if (out_produced >= out_cap)
			return;

		float acc_r = 0.0f;
		float acc_i = 0.0f;

		/*
		 * Select polyphase branch based on current phase state.
		 * Each branch has pre-reversed coefficients for vectorization.
		 */
		const std::complex<float>* h_ptr = &s2_history[s2_head];
		const float* branch_coeffs = s2_coeffs_poly[s2_phase_state];

		/* Vectorizable dot product */
		for (int k = 0; k < S2_TAPS_PER_PHASE; k++) {
			acc_r += h_ptr[k].real() * branch_coeffs[k];
			acc_i += h_ptr[k].imag() * branch_coeffs[k];
		}

		out_buffer[out_produced++] = std::complex<float>(acc_r, acc_i);

		/* Advance phase by decimation factor */
		s2_phase_state += S2_DECIM;
	}

	/* Wrap phase state (subtract interpolation factor) */
	s2_phase_state -= S2_INTERP;
}
