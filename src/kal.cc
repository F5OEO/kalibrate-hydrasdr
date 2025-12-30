/**
 * @file kal.cc
 * @brief Main entry point for kalibrate-hydrasdr.
 */
/*
 * @author Joshua Lackey (original)
 * @author Benjamin Vernoux <bvernoux@hydrasdr.com> (improvements)
 * @copyright 2010 Joshua Lackey, 2025 Benjamin Vernoux
 * @license BSD-2-Clause
 */
#define PACKAGE_VERSION "0.5.0"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <time.h> 
#include <signal.h> // Added for signal handling

#ifdef _WIN32
#include "win_compat.h"
#define basename(x) "kal"
#define strtof strtod
#else
#include <unistd.h>
#include <sys/time.h>
#include <libgen.h>
#endif

#include "iio_source.h"
#include "fcch_detector.h"
#include "arfcn_freq.h"
#include "offset.h"
#include "c0_detect.h"
#include "util.h"

int g_verbosity = 0;
int g_debug = 0;
int g_show_fft = 0;

// Global control flags for signal handling
volatile sig_atomic_t g_kal_exit_req = 0;

// Signal handler: Strictly async-signal-safe
void sighandler(int signum) {
	if (g_kal_exit_req) {
		// Force exit on double Ctrl-C
		const char* msg = "\nForcing exit.\n";
		write(2, msg, strlen(msg)); 
		_exit(1);
	}
	const char* msg = "\nSignal received, stopping...\n";
	write(2, msg, strlen(msg));
	g_kal_exit_req = 1;
}

void usage(char *prog) {
	fprintf(stderr, "kalibrate v%s-iio (PlutoSDR)\n", PACKAGE_VERSION);
	fprintf(stderr, "\nUsage:\n");
	fprintf(stderr, "\tGSM Base Station Scan:\n");
	fprintf(stderr, "\t\t%s <-s band indicator> [options]\n", basename(prog));
	fprintf(stderr, "\n");
	fprintf(stderr, "\tClock Offset Calculation:\n");
	fprintf(stderr, "\t\t%s <-f frequency | -c channel> [options]\n", basename(prog));
	fprintf(stderr, "\n");
	fprintf(stderr, "Where options are:\n");
	fprintf(stderr, "\t-s\tband to scan (GSM850, GSM-R, GSM900, EGSM, DCS)\n");
	fprintf(stderr, "\t-f\tfrequency of nearby GSM base station\n");
	fprintf(stderr, "\t-c\tchannel of nearby GSM base station\n");
	fprintf(stderr, "\t-b\tband indicator (GSM850, GSM-R, GSM900, EGSM, DCS)\n");
	fprintf(stderr, "\t-g\tgain (dB)\n");
	fprintf(stderr, "\t-u\tIIO URI (e.g. ip:192.168.2.1 or usb:x.y.z)\n");
	fprintf(stderr, "\t-A\tShow ASCII FFT of signal\n");
	fprintf(stderr, "\t-B\tRun DSP Benchmark and exit\n");
	fprintf(stderr, "\t-v\tverbose\n");
	fprintf(stderr, "\t-D\tenable debug messages\n");
	fprintf(stderr, "\t-h\thelp\n");
	exit(1);
}


int main(int argc, char **argv) {
	int c;
	int bi = BI_NOT_DEFINED;
	int chan = -1;
	int bts_scan = 0;
	float gain = 40.0; 
	double freq = -1.0;
	int result = 0;
	char *uri = NULL;
	
	iio_source *u = NULL;

	// Setup Windows Console for Unicode/ANSI
#ifdef _WIN32
	SetConsoleOutputCP(65001); // CP_UTF8
	HANDLE hOut = GetStdHandle(STD_OUTPUT_HANDLE);
	DWORD dwMode = 0;
	GetConsoleMode(hOut, &dwMode);
	dwMode |= 0x0004; // ENABLE_VIRTUAL_TERMINAL_PROCESSING
	SetConsoleMode(hOut, dwMode);
#endif

	// Register Signal Handler
#ifdef SIGINT
	signal(SIGINT, sighandler);
#endif

	while((c = getopt(argc, argv, "f:c:s:b:g:u:vDBAh?")) != EOF) {
		switch(c) {
			case 'f':
				freq = strtod(optarg, 0);
				break;
			case 'c':
				chan = strtoul(optarg, 0, 0);
				break;
			case 's':
				if((bi = str_to_bi(optarg)) == -1) {
					fprintf(stderr, "error: bad band indicator: ``%s''\n", optarg);
					usage(argv[0]);
				}
				bts_scan = 1;
				break;
			case 'b':
				if((bi = str_to_bi(optarg)) == -1) {
					fprintf(stderr, "error: bad band indicator: ``%s''\n", optarg);
					usage(argv[0]);
				}
				break;
			case 'g':
				gain = strtof(optarg, 0);
				break;
			case 'u':
				uri = optarg;
				break;
			case 'B':
				run_dsp_benchmark();
				return 0; 
			case 'A':
				g_show_fft = 1;
				break;
			case 'v':
				g_verbosity++;
				break;
			case 'D':
				g_debug = 1;
				break;
			case 'h':
			case '?':
			default:
				usage(argv[0]);
				break;
		}
	}

	if(bts_scan) {
		if(bi == BI_NOT_DEFINED) {
			fprintf(stderr, "error: scanning requires band (-s)\n");
			usage(argv[0]);
		}
	} else {
		if(freq < 0.0) {
			if(chan < 0) {
				fprintf(stderr, "error: must enter scan band -s or channel -c or frequency -f\n");
				usage(argv[0]);
			}
			freq = arfcn_to_freq(chan, &bi);
		}
		
		if (chan == -1 && freq != -1.0) {
			chan = freq_to_arfcn(freq, &bi);
		}
	}

	if(g_debug) {
		printf("debug: Gain                 : %f\n", gain);
	}

	u = new iio_source(gain, uri);
	if(!u) {
		fprintf(stderr, "error: failed to allocate iio_source\n");
		return -1;
	}

	if(u->open() == -1) {
		fprintf(stderr, "error: failed to open IIO device\n");
		delete u;
		return -1;
	}

	if(!bts_scan) {
		if(u->tune(freq) == -1) {
			fprintf(stderr, "error: iio_source::tune failed\n");
			result = -1;
			goto cleanup;
		}

		double tuner_error = 0.0; 

		fprintf(stderr, "%s: Calculating clock frequency offset.\n", basename(argv[0]));
		fprintf(stderr, "Using %s channel %d (%.1fMHz)\n", bi_to_str(bi), chan, freq / 1e6);
		
		result = offset_detect(u, 0, tuner_error);
		goto cleanup;
	}

	fprintf(stderr, "%s: Scanning for %s base stations.\n", basename(argv[0]), bi_to_str(bi));

	result = c0_detect(u, bi);

cleanup:
	if(u) {
		delete u;
	}
	return result;
}