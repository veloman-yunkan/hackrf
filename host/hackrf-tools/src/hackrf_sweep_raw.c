/*
 * Copyright 2016 Dominic Spill <dominicgs@gmail.com>
 * Copyright 2016 Mike Walters <mike@flomp.net>
 * Copyright 2017 Michael Ossmann <mike@ossmann.com>
 *
 * This file is part of HackRF.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#include <hackrf.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <time.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <inttypes.h>

#define _FILE_OFFSET_BITS 64

#ifndef bool
typedef int bool;
#define true 1
#define false 0
#endif

#ifdef _WIN32
#define _USE_MATH_DEFINES
#include <windows.h>
#ifdef _MSC_VER

#ifdef _WIN64
typedef int64_t ssize_t;
#else
typedef int32_t ssize_t;
#endif

#define strtoull _strtoui64
#define snprintf _snprintf

int gettimeofday(struct timeval *tv, void* ignored) {
	FILETIME ft;
	unsigned __int64 tmp = 0;
	if (NULL != tv) {
		GetSystemTimeAsFileTime(&ft);
		tmp |= ft.dwHighDateTime;
		tmp <<= 32;
		tmp |= ft.dwLowDateTime;
		tmp /= 10;
		tmp -= 11644473600000000Ui64;
		tv->tv_sec = (long)(tmp / 1000000UL);
		tv->tv_usec = (long)(tmp % 1000000UL);
	}
	return 0;
}

#endif
#endif

#if defined(__GNUC__)
#include <unistd.h>
#include <sys/time.h>
#endif

#include <signal.h>
#include <math.h>

#define FD_BUFFER_SIZE (8*1024)

#define FREQ_ONE_MHZ (1000000ull)

#define FREQ_MIN_MHZ (0)    /*    0 MHz */
#define FREQ_MAX_MHZ (7250) /* 7250 MHz */

#define DEFAULT_SAMPLE_RATE_HZ (20000000) /* 20MHz default sample rate */
#define DEFAULT_BASEBAND_FILTER_BANDWIDTH (15000000) /* 15MHz default */

#define TUNE_STEP (DEFAULT_SAMPLE_RATE_HZ / FREQ_ONE_MHZ)
#define OFFSET 7500000

#define BLOCKS_PER_TRANSFER 16
#define THROWAWAY_BLOCKS 2

#if defined _WIN32
	#define m_sleep(a) Sleep( (a) )
#else
	#define m_sleep(a) usleep((a*1000))
#endif

uint32_t num_samples = SAMPLES_PER_BLOCK;
uint32_t num_sweeps = 0;
int num_ranges = 0;
uint16_t frequencies[MAX_SWEEP_RANGES*2];
int step_count;

static float TimevalDiff(const struct timeval *a, const struct timeval *b) {
   return (a->tv_sec - b->tv_sec) + 1e-6f * (a->tv_usec - b->tv_usec);
}

int parse_u32(char* s, uint32_t* const value) {
	uint_fast8_t base = 10;
	char* s_end;
	uint64_t ulong_value;

	if( strlen(s) > 2 ) {
		if( s[0] == '0' ) {
			if( (s[1] == 'x') || (s[1] == 'X') ) {
				base = 16;
				s += 2;
			} else if( (s[1] == 'b') || (s[1] == 'B') ) {
				base = 2;
				s += 2;
			}
		}
	}

	s_end = s;
	ulong_value = strtoul(s, &s_end, base);
	if( (s != s_end) && (*s_end == 0) ) {
		*value = (uint32_t)ulong_value;
		return HACKRF_SUCCESS;
	} else {
		return HACKRF_ERROR_INVALID_PARAM;
	}
}

int parse_u32_range(char* s, uint32_t* const value_min, uint32_t* const value_max) {
	int result;

	char *sep = strchr(s, ':');
	if (!sep)
		return HACKRF_ERROR_INVALID_PARAM;

	*sep = 0;

	result = parse_u32(s, value_min);
	if (result != HACKRF_SUCCESS)
		return result;
	result = parse_u32(sep + 1, value_max);
	if (result != HACKRF_SUCCESS)
		return result;

	return HACKRF_SUCCESS;
}

volatile bool do_exit = false;

FILE* outfile = NULL;
volatile uint32_t byte_count = 0;
volatile uint64_t sweep_count = 0;

struct timeval time_start;
struct timeval t_start;
struct timeval time_stamp;

bool amp = false;
uint32_t amp_enable;

bool antenna = false;
uint32_t antenna_enable;

bool one_shot = false;
bool finite_mode = false;
volatile bool sweep_started = false;

int output_samples(int8_t* samples, uint32_t n)
{
	for ( ; n; --n, samples += 2 ) {
		printf("%hhd %hhd\n", *samples, *(samples+1));
	}
	return 0;
}

int rx_callback(hackrf_transfer* transfer) {
	int8_t* buf;
	uint8_t* ubuf;
	uint64_t frequency = UINT64_MAX; /* in Hz */
	int i, j;
	char time_str[50];
	struct timeval usb_transfer_time;

	if(NULL == outfile) {
		return -1;
	}

	gettimeofday(&usb_transfer_time, NULL);
	byte_count += transfer->valid_length;
	buf = (int8_t*) transfer->buffer;
	for(j=0; j<BLOCKS_PER_TRANSFER; j++, buf += BYTES_PER_BLOCK) {
		ubuf = (uint8_t*) buf;
		if(ubuf[0] == 0x7F && ubuf[1] == 0x7F) {
			frequency = ((uint64_t)(ubuf[9]) << 56) | ((uint64_t)(ubuf[8]) << 48) | ((uint64_t)(ubuf[7]) << 40)
					| ((uint64_t)(ubuf[6]) << 32) | ((uint64_t)(ubuf[5]) << 24) | ((uint64_t)(ubuf[4]) << 16)
					| ((uint64_t)(ubuf[3]) << 8) | ubuf[2];
			printf("freq=%" PRIu64 "\n", frequency);
			output_samples(buf+10, (BYTES_PER_BLOCK-10)/2);
		} else {
			if ( frequency != UINT64_MAX ) {
				output_samples(buf, BYTES_PER_BLOCK/2);
			}
		}
		if (frequency == (uint64_t)(FREQ_ONE_MHZ*frequencies[0]) + OFFSET) {
			if(sweep_started) {
				sweep_count++;
				if(one_shot) {
					do_exit = true;
				}
				else if(finite_mode && sweep_count == num_sweeps) {
					do_exit = true;
				}
			}
			sweep_started = true;
			time_stamp = usb_transfer_time;
			time_stamp.tv_usec +=
					(uint64_t)(num_samples + THROWAWAY_BLOCKS * SAMPLES_PER_BLOCK)
					* j * FREQ_ONE_MHZ / DEFAULT_SAMPLE_RATE_HZ;
			if(999999 < time_stamp.tv_usec) {
				time_stamp.tv_sec += time_stamp.tv_usec / 1000000;
				time_stamp.tv_usec = time_stamp.tv_usec % 1000000;
			}
		}
		if(do_exit) {
			return 0;
		}
	}
	return 0;
}

static void usage() {
	fprintf(stderr, "Usage:\n");
	fprintf(stderr, "\t[-h] # this help\n");
	fprintf(stderr, "\t[-d serial_number] # Serial number of desired HackRF\n");
	fprintf(stderr, "\t[-a amp_enable] # RX RF amplifier 1=Enable, 0=Disable\n");
	fprintf(stderr, "\t[-f freq_min:freq_max] # minimum and maximum frequencies in MHz\n");
	fprintf(stderr, "\t[-p antenna_enable] # Antenna port power, 1=Enable, 0=Disable\n");
	fprintf(stderr, "\t[-l gain_db] # RX LNA (IF) gain, 0-40dB, 8dB steps\n");
	fprintf(stderr, "\t[-g gain_db] # RX VGA (baseband) gain, 0-62dB, 2dB steps\n");
	fprintf(stderr, "\t[-n num_samples] # Number of samples per frequency, 8192-4294967296\n");
	fprintf(stderr, "\t[-1] # one shot mode\n");
	fprintf(stderr, "\t[-N num_sweeps] # Number of sweeps to perform\n");
	fprintf(stderr, "\t-r filename # output file\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "Output fields:\n");
	fprintf(stderr, "\tdate, time, hz_low, hz_high, hz_bin_width, num_samples, dB, dB, . . .\n");
}

static hackrf_device* device = NULL;

#ifdef _MSC_VER
BOOL WINAPI
sighandler(int signum) {
	if (CTRL_C_EVENT == signum) {
		fprintf(stderr, "Caught signal %d\n", signum);
		do_exit = true;
		return TRUE;
	}
	return FALSE;
}
#else
void sigint_callback_handler(int signum)  {
	fprintf(stderr, "Caught signal %d\n", signum);
	do_exit = true;
}
#endif

int main(int argc, char** argv) {
	int opt, i, result = 0;
	const char* path = NULL;
	const char* serial_number = NULL;
	int exit_code = EXIT_SUCCESS;
	struct timeval time_now;
	struct timeval time_prev;
	float time_diff;
	float sweep_rate;
	unsigned int lna_gain=16, vga_gain=20;
	uint32_t freq_min = 0;
	uint32_t freq_max = 6000;


	while( (opt = getopt(argc, argv, "a:f:p:l:g:d:n:N:1BIr:h?")) != EOF ) {
		result = HACKRF_SUCCESS;
		switch( opt )
		{
		case 'd':
			serial_number = optarg;
			break;

		case 'a':
			amp = true;
			result = parse_u32(optarg, &amp_enable);
			break;

		case 'f':
			result = parse_u32_range(optarg, &freq_min, &freq_max);
			if(freq_min >= freq_max) {
				fprintf(stderr,
						"argument error: freq_max must be greater than freq_min.\n");
				usage();
				return EXIT_FAILURE;
			}
			if(FREQ_MAX_MHZ <freq_max) {
				fprintf(stderr,
						"argument error: freq_max may not be higher than %u.\n",
						FREQ_MAX_MHZ);
				usage();
				return EXIT_FAILURE;
			}
			if(MAX_SWEEP_RANGES <= num_ranges) {
				fprintf(stderr,
						"argument error: specify a maximum of %u frequency ranges.\n",
						MAX_SWEEP_RANGES);
				usage();
				return EXIT_FAILURE;
			}
			frequencies[2*num_ranges] = (uint16_t)freq_min;
			frequencies[2*num_ranges+1] = (uint16_t)freq_max;
			num_ranges++;
			break;

		case 'p':
			antenna = true;
			result = parse_u32(optarg, &antenna_enable);
			break;

		case 'l':
			result = parse_u32(optarg, &lna_gain);
			break;

		case 'g':
			result = parse_u32(optarg, &vga_gain);
			break;

		case 'n':
			result = parse_u32(optarg, &num_samples);
			break;

		case 'N':
			finite_mode = true;
			result = parse_u32(optarg, &num_sweeps);
			break;

		case '1':
			one_shot = true;
			break;

		case 'r':
			path = optarg;
			break;

		case 'h':
		case '?':
			usage();
			return EXIT_SUCCESS;

		default:
			fprintf(stderr, "unknown argument '-%c %s'\n", opt, optarg);
			usage();
			return EXIT_FAILURE;
		}

		if( result != HACKRF_SUCCESS ) {
			fprintf(stderr, "argument error: '-%c %s' %s (%d)\n", opt, optarg, hackrf_error_name(result), result);
			usage();
			return EXIT_FAILURE;
		}
	}

	if (lna_gain % 8)
		fprintf(stderr, "warning: lna_gain (-l) must be a multiple of 8\n");

	if (vga_gain % 2)
		fprintf(stderr, "warning: vga_gain (-g) must be a multiple of 2\n");

	if (num_samples % SAMPLES_PER_BLOCK) {
		fprintf(stderr, "warning: num_samples (-n) must be a multiple of 8192\n");
		return EXIT_FAILURE;
	}

	if (num_samples < SAMPLES_PER_BLOCK) {
		fprintf(stderr, "warning: num_samples (-n) must be at least 8192\n");
		return EXIT_FAILURE;
	}

	if( amp ) {
		if( amp_enable > 1 ) {
			fprintf(stderr, "argument error: amp_enable shall be 0 or 1.\n");
			usage();
			return EXIT_FAILURE;
		}
	}

	if (antenna) {
		if (antenna_enable > 1) {
			fprintf(stderr, "argument error: antenna_enable shall be 0 or 1.\n");
			usage();
			return EXIT_FAILURE;
		}
	}

	if (0 == num_ranges) {
		frequencies[0] = (uint16_t)freq_min;
		frequencies[1] = (uint16_t)freq_max;
		num_ranges++;
	}

	result = hackrf_init();
	if( result != HACKRF_SUCCESS ) {
		fprintf(stderr, "hackrf_init() failed: %s (%d)\n", hackrf_error_name(result), result);
		usage();
		return EXIT_FAILURE;
	}

	result = hackrf_open_by_serial(serial_number, &device);
	if( result != HACKRF_SUCCESS ) {
		fprintf(stderr, "hackrf_open() failed: %s (%d)\n", hackrf_error_name(result), result);
		usage();
		return EXIT_FAILURE;
	}

	if((NULL == path) || (strcmp(path, "-") == 0)) {
		outfile = stdout;
	} else {
		outfile = fopen(path, "wb");
	}

	if(NULL == outfile) {
		fprintf(stderr, "Failed to open file: %s\n", path);
		return EXIT_FAILURE;
	}
	/* Change outfile buffer to have bigger one to store or read data on/to HDD */
	result = setvbuf(outfile , NULL , _IOFBF , FD_BUFFER_SIZE);
	if( result != 0 ) {
		fprintf(stderr, "setvbuf() failed: %d\n", result);
		usage();
		return EXIT_FAILURE;
	}

#ifdef _MSC_VER
	SetConsoleCtrlHandler( (PHANDLER_ROUTINE) sighandler, TRUE );
#else
	signal(SIGINT, &sigint_callback_handler);
	signal(SIGILL, &sigint_callback_handler);
	signal(SIGFPE, &sigint_callback_handler);
	signal(SIGSEGV, &sigint_callback_handler);
	signal(SIGTERM, &sigint_callback_handler);
	signal(SIGABRT, &sigint_callback_handler);
#ifndef _WIN32
	signal(SIGPIPE, &sigint_callback_handler);
#endif
#endif
	fprintf(stderr, "call hackrf_sample_rate_set(%.03f MHz)\n",
		   ((float)DEFAULT_SAMPLE_RATE_HZ/(float)FREQ_ONE_MHZ));
	result = hackrf_set_sample_rate_manual(device, DEFAULT_SAMPLE_RATE_HZ, 1);
	if( result != HACKRF_SUCCESS ) {
		fprintf(stderr, "hackrf_sample_rate_set() failed: %s (%d)\n",
			   hackrf_error_name(result), result);
		usage();
		return EXIT_FAILURE;
	}

	fprintf(stderr, "call hackrf_baseband_filter_bandwidth_set(%.03f MHz)\n",
			((float)DEFAULT_BASEBAND_FILTER_BANDWIDTH/(float)FREQ_ONE_MHZ));
	result = hackrf_set_baseband_filter_bandwidth(device, DEFAULT_BASEBAND_FILTER_BANDWIDTH);
	if( result != HACKRF_SUCCESS ) {
		fprintf(stderr, "hackrf_baseband_filter_bandwidth_set() failed: %s (%d)\n",
			   hackrf_error_name(result), result);
		usage();
		return EXIT_FAILURE;
	}

	result = hackrf_set_vga_gain(device, vga_gain);
	result |= hackrf_set_lna_gain(device, lna_gain);

	/*
	 * For each range, plan a whole number of tuning steps of a certain
	 * bandwidth. Increase high end of range if necessary to accommodate a
	 * whole number of steps, minimum 1.
	 */
	for(i = 0; i < num_ranges; i++) {
		step_count = 1 + (frequencies[2*i+1] - frequencies[2*i] - 1)
				/ TUNE_STEP;
		frequencies[2*i+1] = (uint16_t) (frequencies[2*i] + step_count * TUNE_STEP);
		fprintf(stderr, "Sweeping from %u MHz to %u MHz\n",
				frequencies[2*i], frequencies[2*i+1]);
	}

	result = hackrf_init_sweep(device, frequencies, num_ranges, num_samples * 2,
			TUNE_STEP * FREQ_ONE_MHZ, OFFSET, INTERLEAVED);
	if( result != HACKRF_SUCCESS ) {
		fprintf(stderr, "hackrf_init_sweep() failed: %s (%d)\n",
			   hackrf_error_name(result), result);
		return EXIT_FAILURE;
	}

	result |= hackrf_start_rx_sweep(device, rx_callback, NULL);
	if (result != HACKRF_SUCCESS) {
		fprintf(stderr, "hackrf_start_rx_sweep() failed: %s (%d)\n", hackrf_error_name(result), result);
		usage();
		return EXIT_FAILURE;
	}

	if (amp) {
		fprintf(stderr, "call hackrf_set_amp_enable(%u)\n", amp_enable);
		result = hackrf_set_amp_enable(device, (uint8_t)amp_enable);
		if (result != HACKRF_SUCCESS) {
			fprintf(stderr, "hackrf_set_amp_enable() failed: %s (%d)\n",
				   hackrf_error_name(result), result);
			usage();
			return EXIT_FAILURE;
		}
	}

	if (antenna) {
		fprintf(stderr, "call hackrf_set_antenna_enable(%u)\n", antenna_enable);
		result = hackrf_set_antenna_enable(device, (uint8_t)antenna_enable);
		if (result != HACKRF_SUCCESS) {
			fprintf(stderr, "hackrf_set_antenna_enable() failed: %s (%d)\n",
				   hackrf_error_name(result), result);
			usage();
			return EXIT_FAILURE;
		}
	}

	gettimeofday(&t_start, NULL);
	time_prev = t_start;

	fprintf(stderr, "Stop with Ctrl-C\n");
	while((hackrf_is_streaming(device) == HACKRF_TRUE) && (do_exit == false)) {
		float time_difference;
		m_sleep(50);

		gettimeofday(&time_now, NULL);
		if (TimevalDiff(&time_now, &time_prev) >= 1.0f) {
			time_difference = TimevalDiff(&time_now, &t_start);
			sweep_rate = (float)sweep_count / time_difference;
			fprintf(stderr, "%" PRIu64 " total sweeps completed, %.2f sweeps/second\n",
					sweep_count, sweep_rate);

			if (byte_count == 0) {
				exit_code = EXIT_FAILURE;
				fprintf(stderr, "\nCouldn't transfer any data for one second.\n");
				break;
			}
			byte_count = 0;
			time_prev = time_now;
		}
	}

	result = hackrf_is_streaming(device);
	if (do_exit) {
		fprintf(stderr, "\nExiting...\n");
	} else {
		fprintf(stderr, "\nExiting... hackrf_is_streaming() result: %s (%d)\n",
			   hackrf_error_name(result), result);
	}

	gettimeofday(&time_now, NULL);
	time_diff = TimevalDiff(&time_now, &t_start);
	fprintf(stderr, "Total sweeps: %" PRIu64 " in %.5f seconds (%.2f sweeps/second)\n",
			sweep_count, time_diff, sweep_rate);

	if(device != NULL) {
		result = hackrf_stop_rx(device);
		if(result != HACKRF_SUCCESS) {
			fprintf(stderr, "hackrf_stop_rx() failed: %s (%d)\n",
				   hackrf_error_name(result), result);
		} else {
			fprintf(stderr, "hackrf_stop_rx() done\n");
		}

		result = hackrf_close(device);
		if(result != HACKRF_SUCCESS) {
			fprintf(stderr, "hackrf_close() failed: %s (%d)\n",
				   hackrf_error_name(result), result);
		} else {
			fprintf(stderr, "hackrf_close() done\n");
		}

		hackrf_exit();
		fprintf(stderr, "hackrf_exit() done\n");
	}

	fflush(outfile);
	if ( ( outfile != NULL ) && ( outfile != stdout ) ) {
		fclose(outfile);
		outfile = NULL;
		fprintf(stderr, "fclose() done\n");
	}
	fprintf(stderr, "exit\n");
	return exit_code;
}
