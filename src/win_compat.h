/**
 * @file win_compat.h
 * @brief Windows compatibility layer for POSIX functions (gettimeofday, usleep, etc).
 */
/*
 * Copyright 2025 Benjamin Vernoux <bvernoux@hydrasdr.com>
 * SPDX-License-Identifier: BSD-2-Clause
 */
#ifndef WIN_COMPAT_H
#define WIN_COMPAT_H

#ifdef _WIN32

#include <windows.h>
#include <time.h>
#include <stdint.h>
#include <unistd.h>
#include <getopt.h>

#ifndef usleep
#define usleep(x) Sleep((x)/1000)
#endif

#ifndef sleep
#define sleep(x) Sleep((x)*1000)
#endif

// Always define timeval manually to avoid needing Winsock
#ifndef _TIMEVAL_DEFINED
#define _TIMEVAL_DEFINED
struct timeval {
    long tv_sec;
    long tv_usec;
};
struct timezone {
    int tz_minuteswest;
    int tz_dsttime;
};

static int gettimeofday(struct timeval * tp, struct timezone * tzp) {
    static const uint64_t EPOCH = ((uint64_t) 116444736000000000ULL);
    SYSTEMTIME  system_time;
    FILETIME    file_time;
    uint64_t    time;

    GetSystemTime( &system_time );
    SystemTimeToFileTime( &system_time, &file_time );
    time =  ((uint64_t)file_time.dwLowDateTime )      ;
    time += ((uint64_t)file_time.dwHighDateTime) << 32;

    tp->tv_sec  = (long) ((time - EPOCH) / 10000000L);
    tp->tv_usec = (long) (system_time.wMilliseconds * 1000);
    return 0;
}
#endif // _TIMEVAL_DEFINED

#endif // _WIN32
#endif // WIN_COMPAT_H