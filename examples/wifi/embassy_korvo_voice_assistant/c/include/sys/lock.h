// picolibc's `sys/lock.h` plus ESP-IDF's `_lock_t`, which dl_fft uses. The locks do nothing,
// because only the second core runs ESP-SR.
#pragma once

#include_next <sys/lock.h>

typedef int _lock_t;

static inline void _lock_init(_lock_t *lock) { (void)lock; }
static inline void _lock_acquire(_lock_t *lock) { (void)lock; }
static inline void _lock_release(_lock_t *lock) { (void)lock; }
