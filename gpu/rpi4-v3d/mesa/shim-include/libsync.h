/*
 * Minimal libsync.h shim - Android sync-fence surface referenced by Mesa's
 * EGL/dmabuf paths, which Phoenix never takes. Synchronous submit means a wait
 * always succeeds.
 *
 * Copyright 2026 Phoenix Systems  %LICENSE%
 */
#ifndef PHX_LIBSYNC_H
#define PHX_LIBSYNC_H
#include <stdint.h>
static inline int sync_wait(int fd,int t){(void)fd;(void)t;return 0;}
static inline int sync_merge(const char*n,int a,int b){(void)n;(void)a;(void)b;return -1;}
static inline int sync_accumulate(const char*n,int*a,int b){(void)n;(void)b;return (a&&*a>=0)?0:0;}
#endif
