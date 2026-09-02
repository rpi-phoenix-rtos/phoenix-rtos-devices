/*
 * Minimal dlfcn.h shim - Mesa's dlopen()-based loader paths are unused on
 * Phoenix (everything is statically linked), so declare the surface and let the
 * stubs in v3d_phoenix_stubs.c report "not found".
 *
 * Copyright 2026 Phoenix Systems  %LICENSE%
 */
#ifndef PHX_DLFCN_H
#define PHX_DLFCN_H
#define RTLD_LAZY 1
#define RTLD_NOW 2
#define RTLD_LOCAL 0
#define RTLD_GLOBAL 0x100
#define RTLD_NOLOAD 0x4
#define RTLD_NODELETE 0x1000
#define RTLD_DEFAULT ((void*)0)
#define RTLD_NEXT ((void*)-1)
static inline void*dlopen(const char*p,int f){(void)p;(void)f;return 0;}
static inline void*dlsym(void*h,const char*s){(void)h;(void)s;return 0;}
static inline int dlclose(void*h){(void)h;return 0;}
static inline char*dlerror(void){return 0;}
typedef struct { const char*dli_fname; void*dli_fbase; const char*dli_sname; void*dli_saddr; } Dl_info;
static inline int dladdr(const void*addr, Dl_info*info){(void)addr; if(info){info->dli_fname=0;info->dli_fbase=0;info->dli_sname=0;info->dli_saddr=0;} return 0;}
#endif
