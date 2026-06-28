/*
 * Phoenix-RTOS compat shim for FreeBSD's teken (force-included).
 * Defines the few <sys/cdefs.h> attribute macros teken uses that Phoenix's
 * headers don't provide. Each is a no-op-equivalent GCC attribute.
 */
#ifndef TEKEN_PHOENIX_H
#define TEKEN_PHOENIX_H
#ifndef __unused
#define __unused __attribute__((__unused__))
#endif
#ifndef __packed
#define __packed __attribute__((__packed__))
#endif
#ifndef __aligned
#define __aligned(x) __attribute__((__aligned__(x)))
#endif
#ifndef __printflike
#define __printflike(fmtarg, firstvararg) __attribute__((__format__(__printf__, fmtarg, firstvararg)))
#endif
#ifndef __FBSDID
#define __FBSDID(s)
#endif
#endif
