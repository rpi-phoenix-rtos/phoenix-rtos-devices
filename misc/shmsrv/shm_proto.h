/*
 * Phoenix-RTOS
 *
 * shmsrv wire protocol: anonymous shared memory objects (the memfd_create()
 * backing for Wayland wl_shm pools on the new GPU lane)
 *
 * Copyright 2026 Phoenix Systems
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef SHMSRV_PROTO_H
#define SHMSRV_PROTO_H

#include <stdint.h>

/*
 * Namespace: the server registers SHMSRV_NS on its port (a top-level name, like
 * /kmsbuf: libphoenix resolves every intermediate path component with
 * mtGetAttr, so /dev/shm/<id> would need devfs to know "shm").
 *
 *   create   mtDevCtl on the root object {port, 0}: i.raw = shmsrv_create_req_t,
 *            o.raw = shmsrv_create_rsp_t -> a fresh id (never reused)
 *   open     open("/shm/<id>", O_RDWR): mtLookup + mtOpen (reply 0) -- the
 *            descriptor's oid is {port, id}; it crosses AF_UNIX (SCM_RIGHTS)
 *   size     ftruncate(fd, n) -> mtTruncate: the first one allocates n bytes
 *            (rounded up to a power of two, at least 1 MiB = the object's capacity) of
 *            zeroed MAP_CONTIGUOUS cached memory and memExport()s the first
 *            n bytes under {port, id}; a later one within the capacity re-exports
 *            the new size over the SAME pages (every mapping stays coherent),
 *            beyond it -> -EFBIG (the pages cannot move under existing mappings)
 *   map      mmap(fd, ...) with no memory-type flags: the kernel maps the export
 *            window (E1), the same physical pages in every process
 *   close    mtClose at the last close of the open file: the object is withdrawn
 *            (memUnexport + munmap); the pages live on in every existing mapping
 *            and are freed with the last one (E1 refcounting)
 *
 * atSize is refused (E1 section 3: a positive answer to the kernel's proc_size
 * after a tree miss would create a file-backed shadow object); fstat() gets the
 * size through mtGetAttrAll.
 */

#define SHMSRV_NS         "/shm"
#define SHMSRV_PROTO      1u
#define SHMSRV_OP_CREATE  0x53480001u /* 'S''H' 1 */
#define SHMSRV_OP_STATS   0x53480002u
#define SHMSRV_OP_QUIT    0x53480003u
#define SHMSRV_MAX_BYTES  (256u << 20) /* one object at most (contiguous memory) */

typedef struct {
	uint32_t op;
	uint32_t proto;
} shmsrv_create_req_t;

typedef struct {
	int32_t err;  /* 0 or -errno */
	uint32_t id;  /* the new object; open SHMSRV_NS "/<id>" */
	uint32_t live;
	uint32_t pad;
	uint64_t bytes; /* STATS: bytes currently allocated */
} shmsrv_create_rsp_t;

#endif
