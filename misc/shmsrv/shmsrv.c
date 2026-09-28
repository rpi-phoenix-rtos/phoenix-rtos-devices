/*
 * Phoenix-RTOS
 *
 * shmsrv - anonymous shared memory objects under /shm (memfd_create() backing)
 *
 * Wayland clients hand the compositor their pixels as a descriptor of an
 * anonymous memory file (wl_shm pools); both sides mmap() it. Phoenix has no
 * memfd/shm_open, and a file on the RAM /tmp is not safe for this: the kernel's
 * page cache of a file outlives the file (it is keyed by the dummyfs id, which
 * dummyfs reuses), and a page first touched after both sides closed the
 * descriptor would be fetched from a destroyed file. shmsrv hands out objects
 * whose pages are the server's MAP_CONTIGUOUS memory exported with memExport()
 * (E1): every mmap() of the descriptor, in any process, maps the same physical
 * pages; ids are never reused; the pages are freed with the last mapping.
 * Protocol: shm_proto.h.
 *
 * Usage: shmsrv [-f] [-v]      run the server (detaches unless -f)
 *        shmsrv -s | -q        print the server's stats / ask it to quit
 * Every line starts with "SHMSRV " (grading).
 *
 * Copyright 2026 Phoenix Systems
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <sys/file.h>
#include <sys/mman.h>
#include <sys/msg.h>
#include <sys/stat.h>

#include "shm_proto.h"

#define SHM_PAGE 4096u

typedef struct shm_obj {
	struct shm_obj *next;
	uint32_t id;
	uint32_t opens;
	size_t size;     /* ftruncate() size */
	size_t exported; /* bytes currently exported (page-rounded size), 0 = none */
	size_t cap;      /* bytes mapped (power of two of pages), 0 = no memory yet */
	void *va;
} shm_obj_t;

static struct {
	uint32_t port;
	uint32_t next_id;
	shm_obj_t *objs;
	uint32_t live;
	uint64_t bytes;
	uint64_t created;
	uint64_t destroyed;
	int verbose;
	int quit;
} srv = { .next_id = 1u };


static void shm_log(const char *fmt, ...)
{
	va_list ap;

	printf("SHMSRV ");
	va_start(ap, fmt);
	vprintf(fmt, ap);
	va_end(ap);
	printf("\n");
	fflush(stdout);
}


static shm_obj_t *obj_find(uint32_t id)
{
	shm_obj_t *o;

	for (o = srv.objs; o != NULL; o = o->next) {
		if (o->id == id) {
			return o;
		}
	}
	return NULL;
}


static oid_t obj_oid(const shm_obj_t *o)
{
	oid_t oid;

	oid.port = srv.port;
	oid.id = o->id;
	return oid;
}


static void obj_destroy(shm_obj_t *o)
{
	shm_obj_t **pp;
	oid_t oid = obj_oid(o);

	/* Withdraw first: from here no new mmap() can find the window; existing
	 * mappings keep the pages (E1 lifetime rules). */
	if (o->exported != 0u) {
		(void)memUnexport(&oid);
	}
	if (o->cap != 0u) {
		(void)munmap(o->va, o->cap);
		srv.bytes -= o->cap;
	}
	for (pp = &srv.objs; *pp != NULL; pp = &(*pp)->next) {
		if (*pp == o) {
			*pp = o->next;
			break;
		}
	}
	if (srv.verbose) {
		shm_log("destroy id=%u size=%zu cap=%zu", (unsigned)o->id, o->size, o->cap);
	}
	srv.live--;
	srv.destroyed++;
	free(o);
}


static size_t round_pages(size_t n)
{
	return (n + SHM_PAGE - 1u) & ~(size_t)(SHM_PAGE - 1u);
}


/* Capacity: a power of two, at least SHM_MIN_CAP, so that pools that grow in
 * small steps (libwayland-cursor's theme pool) usually grow in place. */
#define SHM_MIN_CAP (1024u * 1024u)

static size_t pow2_pages(size_t n)
{
	size_t c = SHM_MIN_CAP;

	while (c < n) {
		c <<= 1;
	}
	return c;
}


/* ftruncate(): allocate on the first call, re-export within the capacity later. */
static int obj_truncate(shm_obj_t *o, size_t size)
{
	oid_t oid = obj_oid(o);
	size_t exp = round_pages(size);
	int rc;

	if (size > SHMSRV_MAX_BYTES) {
		return -EFBIG;
	}
	if ((o->cap == 0u) && (size != 0u)) {
		size_t cap = pow2_pages(exp);
		void *va = mmap(NULL, cap, PROT_READ | PROT_WRITE, MAP_ANONYMOUS | MAP_CONTIGUOUS, -1, 0);
		if (va == MAP_FAILED) {
			shm_log("FAIL alloc id=%u size=%zu cap=%zu errno=%d", (unsigned)o->id, size, cap, errno);
			return -ENOMEM;
		}
		memset(va, 0, cap); /* memfd contents start zeroed */
		o->va = va;
		o->cap = cap;
		srv.bytes += cap;
	}
	else if (exp > o->cap) {
		/* The pages cannot move: other processes may map them already. */
		if (srv.verbose) {
			shm_log("refuse grow id=%u size=%zu cap=%zu", (unsigned)o->id, size, o->cap);
		}
		return -EFBIG;
	}
	if ((o->cap != 0u) && (size < o->size)) {
		memset((char *)o->va + size, 0, o->size - size); /* shrunk bytes read back as zero */
	}
	if (exp != o->exported) {
		if (o->exported != 0u) {
			(void)memUnexport(&oid);
			o->exported = 0u;
		}
		if (exp != 0u) {
			rc = memExport(&oid, o->va, exp);
			if (rc < 0) {
				shm_log("FAIL memExport id=%u size=%zu rc=%d", (unsigned)o->id, exp, rc);
				return rc;
			}
			o->exported = exp;
		}
	}
	o->size = size;
	if (srv.verbose) {
		shm_log("truncate id=%u size=%zu exported=%zu cap=%zu", (unsigned)o->id, size, o->exported, o->cap);
	}
	return 0;
}


static int attr_all(msg_t *msg, const shm_obj_t *o)
{
	struct _attrAll *a = msg->o.data;
	long long now = (long long)time(NULL);

	if ((a == NULL) || (msg->o.size < sizeof(*a))) {
		return -EINVAL;
	}
	memset(a, 0, sizeof(*a));
	a->mode.val = (o == NULL) ? (S_IFDIR | 0555) : (S_IFREG | 0600);
	a->size.val = (o == NULL) ? 0 : (long long)o->size;
	a->blocks.val = (o == NULL) ? 0 : (long long)(o->cap / 512u);
	a->ioblock.val = SHM_PAGE;
	a->type.val = (o == NULL) ? otDir : otFile;
	a->port.val = srv.port;
	a->dev.val = srv.port;
	a->links.val = 1;
	a->cTime.val = now;
	a->mTime.val = now;
	a->aTime.val = now;
	a->pollStatus.err = -EINVAL;
	a->eventMask.err = -EINVAL;
	return 0;
}


static void handle_devctl(msg_t *msg)
{
	shmsrv_create_req_t req;
	shmsrv_create_rsp_t rsp;
	shm_obj_t *o;

	memcpy(&req, msg->i.raw, sizeof(req));
	memset(&rsp, 0, sizeof(rsp));
	if (msg->oid.id != 0u) {
		rsp.err = -EINVAL;
	}
	else if (req.op == SHMSRV_OP_CREATE) {
		if (req.proto != SHMSRV_PROTO) {
			rsp.err = -EPROTO;
		}
		else if ((o = calloc(1, sizeof(*o))) == NULL) {
			rsp.err = -ENOMEM;
		}
		else {
			o->id = srv.next_id++;
			o->next = srv.objs;
			srv.objs = o;
			srv.live++;
			srv.created++;
			rsp.id = o->id;
			if (srv.verbose) {
				shm_log("create id=%u pid=%d", (unsigned)o->id, msg->pid);
			}
		}
	}
	else if ((req.op == SHMSRV_OP_STATS) || (req.op == SHMSRV_OP_QUIT)) {
		rsp.live = srv.live;
		rsp.bytes = srv.bytes;
		rsp.id = srv.next_id - 1u;
		if (req.op == SHMSRV_OP_QUIT) {
			srv.quit = 1;
		}
	}
	else {
		rsp.err = -ENOSYS;
	}
	memcpy(msg->o.raw, &rsp, sizeof(rsp));
	msg->o.err = rsp.err;
}


static void handle(msg_t *msg)
{
	shm_obj_t *o = (msg->oid.id != 0u) ? obj_find((uint32_t)msg->oid.id) : NULL;
	int is_root = (msg->oid.id == 0u);
	char name[16];
	unsigned long id;
	char *end;
	size_t len, off, n;

	switch (msg->type) {
		case mtLookup:
			len = (msg->i.data != NULL) ? strnlen(msg->i.data, msg->i.size) : 0u;
			if ((len == 0u) || (len >= sizeof(name))) {
				msg->o.err = -ENOENT;
				break;
			}
			memcpy(name, msg->i.data, len);
			name[len] = '\0';
			id = strtoul(name, &end, 10);
			if ((*end != '\0') || (id == 0u) || (obj_find((uint32_t)id) == NULL)) {
				msg->o.err = -ENOENT;
				break;
			}
			msg->o.lookup.fil.port = srv.port;
			msg->o.lookup.fil.id = id;
			msg->o.lookup.dev = msg->o.lookup.fil;
			msg->o.err = (int)len;
			break;

		case mtOpen:
			/* 0, never an id: a positive reply would rewrite the descriptor's oid. */
			if (o != NULL) {
				o->opens++;
			}
			msg->o.err = (is_root || (o != NULL)) ? 0 : -ENOENT;
			break;

		case mtClose:
			if (o != NULL) {
				if (o->opens > 0u) {
					o->opens--;
				}
				if (o->opens == 0u) {
					obj_destroy(o);
				}
			}
			msg->o.err = 0;
			break;

		case mtTruncate:
			msg->o.err = (o != NULL) ? obj_truncate(o, msg->i.io.len) : -EINVAL;
			break;

		case mtRead:
			if (o == NULL) {
				msg->o.err = -EINVAL;
				break;
			}
			off = (size_t)msg->i.io.offs;
			n = (off < o->size) ? (o->size - off) : 0u;
			n = (n < msg->o.size) ? n : msg->o.size;
			if (n != 0u) {
				memcpy(msg->o.data, (char *)o->va + off, n);
			}
			msg->o.err = (int)n;
			break;

		case mtWrite:
			if (o == NULL) {
				msg->o.err = -EINVAL;
				break;
			}
			/* no implicit growth: the object's size is set by ftruncate() */
			off = (size_t)msg->i.io.offs;
			n = (off < o->size) ? (o->size - off) : 0u;
			n = (n < msg->i.size) ? n : msg->i.size;
			if (n != 0u) {
				memcpy((char *)o->va + off, msg->i.data, n);
			}
			msg->o.err = (n != 0u) ? (int)n : ((msg->i.size == 0u) ? 0 : -EFBIG);
			break;

		case mtGetAttr:
			if (!is_root && (o == NULL)) {
				msg->o.err = -ENOENT;
			}
			else if (msg->i.attr.type == atMode) {
				msg->o.attr.val = is_root ? (S_IFDIR | 0555) : (S_IFREG | 0600);
				msg->o.err = 0;
			}
			else if (msg->i.attr.type == atType) {
				msg->o.attr.val = is_root ? otDir : otFile;
				msg->o.err = 0;
			}
			else {
				/* atSize in particular (shm_proto.h: no shadow objects) */
				msg->o.err = -EINVAL;
			}
			break;

		case mtGetAttrAll:
			msg->o.err = (!is_root && (o == NULL)) ? -ENOENT : attr_all(msg, o);
			break;

		case mtDevCtl:
			handle_devctl(msg);
			break;

		default:
			msg->o.err = -ENOSYS;
			break;
	}
}


/* Client side of -s / -q: one devctl to the running server. */
static int client_op(uint32_t op)
{
	oid_t oid;
	msg_t msg;
	shmsrv_create_req_t req = { .op = op, .proto = SHMSRV_PROTO };
	shmsrv_create_rsp_t rsp;

	if (lookup(SHMSRV_NS, NULL, &oid) < 0) {
		shm_log("%s rc=-1 err=no-server", (op == SHMSRV_OP_QUIT) ? "quit" : "stats");
		return 1;
	}
	memset(&msg, 0, sizeof(msg));
	msg.type = mtDevCtl;
	msg.oid.port = oid.port;
	msg.oid.id = 0;
	memcpy(msg.i.raw, &req, sizeof(req));
	if (msgSend(oid.port, &msg) < 0) {
		shm_log("%s rc=-1 err=send", (op == SHMSRV_OP_QUIT) ? "quit" : "stats");
		return 1;
	}
	memcpy(&rsp, msg.o.raw, sizeof(rsp));
	shm_log("%s rc=%d live=%u bytes=%llu ids=%u", (op == SHMSRV_OP_QUIT) ? "quit" : "stats", (int)rsp.err,
		(unsigned)rsp.live, (unsigned long long)rsp.bytes, (unsigned)rsp.id);
	return (rsp.err == 0) ? 0 : 1;
}


int main(int argc, char **argv)
{
	msg_t msg;
	msg_rid_t rid;
	oid_t dev;
	int c, foreground = 0, readyfd = -1;

	while ((c = getopt(argc, argv, "fvsq")) != -1) {
		switch (c) {
			case 'f': foreground = 1; break;
			case 'v': srv.verbose = 1; break;
			case 's': return client_op(SHMSRV_OP_STATS);
			case 'q': return client_op(SHMSRV_OP_QUIT);
			default:
				fprintf(stderr, "usage: %s [-f] [-v] | -s | -q\n", argv[0]);
				return 2;
		}
	}

	if (lookup(SHMSRV_NS, NULL, &dev) >= 0) {
		memset(&msg, 0, sizeof(msg));
		msg.type = mtGetAttr;
		msg.oid = dev;
		msg.i.attr.type = atMode;
		if (msgSend(dev.port, &msg) == EOK) {
			shm_log("srv FAIL %s is served already", SHMSRV_NS);
			return 1;
		}
		(void)portUnregister(SHMSRV_NS); /* left by a dead server */
	}

	/* Detach before the port exists (psh has no '&'); the parent waits for 'R'. */
	if (!foreground) {
		int pfd[2];
		pid_t pid;
		char r = 0;

		if (pipe(pfd) < 0) {
			shm_log("srv FAIL pipe errno=%d", errno);
			return 1;
		}
		fflush(stdout);
		pid = fork();
		if (pid < 0) {
			shm_log("srv FAIL fork errno=%d", errno);
			return 1;
		}
		if (pid > 0) {
			close(pfd[1]);
			if ((read(pfd[0], &r, 1) != 1) || (r != 'R')) {
				shm_log("srv FAIL child did not come up");
				return 1;
			}
			shm_log("srv detached pid=%d", (int)pid);
			_exit(0);
		}
		close(pfd[0]);
		readyfd = pfd[1];
	}

	if (portCreate(&srv.port) != EOK) {
		shm_log("srv FAIL portCreate");
		return 1;
	}
	dev.port = srv.port;
	dev.id = 0;
	if (portRegister(srv.port, SHMSRV_NS, &dev) < 0) {
		shm_log("srv FAIL portRegister %s", SHMSRV_NS);
		return 1;
	}
	shm_log("srv ready ns=%s port=%u proto=%u max_object=%u", SHMSRV_NS, (unsigned)srv.port, SHMSRV_PROTO,
		SHMSRV_MAX_BYTES);
	if (readyfd >= 0) {
		(void)write(readyfd, "R", 1);
		close(readyfd);
	}

	while (!srv.quit) {
		if (msgRecv(srv.port, &msg, &rid) < 0) {
			continue;
		}
		handle(&msg);
		(void)msgRespond(srv.port, &msg, rid);
	}
	shm_log("srv quit live=%u bytes=%llu created=%llu destroyed=%llu", (unsigned)srv.live,
		(unsigned long long)srv.bytes, (unsigned long long)srv.created, (unsigned long long)srv.destroyed);
	(void)portUnregister(SHMSRV_NS);
	/* live objects: port release withdraws their exports, mappings keep the pages */
	return 0;
}
