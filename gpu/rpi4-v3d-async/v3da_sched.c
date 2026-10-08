/*
 * Phoenix-RTOS
 *
 * Raspberry Pi 4 (BCM2711) V3D 4.2 asynchronous render server - queues, fences,
 * the event thread, parked waits and syncobjs
 *
 * The event thread is the only place where jobs complete and parked requests are
 * answered. It wakes on the V3D interrupt (the handler broadcasts srv.cond), on a
 * dispatch thread's kick, or on its own timeout (earliest job/wait deadline, the
 * poll period in poll mode). In IRQ mode it drains the event words the handler
 * filled; in poll mode it reads the status registers itself through the same
 * service routine - so the completion path is identical in both modes.
 *
 * The jobs themselves (submission, kicks, completions, watchdog, reset,
 * overflow memory, present) live in v3da_jobs.c; this file owns the fence page,
 * parked waits, syncobjs and the event-thread loop that drives v3da_jobs.c.
 *
 * Copyright 2026 Phoenix Systems
 * Author: Witold Bołt
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <sys/mman.h>
#include <sys/msg.h>
#include <sys/threads.h>

#include "v3da.h"
#include "v3da_regs.h"


#define IDLE_TIMEOUT_US   100000u   /* heartbeat granularity when nothing is pending */
#define IRQ_BACKSTOP_US   10000u    /* IRQ mode: bounds a lost wake-up */
#define ERR_SEQ(q, s)     (((uint64_t)(q) << 56) | ((s) & 0x00ffffffffffffffULL))

static int kick_pending;


uint64_t v3da_now_us(void)
{
	struct timespec ts;

	(void)clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint64_t)ts.tv_sec * 1000000u + (uint64_t)ts.tv_nsec / 1000u;
}


void v3da_kick_event_thread(void)
{
	kick_pending = 1;
	(void)condSignal(srv.cond);
}


/* ========================================================================= */
/* Fence page                                                                 */
/* ========================================================================= */

int v3da_sched_init(void)
{
	void *p;

	/* One cached page, MAP_CONTIGUOUS so its PA can never change under a client's
	 * MAP_PHYSMEM view. Clients map it cached too: one memory type per page. */
	p = mmap(NULL, V3DA_FENCE_PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_ANONYMOUS | MAP_CONTIGUOUS, -1, 0);
	if (p == MAP_FAILED) {
		return -ENOMEM;
	}
	memset(p, 0, V3DA_FENCE_PAGE_SIZE);
	srv.fp = p;
	srv.fp->hdr.magic = V3DA_FENCE_MAGIC;
	srv.fp->hdr.version = V3DA_PROTO_BASE;   /* the page layout of proto 2, unchanged since */
	srv.fp->hdr.nslots = V3DA_FENCE_NSLOTS;
	srv.fp->hdr.slot_size = V3DA_FENCE_SLOT_SIZE;
	srv.fp->hdr.server_pid = (uint32_t)getpid();
	__atomic_thread_fence(__ATOMIC_RELEASE);
	srv.fp_pa = (uintptr_t)va2pa(p);   /* after the first write: the page is present */
	srv.acct_t_us = v3da_now_us();
	srv.acct_t0_us = srv.acct_t_us;
	return v3da_ovf_init();
}


static int slot_gen_ok(const v3da_fence_t *f)
{
	return ((uint32_t)v3da_load64(&srv.fp->slot[f->slot].gen) == f->gen) ? 1 : 0;
}


/* Signalled? A fence of a reassigned slot is signalled by construction: a slot is
 * only reassigned after its last job completed. */
int v3da_fence_signaled(const v3da_fence_t *f, int *error)
{
	uint64_t done, es;

	if (error != NULL) {
		*error = 0;
	}
	if ((f->slot >= V3DA_FENCE_NSLOTS) || (f->queue >= V3DA_Q_COUNT)) {
		return 1;
	}
	if (slot_gen_ok(f) == 0) {
		return 1;
	}
	done = v3da_load64(&srv.fp->slot[f->slot].completed[f->queue]);
	if (done < f->seqno) {
		return 0;
	}
	es = v3da_load64(&srv.fp->slot[f->slot].error_seq);
	if ((error != NULL) && (es != 0u) && ((es >> 56) == f->queue) && ((es & 0x00ffffffffffffffULL) >= f->seqno)) {
		*error = 1;
	}
	return 1;
}


/* A wait may only name a fence that has been handed out. */
int v3da_fence_valid(const v3da_client_t *c, const v3da_fence_t *f)
{
	const v3da_client_t *owner;

	(void)c;
	if ((f->slot >= V3DA_FENCE_NSLOTS) || (f->queue >= V3DA_Q_COUNT) || (f->seqno == 0u)) {
		return 0;
	}
	owner = &srv.clients[f->slot];
	if ((owner->used == 0) || (slot_gen_ok(f) == 0)) {
		return 1;   /* stale: signalled */
	}
	return (f->seqno <= owner->next_seq[f->queue]) ? 1 : 0;
}


void v3da_fence_complete(const v3da_job_t *j, int error)
{
	uint64_t seq = j->fence.seqno;
	uint32_t slot = j->fence.slot;
	int q = j->queue;

	if ((srv.clients[slot].used == 0) && (srv.dead_flush[slot][q] > seq)) {
		seq = srv.dead_flush[slot][q];   /* discarded jobs of a dead client complete with it */
		error = 1;
	}
	if (error != 0) {
		v3da_store64(&srv.fp->slot[slot].error_seq, ERR_SEQ(q, seq));
	}
	v3da_store64(&srv.fp->slot[slot].completed[q], seq);
	v3da_store64(&srv.fp->hdr.hw_completed[q], v3da_load64(&srv.fp->hdr.hw_completed[q]) + 1u);
}


/* A slot may be reassigned only when no job of its previous owner is in flight
 * (its queued jobs were discarded at client death). */
int v3da_slot_busy(uint32_t slot)
{
	int q;

	for (q = 0; q < V3DA_Q_COUNT; q++) {
		if ((srv.q[q].active != NULL) && (srv.q[q].active->fence.slot == slot)) {
			return 1;
		}
	}
	return 0;
}


/* New owner: zero the row, then publish the new generation. A reader holding an
 * old-generation fence sees the mismatch and treats it as signalled - correct,
 * because every job of the previous owner has completed (v3da_slot_busy). */
void v3da_slot_assign(uint32_t slot)
{
	int q;

	for (q = 0; q < V3DA_Q_COUNT; q++) {
		v3da_store64(&srv.fp->slot[slot].completed[q], 0u);
		srv.dead_flush[slot][q] = 0u;
	}
	v3da_store64(&srv.fp->slot[slot].error_seq, 0u);
	srv.slot_gen[slot]++;
	v3da_store64(&srv.fp->slot[slot].gen, srv.slot_gen[slot]);
}


/* ========================================================================= */
/* Parked waits                                                               */
/* ========================================================================= */

static void wait_unlink(v3da_wait_t *w)
{
	if (w->prev != NULL) {
		w->prev->next = w->next;
	}
	else {
		srv.waits = w->next;
	}
	if (w->next != NULL) {
		w->next->prev = w->prev;
	}
	w->next = NULL;
	w->prev = NULL;
	srv.nparked--;
	if ((w->client >= 1u) && (w->client <= V3DA_MAX_CLIENTS) && (srv.clients[w->client - 1u].nparked > 0u)) {
		srv.clients[w->client - 1u].nparked--;
	}
}


/* The ONE way a parked request leaves srv.waits: under srv.lock, onto the caller's
 * local answer list, with its response filled in. Whoever claims it answers it,
 * exactly once, after dropping the lock. */
static void wait_claim(v3da_wait_t *w, int err, v3da_wait_t **answer)
{
	v3da_resp_t *r = (v3da_resp_t *)w->msg.o.raw;

	wait_unlink(w);
	if (w->kind == V3DA_WAIT_FENCE) {
		r->u.fence_wait.echo = (uint32_t)w->fence[0].seqno;
	}
	r->err = err;
	w->msg.o.err = EOK;
	w->next = *answer;
	*answer = w;
}


/* Evaluate one wait: 1 = satisfied (resp filled), 0 = not yet, <0 = fail now. */
static int wait_eval(v3da_wait_t *w)
{
	v3da_resp_t *r = (v3da_resp_t *)w->msg.o.raw;
	v3da_client_t *c;
	v3da_syncobj_t *s;
	uint32_t i, nsig = 0u, first = 0xffffffffu;
	int err = 0, sig;

	switch (w->kind) {
		case V3DA_WAIT_FENCE:
			if (v3da_fence_signaled(&w->fence[0], &err) == 0) {
				return 0;
			}
			r->u.fence_wait.completed = v3da_load64(&srv.fp->slot[w->fence[0].slot].completed[w->fence[0].queue]);
			r->u.fence_wait.error = (uint32_t)err;
			r->u.fence_wait.echo = (uint32_t)w->fence[0].seqno;
			return 1;

		case V3DA_WAIT_BO:
			for (i = 0u; i < w->n; i++) {
				if ((w->fence[i].seqno != 0u) && (v3da_fence_signaled(&w->fence[i], NULL) == 0)) {
					return 0;
				}
			}
			return 1;

		case V3DA_WAIT_SYNCOBJ:
			c = &srv.clients[w->client - 1u];
			for (i = 0u; i < w->n; i++) {
				s = v3da_syncobj_get(c, w->sync_handle[i]);
				if (s == NULL) {
					return -EINVAL;   /* destroyed while waited on */
				}
				if (s->state == V3DA_SYNC_SIGNALED) {
					sig = 1;
				}
				else if (s->state == V3DA_SYNC_FENCE) {
					sig = v3da_fence_signaled(&s->fence, NULL);
				}
				else {
					if (w->for_submit == 0) {
						return -EINVAL;   /* DRM: waiting on an empty syncobj */
					}
					sig = 0;
				}
				if (sig != 0) {
					nsig++;
					if (first == 0xffffffffu) {
						first = i;
					}
				}
			}
			if (((w->all != 0) && (nsig == w->n)) || ((w->all == 0) && (nsig > 0u))) {
				r->u.syncobj.first = (first == 0xffffffffu) ? 0u : first;
				return 1;
			}
			return 0;

		default:
			return -EINVAL;
	}
}


int v3da_wait_park(v3da_client_t *c, int kind, msg_t *msg, msg_rid_t rid,
	const v3da_fence_t *fences, const uint32_t *handles, uint32_t n, int all,
	int for_submit, uint32_t timeout_ms)
{
	v3da_wait_t *w;
	v3da_resp_t *r;
	int rc;

	if (n > V3DA_SYNCOBJ_WAIT_MAX) {
		return -EINVAL;
	}
	w = calloc(1, sizeof(*w));
	if (w == NULL) {
		return -ENOMEM;
	}
	w->kind = kind;
	w->client = c->id;
	w->msg = *msg;
	w->rid = rid;
	w->port = srv.rx_port;   /* answered on the port it arrived on (rids are per port) */
	w->n = n;
	w->all = all;
	w->for_submit = for_submit;
	if (fences != NULL) {
		memcpy(w->fence, fences, n * sizeof(*fences));
	}
	if (handles != NULL) {
		memcpy(w->sync_handle, handles, n * sizeof(*handles));
	}
	r = (v3da_resp_t *)w->msg.o.raw;
	memset(r, 0, sizeof(*r));
	r->op = ((const v3da_req_t *)msg->i.raw)->op;

	rc = wait_eval(w);
	if (rc != 0) {
		/* Satisfied or failed at once: the caller answers with its own msg. */
		memcpy(msg->o.raw, r, sizeof(*r));
		free(w);
		return (rc > 0) ? 1 : rc;
	}
	if (timeout_ms == 0u) {
		free(w);
		return -ETIMEDOUT;
	}
	if (timeout_ms > V3DA_WAIT_MAX_MS) {
		timeout_ms = V3DA_WAIT_MAX_MS;
	}
	w->deadline_us = v3da_now_us() + (uint64_t)timeout_ms * 1000u;

	w->prev = NULL;
	w->next = srv.waits;
	if (srv.waits != NULL) {
		srv.waits->prev = w;
	}
	srv.waits = w;
	c->nparked++;
	srv.nparked++;
	if (srv.nparked > srv.parked_max) {
		srv.parked_max = srv.nparked;
	}
	v3da_kick_event_thread();
	return 0;
}


/* Claim every wait that is satisfied, failed or past its deadline. */
static void waits_scan(uint64_t now, v3da_wait_t **answer)
{
	v3da_wait_t *w = srv.waits, *next;
	int rc;

	while (w != NULL) {
		next = w->next;
		rc = wait_eval(w);
		if (rc > 0) {
			wait_claim(w, 0, answer);
		}
		else if (rc < 0) {
			wait_claim(w, rc, answer);
		}
		else if (now >= w->deadline_us) {
			srv.timeouts++;
			wait_claim(w, -ETIMEDOUT, answer);
		}
		w = next;
	}
}


void v3da_waits_client_gone(uint32_t client, v3da_wait_t **answer)
{
	v3da_wait_t *w = srv.waits, *next;

	while (w != NULL) {
		next = w->next;
		if (w->client == client) {
			wait_claim(w, -EPIPE, answer);
		}
		w = next;
	}
}


void v3da_waits_all(v3da_wait_t **answer, int err)
{
	while (srv.waits != NULL) {
		wait_claim(srv.waits, err, answer);
	}
}


/* UNLOCKED. proc_respond reschedules after every response, so answering under
 * srv.lock would convoy the dispatch threads behind the event thread. */
void v3da_waits_answer(v3da_wait_t *list)
{
	v3da_wait_t *next;

	while (list != NULL) {
		next = list->next;
		(void)msgRespond((list->port != 0u) ? list->port : srv.port, &list->msg, list->rid);
		free(list);
		list = next;
	}
}


/* ========================================================================= */
/* Syncobjs                                                                   */
/* ========================================================================= */

v3da_syncobj_t *v3da_syncobj_get(v3da_client_t *c, uint32_t handle)
{
	if ((handle == 0u) || (handle > V3DA_MAX_SYNCOBJS) || (c->sync[handle - 1u].used == 0)) {
		return NULL;
	}
	return &c->sync[handle - 1u];
}


int v3da_syncobj_create(v3da_client_t *c, uint32_t flags, uint32_t *handle)
{
	uint32_t i;

	for (i = 0u; i < V3DA_MAX_SYNCOBJS; i++) {
		if (c->sync[i].used == 0) {
			memset(&c->sync[i], 0, sizeof(c->sync[i]));
			c->sync[i].used = 1;
			c->sync[i].state = ((flags & V3DA_SYNCOBJ_CREATE_SIGNALED) != 0u) ? V3DA_SYNC_SIGNALED : V3DA_SYNC_EMPTY;
			*handle = i + 1u;
			return 0;
		}
	}
	return -ENOMEM;
}


int v3da_syncobj_destroy(v3da_client_t *c, uint32_t handle)
{
	v3da_syncobj_t *s = v3da_syncobj_get(c, handle);

	if (s == NULL) {
		return -EINVAL;
	}
	s->used = 0;
	v3da_kick_event_thread();   /* a waiter on it must now fail */
	return 0;
}


int v3da_syncobj_reset(v3da_client_t *c, uint32_t handle)
{
	v3da_syncobj_t *s = v3da_syncobj_get(c, handle);

	if (s == NULL) {
		return -EINVAL;
	}
	s->state = V3DA_SYNC_EMPTY;
	memset(&s->fence, 0, sizeof(s->fence));
	return 0;
}


int v3da_syncobj_signal(v3da_client_t *c, uint32_t handle)
{
	v3da_syncobj_t *s = v3da_syncobj_get(c, handle);

	if (s == NULL) {
		return -EINVAL;
	}
	s->state = V3DA_SYNC_SIGNALED;
	v3da_kick_event_thread();
	return 0;
}


/* Sync-file emulation: the library exported a fence snapshot as an fd and now
 * imports it into (usually a temporary) syncobj. */
int v3da_syncobj_import(v3da_client_t *c, uint32_t handle, const v3da_fence_t *f)
{
	v3da_syncobj_t *s = v3da_syncobj_get(c, handle);

	if (s == NULL) {
		return -EINVAL;
	}
	if (f->seqno == 0u) {
		s->state = V3DA_SYNC_SIGNALED;   /* an "already signalled" snapshot */
	}
	else if (v3da_fence_valid(c, f) != 0) {
		s->state = V3DA_SYNC_FENCE;
		s->fence = *f;
	}
	else {
		return -EINVAL;
	}
	v3da_kick_event_thread();
	return 0;
}


int v3da_syncobj_query(v3da_client_t *c, uint32_t handle, v3da_syncobj_resp_t *out)
{
	v3da_syncobj_t *s = v3da_syncobj_get(c, handle);

	if (s == NULL) {
		return -EINVAL;
	}
	out->handle = handle;
	out->state = (uint32_t)s->state;
	out->fence = s->fence;
	return 0;
}


/* ========================================================================= */
/* The event thread                                                           */
/* ========================================================================= */

static uint32_t next_timeout_us(uint64_t now)
{
	uint64_t t = IDLE_TIMEOUT_US, d;
	const v3da_wait_t *w;

	d = v3da_jobs_next_deadline(now);
	if (d < t) {
		t = d;
	}
	for (w = srv.waits; w != NULL; w = w->next) {
		d = (w->deadline_us > now) ? (w->deadline_us - now) : 1u;
		if (d < t) {
			t = d;
		}
	}
	if ((srv.scan.nq != 0u) && (t > 1000u)) {
		t = 1000u;   /* a fence-gated flip: re-check soon (completions also kick us) */
	}
	/* IRQ mode: the handler's broadcast can land between our "no events" check and
	 * condWait (it does not take srv.lock), so a lost wake-up must be bounded even
	 * when nothing is in flight - the self-test raises interrupts on idle queues.
	 * The same bound keeps the 125 ms job watchdog honest. */
	if (srv.hw.irq_on != 0) {
		if (IRQ_BACKSTOP_US < t) {
			t = IRQ_BACKSTOP_US;
		}
	}
	else if ((v3da_jobs_hw_busy() != 0) && (srv.poll_us < t)) {
		t = srv.poll_us;
	}
	return (t == 0u) ? 1u : (uint32_t)t;   /* condWait: 0 would mean "forever" */
}


void v3da_event_thread(void *arg)
{
	v3da_wait_t *answer;
	uint64_t now;
	uint32_t core, hub, storm;

	(void)arg;

	(void)mutexLock(srv.lock);
	while (srv.quit == 0) {
		now = v3da_now_us();
		if ((kick_pending == 0) && (srv.hw.ev_core == 0u) && (srv.hw.ev_hub == 0u)) {
			(void)condWait(srv.cond, srv.lock, next_timeout_us(now));
		}
		kick_pending = 0;
		now = v3da_now_us();

		srv.loops++;
		v3da_store64(&srv.fp->hdr.heartbeat, srv.loops);
		__atomic_store_n(&srv.hw.irq_count_snap, __atomic_load_n(&srv.hw.irq_count, __ATOMIC_RELAXED),
			__ATOMIC_RELAXED);
		__atomic_store_n(&srv.hw.irq_spurious, 0u, __ATOMIC_RELAXED);

		storm = __atomic_load_n(&srv.hw.storm, __ATOMIC_ACQUIRE);
		if ((storm != 0u) && (srv.hw.irq_on != 0)) {
			printf("V3DA srv IRQ STORM (%s) after %u interrupts - masked, falling back to polling\n",
				(storm == 1u) ? "spurious" : "burst", srv.hw.irq_count);
			v3da_hw_irq_disable(&srv.hw);
			srv.fp->hdr.flags = (srv.fp->hdr.flags & ~V3DA_FP_IRQ) | V3DA_FP_STORM;
		}

		if ((srv.hw.irq_on == 0) && (v3da_jobs_hw_busy() != 0)) {
			v3da_hw_poll_status(&srv.hw);
		}
		core = __atomic_exchange_n(&srv.hw.ev_core, 0u, __ATOMIC_ACQ_REL);
		hub = __atomic_exchange_n(&srv.hw.ev_hub, 0u, __ATOMIC_ACQ_REL);
		v3da_jobs_events(core, hub, now);
		v3da_jobs_tick(now);
		v3da_bo_quarantine_poll();
		v3da_bo_pool_tick(now, 0);
		v3da_sched_run();

		answer = NULL;
		waits_scan(now, &answer);
		if (answer != NULL) {
			(void)mutexUnlock(srv.lock);
			v3da_waits_answer(answer);
			(void)mutexLock(srv.lock);
		}
	}
	(void)mutexUnlock(srv.lock);
	endthread();
}
