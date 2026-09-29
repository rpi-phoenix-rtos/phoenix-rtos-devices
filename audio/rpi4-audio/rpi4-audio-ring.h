/*
 * Phoenix-RTOS
 *
 * Raspberry Pi 4 (BCM2711) PWM audio - streaming ring bookkeeping
 *
 * The DMA engine plays the ring in a loop forever and never stops, so a word it
 * has played stays in the ring and is played AGAIN one lap later unless someone
 * overwrites it. Left alone, whatever the ring held when the writer stopped (an
 * underrun, a pause, a close, a process exit) repeats on the jack until the next
 * program writes.
 *
 * This keeps the invariant that makes that impossible: every word outside the
 * pending region [rd, wr) -- written and not yet played -- holds silence. Each
 * service call learns how far the DMA read cursor moved and overwrites the words
 * it played with silence. The writer only ever writes into that already-silent
 * free region, so it never races the silencing. Once the writer stops, the tail
 * plays out and the DMA then loops over silence.
 *
 * Pure functions on plain memory, no MMIO and no Phoenix API, so the driver and
 * the host test (tools/rpi4-audio-ring-hosttest in the coordination repository)
 * compile the same source.
 *
 * Copyright 2026 Phoenix Systems
 * Author: Witold Bołt
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _RPI4_AUDIO_RING_H_
#define _RPI4_AUDIO_RING_H_

#include <stdint.h>


typedef struct {
	uint32_t size;    /* ring length in words; even, so word parity = PWM channel */
	uint32_t wr;      /* next word the writer fills */
	uint32_t rd;      /* DMA read cursor at the last service */
	uint32_t pending; /* words the engine plays before reaching wr: [rd, wr) */
} audioring_t;


/* audioring_advance() result flags. */
#define AUDIORING_DRAINED 1u /* pending ran out: the tail has played, the ring is all silence */
#define AUDIORING_LAPPED  2u /* the cursor moved a full lap or more between two services */


/* Empty bookkeeping for a ring of `size` words whose contents are all silence. */
static inline void audioring_init(audioring_t *r, uint32_t size)
{
	r->size = size;
	r->wr = 0;
	r->rd = 0;
	r->pending = 0;
}


/* Write `value` to `n` words starting at `start`, wrapping at `size`. Count-based on
 * purpose: a [from, to) pair is ambiguous when from == to (nothing, or the whole
 * ring?). n is clamped to one lap. Returns the number of words written. */
static inline uint32_t audioring_fill(volatile uint32_t *ring, uint32_t size, uint32_t start,
	uint32_t n, uint32_t value)
{
	uint32_t i, pos = start % size;

	if (n > size) {
		n = size;
	}
	for (i = 0; i < n; i++) {
		ring[pos] = value;
		pos = (pos + 1u == size) ? 0u : pos + 1u;
	}
	return n;
}


/* Words the DMA consumed since the last service, given the cursor now and the number
 * of words the elapsed time says it should have consumed (`expected`).
 *
 * The cursor alone cannot tell "moved d words" from "moved d words plus some whole
 * laps": that is the same index. The clock can, as long as it is right to within half
 * a lap (~93 ms at this ring size). Returns `size` for "a full lap or more" -- the
 * caller then knows every pending word has played and does not need the exact count.
 *
 * A cursor that did not move at all is taken at its word, however much time passed:
 * that is a PARKED engine, not one that lapped exactly. Calling it a lap would declare
 * the ring drained on every service, the writer would never see a full ring, and the
 * write path's stall detector (which waits for a full ring that does not drain) could
 * never fire. The cost is a true lap landing on exactly the same word, 1 in `size`. */
static inline uint32_t audioring_consumed(const audioring_t *r, uint32_t cur, uint64_t expected)
{
	uint32_t delta = (cur % r->size + r->size - r->rd) % r->size;

	if ((delta != 0u) && (expected >= (uint64_t)delta + r->size / 2u)) {
		return r->size;
	}
	return delta;
}


/* Free words the writer may fill now. One word of headroom keeps wr from catching up
 * with rd, where "full" and "empty" would have the same indices. */
static inline uint32_t audioring_space(const audioring_t *r)
{
	return r->size - 1u - r->pending;
}


/* The writer filled `n` words at wr. */
static inline void audioring_commit(audioring_t *r, uint32_t n)
{
	r->wr = (r->wr + n) % r->size;
	r->pending += n;
}


/* Called by the writer, after a service and before it writes. If the ring ran dry, the
 * write position is stale -- somewhere the cursor has already passed, i.e. up to a whole
 * lap from being heard -- so move it `lead` words ahead of the cursor. The lead words are
 * silence, and they count as PENDING: the engine has to traverse them before it reaches
 * the first new word, and a drain is only real once it has.
 *
 * Keeps the write position's parity: the FIFO alternates channel 1 / channel 2 per word
 * and the ring size is even, so word parity is the channel, and a stereo stream must
 * resume on the channel it left off. */
static inline void audioring_resync(audioring_t *r, uint32_t lead)
{
	uint32_t wr;

	if (r->pending != 0u) {
		return;
	}
	wr = (r->rd + lead) % r->size;
	if ((((wr + r->size) - r->wr) & 1u) != 0u) {
		wr = (wr + 1u) % r->size;
	}
	r->wr = wr;
	r->pending = (wr + r->size - r->rd) % r->size;
}


/* Account for the DMA having moved to `cur`, having consumed `consumed` words (from
 * audioring_consumed()). Overwrites every word it played with `silence`; `*filled` gets
 * the number of words overwritten. Returns AUDIORING_* flags.
 *
 * With nothing pending the ring is already all silence (the invariant), so an idle
 * ring costs nothing however long the DMA has been looping over it. */
static inline uint32_t audioring_advance(audioring_t *r, volatile uint32_t *ring, uint32_t cur,
	uint32_t consumed, uint32_t silence, uint32_t *filled)
{
	cur %= r->size;
	*filled = 0;

	if (r->pending == 0u) {
		r->rd = cur;
		return 0;
	}
	if (consumed >= r->size) {
		/* Lapped: the pending words have all played, and possibly again. Nothing
		 * in the ring is worth keeping, so silence all of it, ahead of the cursor
		 * first -- that is the part the DMA reaches next. */
		*filled = audioring_fill(ring, r->size, cur, r->size, silence);
		r->rd = cur;
		r->pending = 0;
		return AUDIORING_DRAINED | AUDIORING_LAPPED;
	}

	*filled = audioring_fill(ring, r->size, r->rd, consumed, silence);
	r->rd = cur;
	if (consumed >= r->pending) {
		r->pending = 0;
		return AUDIORING_DRAINED;
	}
	r->pending -= consumed;
	return 0;
}


/* Words that do not hold `silence` (a self-check for the log and the stats). */
static inline uint32_t audioring_countNot(const volatile uint32_t *ring, uint32_t size,
	uint32_t silence)
{
	uint32_t i, n = 0;

	for (i = 0; i < size; i++) {
		if (ring[i] != silence) {
			n++;
		}
	}
	return n;
}


#endif
