/*
 * Phoenix-RTOS
 *
 * Raspberry Pi 4 (BCM2711) VideoCore property-mailbox client library
 *
 * Thin IPC veneer over the rpi4-vcmbox server (/dev/vcmbox): packs a property
 * call into msg.i.raw, sends it as an mtDevCtl message, and copies the firmware
 * response words out of msg.o.raw. All mailbox protocol + cross-process
 * serialization lives in the server; callers see only this API.
 *
 * Copyright 2026 Phoenix Systems
 * Author: Witold Bołt
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <string.h>
#include <unistd.h>

#include <sys/msg.h>
#include <sys/types.h>

#include "libvcmbox.h"


/* Cached server node. lookup() is retried until the server registers; once
 * resolved the oid is reused for the lifetime of the calling process. */
static struct {
	oid_t oid;
	int resolved;
} vcmbox_common;


/* Block until /dev/vcmbox is registered. The server is launched first among the
 * device apps (see user.plo.yaml), but a client may still race its registration,
 * so this is an unbounded retry-lookup (the standard Phoenix client pattern).
 * The mailbox is required infrastructure; there is no useful fallback if the
 * server never appears. */
static int vcmbox_resolve(void)
{
	if (vcmbox_common.resolved != 0) {
		return 0;
	}

	while (lookup("/dev/vcmbox", NULL, &vcmbox_common.oid) < 0) {
		usleep(10 * 1000);
	}

	vcmbox_common.resolved = 1;
	return 0;
}


int vcmbox_call(uint32_t tag, uint32_t valBufSize, const uint32_t *in, uint32_t nIn,
	uint32_t *out, uint32_t nOut)
{
	msg_t msg;
	vcmbox_req_t *req = (vcmbox_req_t *)msg.i.raw;
	const vcmbox_resp_t *resp = (const vcmbox_resp_t *)msg.o.raw;
	uint32_t i;
	int err;

	if ((nIn > VCMBOX_MAX_WORDS) || (nOut > VCMBOX_MAX_WORDS)) {
		return -EINVAL;
	}

	if (vcmbox_resolve() != 0) {
		return -EIO;
	}

	memset(&msg, 0, sizeof(msg));
	msg.type = mtDevCtl;
	msg.oid = vcmbox_common.oid;

	req->tag = tag;
	req->valBufSize = valBufSize;
	req->nIn = nIn;
	for (i = 0; i < nIn; i++) {
		req->in[i] = in[i];
	}

	err = msgSend(vcmbox_common.oid.port, &msg);
	if (err < 0) {
		return err;
	}

	if (resp->err != 0) {
		return resp->err;
	}

	if (out != NULL) {
		for (i = 0; (i < nOut) && (i < resp->nOut); i++) {
			out[i] = resp->out[i];
		}
	}

	return 0;
}


int vcmbox_prop(uint32_t tag, uint32_t arg_in, uint32_t *out)
{
	/* The property value buffer is 8 bytes = two words: word 0 is the input id
	 * (firmware echoes it back), word 1 is the answer. So request both words and
	 * return word 1 — copying only word 0 would return the echoed input. */
	uint32_t v[2];
	int rc = vcmbox_call(tag, 8u, &arg_in, 1u, v, 2u);

	if ((rc == 0) && (out != NULL)) {
		*out = v[1];
	}
	return rc;
}
